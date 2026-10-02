"""SYSID 事务、状态轮询的对账、存证与自动分析。

**回显核对。** 每条配置命令都要等到飞控回一整份状态报告（以 `SYSID LIMITS` 结尾）
才发下一条；最后一条到齐后逐项比对 `expected`，全部对上才发 `SYSID START`。

**轮询不许冒充回显。** 页面每秒发一次 `SYSID THR?`（只回一行 `SYSID THR`）刷新
遥控器状态，记账是"一次轮询 = 一行 THR"。页面上别的按钮（停止、待机、读回状态、
单独下发）会引出整份状态报告，这些"不属于事务"的报告按发出顺序记账，
回来一份（以 LIMITS 结尾）销一笔——否则点开始那一刻正在路上的报告会被当成配置回显，
核对就对错了行。记账有 2.5 秒过期：回复丢了的话，最坏只是事务超时失败，不会误放行。

**台架几何等机体参数回来再组装。** 杆到质心距离 d = 你量的杆到飞控 + 机体参数里的
飞控到质心（见 `geometry.py`），所以 `SYSID RIG` 排在 PARAM? 之后的第一份状态报告之后才生成。

**自动分析。** FF 轮正常结束 → 先存档 → 存档落盘后自动拟合 → 质量够就自动算候选参数。
舵机单独轮（SERVO，电机不转、全程上锁）同样自动存档、自动拟合，只报舵机与反作用，不给参数。
高度轮（ALT，槽式台架）存档（目录名 alt_ 开头），不跑姿态拟合；break 轮页面当场算离地/滑落阈值，
vel/pos 轮分析离线进行（`alt_config.py`、`alt_breakaway.py`）。
水平槽轮（XY）存档到 data/identification/xy（目录名 xy_ 开头），同样不跑姿态拟合（`xy_config.py`）。
吊绳偏航轮（YAW）存档到 data/identification/yaw（目录名 yaw_ 开头），同样不跑姿态拟合（`yaw_config.py`）。
写 RAM 永远要人点；本文件没有任何写 Flash 的路径。
"""
from __future__ import annotations

import json
import queue
import threading
import time
from dataclasses import asdict
from datetime import datetime
from uuid import uuid4

from ...proto import parse_kv
from ._core import RECORD_VERSIONS
from .alt_config import ALT_MODE_CODE, AltWorkflow, schema_has_height
from .alt_config import UNSUPPORTED_SCHEMA as ALT_UNSUPPORTED_SCHEMA
from .xy_config import UNSUPPORTED_SCHEMA as XY_UNSUPPORTED_SCHEMA
from .xy_config import XY_MODE_CODE, XyWorkflow, schema_has_xy
from .yaw_config import UNSUPPORTED_SCHEMA as YAW_UNSUPPORTED_SCHEMA
from .yaw_config import YAW_MODE_CODE, YawWorkflow, schema_has_yaw
from .analysis import Analysis
from .ram_params import RamParams
from .reasons import explain_error
from .start_config import StartConfig

ECHO_TIMEOUT_MS = 3000
#: 舵机单独模式在固件里的编号（READY/快照里的 mode=3）。
SERVO_MODE_CODE = 3
STALE_REPORT_S = 2.5
THR_POLL = "SYSID THR?"
#: 这些子命令的回复是一整份状态报告（以 SYSID LIMITS 结尾）。
EXACT_ECHO_KEYS = {"mode", "profile", "auto", "repeat", "seed"}
REPORT_SUBCOMMANDS = {"?", "STATUS", "HOLD", "STOP", "RIG", "EXC", "RATE",
                      "INERTIA", "LIMIT", "MODE", "THROTTLE"}


def _attitude_directory():
    try:
        from ....project_paths import ATTITUDE_IDENT_DIR, dated_directory
    except ImportError:
        try:
            from tools.project_paths import ATTITUDE_IDENT_DIR, dated_directory
        except ImportError:
            from project_paths import ATTITUDE_IDENT_DIR, dated_directory
    return dated_directory(ATTITUDE_IDENT_DIR)


def _xy_directory():
    try:
        from ....project_paths import IDENTIFICATION_ROOT, dated_directory
    except ImportError:
        try:
            from tools.project_paths import IDENTIFICATION_ROOT, dated_directory
        except ImportError:
            from project_paths import IDENTIFICATION_ROOT, dated_directory
    return dated_directory(IDENTIFICATION_ROOT / "xy")


def _yaw_directory():
    try:
        from ....project_paths import IDENTIFICATION_ROOT, dated_directory
    except ImportError:
        try:
            from tools.project_paths import IDENTIFICATION_ROOT, dated_directory
        except ImportError:
            from project_paths import IDENTIFICATION_ROOT, dated_directory
    return dated_directory(IDENTIFICATION_ROOT / "yaw")


def produces_report(text: str) -> bool:
    if text == "SYSID?":
        return True
    parts = text.split()
    return len(parts) >= 2 and parts[0] == "SYSID" and parts[1] in REPORT_SUBCOMMANDS


class Workflow(AltWorkflow, XyWorkflow, YawWorkflow, StartConfig, RamParams, Analysis):
    def __init__(self, page):
        self.page = page
        self.pending = []
        self.params = {}
        self.status = {}
        self.expected = {}
        self.kind = None
        self.snapshot = None
        self.end = None
        self.error = ""          # 最近一次失败（任何原因），显示用
        self.data_error = ""     # 本轮数据本身的问题：只有它会让分析/写参数拒绝
        self.notice = ""
        self.session = None
        self.run_id = None
        self.job = None
        self.results = queue.Queue()
        self.saved = None
        self.saving = False
        self.auto_fit = False
        self.closed = False
        self.awaiting = None
        self.ticket = 0
        self.original_gains = None
        self.original_torque = None      # 读原增益时飞控的力矩单位签名（lever_units.torque_record）
        self.original_gains_path = None
        self.conversion_note = ""
        self.candidate_dir = None       # 候选参数来自哪一轮（联合分析时是最近一轮）的目录
        self.restore_source = ""
        self.ram_done = 0
        self.run_note = ""
        self.pivot_request = (None, None)
        self.pivot_to_cg = (None, None)
        self._state_grace = None
        self.throttle = (False, None, 75.0)
        self.stale_reports: list[float] = []
        self.thr_polls: list[float] = []
        self.thr_poll_refused = None
        self.ready = {}
        self.rig_request = None
        self._pump_pending = False
        page.parent.bind("<Destroy>", lambda event: self.dispose() if event.widget is page.parent else None, add="+")

    def dispose(self):
        self.closed = True
        self.job = None
        self.pending.clear()
        self.awaiting = None
        self.ticket += 1

    def generation(self):
        transport = self.page.panel.transport
        return (id(transport), getattr(transport, "connection_generation", 0))

    def _forget_airframe_params(self):
        """每次发 PARAM? 重读之前先扔掉旧的机体参数。

        新固件不再回显退役的键（旧力臂 airframe.*_thrust_lever_arm_m），PARAM? 只会覆盖、不会
        删除：板子经 ST-Link 重烧而链路没断（数传/蓝牙串口的连接代次不变）时，旧键会一直留在
        这里，开始时的参数快照就被当成旧力矩模型——候选增益按错的力臂换算、k 核对用错力臂、
        联合分析跟真正的旧轮次分到一组，还写进 conditions.json。开始、下发台架、临时应用、
        恢复这四处都先清再读。
        """
        for name in [name for name in self.params if name.startswith("airframe.")]:
            del self.params[name]

    def fail(self, text):
        if self.kind == "start" and (self.awaiting or self.pending):
            self.notice = text
        self.pending.clear()
        self.awaiting = None
        self.ticket += 1
        self.error = text
        self.page.status_var.set(text)
        self.page.refresh_banner()

    def data_fail(self, text):
        self.data_error = text
        self.fail(text)

    def mark_data_error(self, text):
        self.data_error = text
        self.error = text

    # ------------------------------------------------------------ 报告记账

    def note_sent(self, text):
        """页面每成功发出一条命令都来这里报到；事务自己的命令不记账。"""
        self.xy_note_sent(text)
        self.yaw_note_sent(text)
        if text == THR_POLL:
            self.thr_polls.append(time.monotonic() + STALE_REPORT_S)
        elif text != self.awaiting and produces_report(text):
            self.stale_reports.append(time.monotonic() + STALE_REPORT_S)

    def reports_in_flight(self):
        now = time.monotonic()
        self.stale_reports = [t for t in self.stale_reports if t > now]
        return len(self.stale_reports)

    def thr_polls_in_flight(self):
        now = time.monotonic()
        self.thr_polls = [t for t in self.thr_polls if t > now]
        return len(self.thr_polls)

    def thr_poll_allowed(self):
        return self.thr_poll_refused != self.generation()

    def _consume_stale(self):
        if self.reports_in_flight():
            self.stale_reports.pop(0)
            return True
        return False

    # ------------------------------------------------------------ 事务

    def submit(self, commands, kind):
        if self.awaiting or self.job:
            self.page.status_var.set("正在等待飞控回显或分析结束，请稍等")
            return
        self.pending = list(commands)
        self.kind = kind
        self.session = self.generation()
        self.error = ""
        if kind == "start":
            self.notice = ""
        else:
            self.run_note = ""
        self.advance()
        self.page.refresh_banner()

    def advance(self):
        if self.generation() != self.session:
            self.fail("连接已改变；本次事务取消，需重新开始")
            return
        if not self.pending:
            self.awaiting = None
            if self.kind == "start":
                self._send_start()
            elif self.kind == "rig":
                self.page.status_var.set("台架几何已回读确认（RAM，未保存 Flash）；未启动辨识")
            else:
                self.finish_ram()
            return
        command = self.pending.pop(0)
        if callable(command):
            try:
                command = command()
            except ValueError as error:
                self.fail(str(error))
                return
        self.awaiting = command
        if command == "PARAM?":
            if not self.page.send(command):
                self.fail(f"机体参数请求没发出去；后续动作已取消。{self.page.status_var.get()}"); return
            self.advance()  # following SYSID response is a command-order barrier
            return
        self.ticket += 1
        token = self.ticket
        self.page.status_var.set(f"正在发送，等待飞控回显（3秒超时）{self.run_note}")
        if not self.page.send(command):
            self.fail(f"命令未发出；后续动作已取消。{self.page.status_var.get()}")
            return
        self.page.parent.after(ECHO_TIMEOUT_MS, lambda: self.timeout(token))

    def _send_start(self):
        problem = None
        schema = self.page.schema
        servo = self.expected.get("mode") == SERVO_MODE_CODE
        if schema is None or schema.version not in RECORD_VERSIONS:
            problem = "需要支持 SYSID v2/v3 的固件及完整字段表：请更新固件后重试。"
        elif servo and (schema.version < 3 or "servo_tilt" not in schema.field_names()):
            problem = "舵机单独模式需要 SYSID 记录 v3 的固件（记录里要有 servo_tilt）：请更新固件后重试。"
        elif self.expected.get("mode") == ALT_MODE_CODE and not schema_has_height(schema):
            problem = ALT_UNSUPPORTED_SCHEMA
        elif self.expected.get("mode") == XY_MODE_CODE and not schema_has_xy(schema):
            problem = XY_UNSUPPORTED_SCHEMA
        elif self.expected.get("mode") == YAW_MODE_CODE and not schema_has_yaw(schema):
            problem = YAW_UNSUPPORTED_SCHEMA
        elif not servo and self.status.get("thrust") != "lut":
            problem = "飞控仍在使用旧推力曲线：请先在推力页选择当前查补表（LUT）后重试。"
        if problem:
            self.notice = problem
            self.fail(problem)
            return
        self.page.clear_samples()
        self.snapshot = dict(self.status)
        self.snapshot["parameter_echo"] = dict(self.params)
        self.snapshot["schema"] = asdict(self.page.schema)
        roll, pitch = self.pivot_to_cg
        self.snapshot.update(roll_pivot_to_cg_z_m=roll, pitch_pivot_to_cg_z_m=pitch,
                             pivot_to_fc_input_m=dict(zip(("roll", "pitch"), self.pivot_request)))
        self.alt_snapshot()
        self.xy_snapshot()
        self.yaw_snapshot()
        self.end = None
        self.run_id = None
        self.saved = None
        self.auto_fit = False
        self.page.on_run_starting()
        self.awaiting = "SYSID START"
        self.ticket += 1
        token = self.ticket
        self.page.status_var.set("配置已逐项核对，已发送开始命令，等待飞控确认")
        if not self.page.send("SYSID START"):
            self.fail("开始命令发送失败"); return
        self.page.parent.after(ECHO_TIMEOUT_MS, lambda: self.timeout(token))

    def timeout(self, token):
        if not self.closed and self.awaiting and token == self.ticket:
            if self.awaiting == "SYSID SCHEMA":
                self.fail("尚未开始辨识：读取飞控采集格式超时。请确认连接正常、固件支持 SYSID v2；"
                          "可在准备页读回飞控状态，核对后再点开始。（诊断：SYSID SCHEMA）")
                return
            if self.awaiting == "SYSID START":
                self.fail("飞控 3 秒内没有确认开始。如果电机已经转起来，点「停止」或用遥控器上锁。")
                return
            self.fail(f"等待回显超时：{self.awaiting}；不自动重发动作" + self.ram_partial_note())

    # ------------------------------------------------------------ 收行

    def on_line(self, line):
        if self.session and self.generation() != self.session:
            open_run = self.run_id is not None and self.end is None
            self.data_fail("连接代次改变，旧采集不得继续使用")
            if open_run:
                self.close_run_locally({"state": "aborted", "reason": "link_lost",
                                        "run": str(self.run_id), "dropped": "0"})
            self.page.schema = None
            self.session = None
            self.original_gains = None     # 存档路径留着：重连后「恢复原参数」从存档读
            self.original_torque = None
            self.params.clear()
            self.snapshot = None
            self.page._commands = None
            # 不 return：这一行可能正是新连接上的 SYSID end / 状态报告。
        values = parse_kv(line)
        if line == "OK sysid discarded stopped run":
            self.run_id = None
            self.end = None
            self.snapshot = None
            self.page.clear_samples()
            self.page.status_var.set("已丢弃待传数据；可重新配置开始")
            return
        if line.startswith("ERR"):
            if "THR?" in line and self.thr_polls_in_flight():
                self.thr_polls.pop(0)       # 旧固件不认轮询命令：不许它打断正在进行的事务
                self.thr_poll_refused = self.generation()
                return
            if self.awaiting:
                self.fail(explain_error(line) + self.ram_partial_note())
            elif line.startswith(("ERR sysid", "ERR usage SYSID", "ERR unknown sysid")):
                self._consume_stale()  # 单独下发的命令被拒：它不会再有状态报告
            return
        if line.startswith("OK sysid param "):
            self.on_ram_echo(values)
            return
        if self.alt_handle_line(line, values):
            return                        # SYSID ALT 回显/拒绝、SYSID ALTSTART 溯源（alt_config.py）
        if self.xy_handle_line(line, values):
            return                        # SYSID XY 回显/拒绝、SYSID XYSTART 溯源（xy_config.py）
        if self.yaw_handle_line(line, values):
            return                        # SYSID YAW 回显/拒绝、SYSID YAWSTART 溯源（yaw_config.py）
        if line.startswith("PARAM ") or line.startswith("OK param "):
            name, value = values.get("name"), values.get("value")
            if name and value:
                self.params[name] = value
                if name == "airframe.weight_n":
                    self.page.refresh_thrust_hint()
                elif name.startswith("airframe."):
                    self.page.refresh_geometry_hint()
            return
        if line.startswith("SYSID READY "):
            self.status = dict(values)
            self.ready = dict(values)
            self.page.refresh_geometry_hint()
        elif line.startswith(("SYSID RIG ", "SYSID EXC ", "SYSID THR ", "SYSID LIMITS ")):
            self.status.update(values)
        if line.startswith("SYSID state="):
            self.check_firmware_state(values)
        if line.startswith("SYSID THR ") and self.thr_polls_in_flight():
            self.thr_polls.pop(0)
        if line.startswith("SYSID LIMITS "):
            belongs_elsewhere = self._consume_stale()
            # SYSID ALT 的回显是它自己的一行（alt_config），整份报告的结尾不能冒充它。
            if (not belongs_elsewhere and self.awaiting and self.awaiting.startswith("SYSID")
                    and self.awaiting not in ("SYSID SCHEMA", "SYSID START")
                    and not self.awaiting.startswith(("SYSID ALT ", "SYSID XY ", "SYSID YAW "))):
                if not self.pending and not self._verify():
                    return
                self.advance()
        elif self.awaiting == "SYSID SCHEMA" and self.page.schema is not None:
            self.advance()
        if line.startswith("SYSID start "):
            if self.awaiting == "SYSID START":
                self.awaiting = None
                self.ticket += 1
            self.notice = ""
            self.run_id = int(values["run"])
            self.page.status_var.set(f"飞控已确认开始{self.run_note}")
            if self.snapshot is not None:
                self.snapshot["I_ugm2"] = values.get("I", "0")
                self.snapshot["start"] = dict(values)
            self.page.firmware_inertia_kg_m2 = float(values.get("I", "0"))*1e-6
            self.page.run_state_var.set("正在运行")
        # 开跑成功后固件紧跟一行 SYSID NOTCH：本轮控制用陀螺的陷波配置，随快照进 conditions.json。
        if (line.startswith("SYSID NOTCH ") and self.snapshot is not None
                and str(self.run_id) == values.get("run")):
            self.snapshot["rpm_notch"] = dict(values)
        # 紧跟着还有一行 SYSID BACKLASH：本轮舵机回差补偿的配置，同样进 conditions.json。
        if (line.startswith("SYSID BACKLASH ") and self.snapshot is not None
                and str(self.run_id) == values.get("run")):
            self.snapshot["backlash"] = dict(values)
        if (line.startswith("SYSID end ") and self.run_id == int(values.get("run", "-1"))
                and (self.end is None or self.end.get("synthetic"))):
            self.end = dict(values)       # 真的结束报告总是替换本地合成的那份
            self.page.run_state_var.set(f"{values.get('state')} · {values.get('reason')}")
            self._finish_run()

    # ------------------------------------------------------------ 结束报告丢了怎么办

    def close_run_locally(self, end):
        """结束报告收不到时，本地关掉这一轮（标记 synthetic；真报告来了会替换）。"""
        self._state_grace = None
        self.end = dict(end, synthetic="1")
        self.page.run_state_var.set(f"{end.get('state')} · {end.get('reason')}（本地判定）")
        self._finish_run()

    def check_firmware_state(self, values):
        """`SYSID state=` 与页面认为的不一致时，把页面从"一直在跑"里放出来。"""
        if self.run_id is None or self.end is not None:
            return
        state, run = values.get("state"), values.get("run")
        if state == "running" and run == str(self.run_id):
            return
        if run == str(self.run_id) and state in ("done", "aborted"):
            try:
                queued = int(values.get("queued", "0"))
            except ValueError:
                queued = 0
            if queued > 0:
                return                    # 数据还在上传，结束报告随后就到
            token = object()
            self._state_grace = token     # 给正常的 SYSID end 留 1.5 秒
            snapshot = dict(values)
            self.page.parent.after(1500, lambda: self.close_if_still_open(token, snapshot))
            return
        self.close_run_locally({"state": "aborted", "reason": "state_mismatch",
                                "run": str(self.run_id), "dropped": "0",
                                "firmware_state": f"{state} run={run}"})

    def close_if_still_open(self, token, values):
        if self.closed or token is not self._state_grace or self.end is not None:
            return
        self.close_run_locally({"state": values.get("state", "aborted"),
                                "reason": values.get("reason", "state_mismatch"),
                                "run": str(self.run_id), "dropped": values.get("dropped", "0")})

    def _verify(self):
        if self.kind == "start" and not all(k in self.status for k in ("auto", "target_cn", "max_pct_x10")):
            self.fail("飞控回报里没有油门状态：固件太旧，不支持程序控油门。请先更新固件（需要 SYSID ver=3）。")
            return False
        for key, expected in self.expected.items():
            # 物理量允许 1 个量化单位的误差；枚举/开关量必须完全相等（差 1 就是另一种模式）。
            tolerance = 0 if key in EXACT_ECHO_KEYS else 1
            try:
                ok = key in self.status and abs(float(self.status[key])-expected) <= tolerance
            except (TypeError, ValueError):
                ok = False
            if not ok:
                self.fail(f"配置回读不符：{key}，取消开始")
                return False
        return True

    # ------------------------------------------------------------ 结束 → 存档 → 分析

    def _finish_run(self):
        done = self.end.get("state") == "done"
        mode = str((self.snapshot or {}).get("mode", "0"))
        self.auto_fit = done and mode in ("0", str(SERVO_MODE_CODE))
        self.page.on_run_finished(self.end, mode)
        started_saving = self.archive()
        if self.auto_fit and not started_saving:
            self.auto_fit = False
            self.fit()

    def archive_directory(self):
        # 高度轮目录用 alt_ 开头：联合分析/舵机对照只扫 rod_*，也方便离线脚本挑出来。
        # 水平槽轮单独放 data/identification/xy/<日期>/xy_*，吊绳偏航轮放 data/identification/yaw/<日期>/yaw_*。
        mode = str((self.snapshot or {}).get("mode"))
        prefix = ("alt" if mode == str(ALT_MODE_CODE) else "xy" if mode == str(XY_MODE_CODE)
                  else "yaw" if mode == str(YAW_MODE_CODE) else "rod")
        root = (_xy_directory() if prefix == "xy" else _yaw_directory() if prefix == "yaw"
                else _attitude_directory())
        return root / (datetime.now().strftime(f"{prefix}_%H%M%S_") + uuid4().hex[:8])

    def archive(self):
        if self.saved or not self.page.batches or self.snapshot is None:
            return False
        from sysid.report import write_samples_csv
        directory = self.archive_directory()
        batches, metadata = list(self.page.batches), dict(self.snapshot)
        metadata.update(end=self.end, data_error=self.data_error, last_message=self.error,
                        run_id=self.run_id)
        self.saved = directory
        self.saving = True
        def save():
            try:
                directory.mkdir(parents=True, exist_ok=False)
                write_samples_csv(batches, directory / "samples.csv")
                (directory / "conditions.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
                self.results.put(("saved", str(directory)))
            except Exception as error:
                self.results.put(("save_error", f"保存失败：{error}"))
        threading.Thread(target=save, daemon=True).start()
        self._schedule_pump()
        return True

    def _schedule_pump(self):
        if self._pump_pending or self.closed:
            return
        self._pump_pending = True
        self.page.parent.after(100, self._pump_tick)

    def _pump_tick(self):
        self._pump_pending = False
        self.pump()

    def pump(self):
        if self.closed:
            return
        try:
            while True:
                kind, value = self.results.get_nowait()
                if kind == "fit":
                    token, result, note, context = value
                    if token is self.job:
                        self.job = None
                        self.page.on_fit(result, note, context)
                elif kind == "fit_error":
                    token, text = value
                    if token is self.job:
                        self.job = None
                        self.page.on_fit_error(text)
                elif kind == "saved":
                    self.saving = False
                    self.page.status_var.set(f"原始记录与本轮物理条件已保存：{value}")
                    self._after_save()
                else:
                    self.saving = False
                    self.saved = None  # 目录没建成，拟合结果就不往里写
                    self.page.status_var.set(value)
                    self._after_save()
        except queue.Empty:
            pass
        if self.job or self.saving:
            self._schedule_pump()

    def _after_save(self):
        if self.auto_fit:
            self.auto_fit = False
            self.fit()

    def discard(self):
        self.fail("正在请求丢弃已停止轮次的待传数据")
        self.page.send("SYSID DISCARD")
