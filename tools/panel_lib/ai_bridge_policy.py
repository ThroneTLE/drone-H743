"""AI 接口的命令安全策略：纯函数，不碰 Tk、不碰网络，方便单测逐条核对。

判定顺序固定为「永远拒绝 -> DFU 特例 -> 白名单 -> 默认拒绝」。
永远拒绝表先于白名单，所以就算以后有人往白名单里多加一条，
也不可能把让电机转起来的命令放出去。

命令语法以固件为准（核对处见各条注释）：
  App/Src/app_control.c      MOTOR / IDENT START / PWMn / ACCEPT V2 / BOOT DFU CONFIRM / PARAM
  App/Src/app_cmd_sysid.c    SYSID 命令族、IDENT ARM|STEP|DOUBLET|PRBS|...
  App/Src/app_cmd_thrust_bench.c  TBENCH ARM|SET|STOP
  App/Src/app_cmd_propcal.c  PROPCAL SPIN ARM|STOP
  App/Src/app_cmd_esc_kv.c / app_cmd_esc_edt.c  ESC KV / ESC EDT（电调编程）
"""

from __future__ import annotations

from dataclasses import dataclass

ALLOW = "allow"
DENY = "deny"
DFU = "dfu"

MAX_LINE_CHARS = 200

DFU_LINE = ("BOOT", "DFU", "CONFIRM")

# 白名单：按 token 前缀匹配（已转大写）。
WHITELIST_PREFIXES: tuple[tuple[str, ...], ...] = (
    ("PARAM?",),
    ("PARAM", "GET"),
    ("PARAM", "SET"),
    ("SYSID", "PARAM"),
    ("SYSID", "THR?"),
    ("SYSID", "ALT"),
    ("SYSID", "MODE"),
    ("SYSID", "EXC"),
    ("SYSID", "LIMIT"),
    ("SYSID", "THROTTLE"),
    ("SYSID", "XY"),
    ("SYSID", "YAW"),
    # 磁航向配置（不转电机；VERIFY CONFIRM 只在实物转向核对通过后由 AI 发，2026-09-30 作者要看 XY 磁航向效果）
    ("MAGXY", "SET"),
    ("MAGXY", "VERIFY", "CONFIRM"),
    ("MAGXY", "ENABLE"),
    ("MAGXY", "CLEAR"),
    ("MAGXY", "COMMIT"),
    # 软件重新标定陀螺零偏与姿态零点（app_cmd_imuzero.c，上锁才接受；作者 2026-10-01 要求）
    ("IMUZERO",),
    # 飞行日志记录频率子分频 1..5（app_cmd_flograte.c，只改 RAM，记录中拒绝）
    ("FLOGRATE",),
    # 光流逐帧抓取诊断（app_cmd_flowcap.c，只记录不改任何量；2026-10-01 查中值滤波丢数）
    ("FLOWCAP", "START"),
    ("FLOWCAP", "DUMP"),
    # 让东西停下来的命令
    ("SYSID", "STOP"),
    ("STOP",),
    ("MOTOR", "STOP"),
    ("IDENT", "STOP"),
    ("TBENCH", "STOP"),
    ("PROPCAL", "SPIN", "STOP"),
    ("ACCEPT", "V2", "STOP"),
)

# 永远拒绝：(说明, 判定函数)。判定函数收到大写 token 列表。
def _is_arm(t):
    # 解锁：`ARM` 作首词或出现在子命令位置；REQ 的 op=ARM 写法也算。
    return (t[0] == "ARM" or "ARM" in t[1:3] or "OP=ARM" in t
            or (t[0] == "IDENT" and len(t) > 1 and t[1] == "ARM"))


def _is_motor_run(t):
    # app_control.c app_control_handle_motor：MOTOR SET <0|1|2> <pct>，pct 非 0 转电机。
    if t[0] != "MOTOR" or len(t) < 2 or t[1] in ("STOP", "?"):
        return False
    return not (t[1] == "SET" and len(t) == 4 and t[3] == "0")


def _is_raw_pwm(t):
    # app_control.c：`PWM1..2:0..100` 直接给电调占空比；只有查询 `PWM?` 放行。
    return t[0].startswith("PWM") and t[0] != "PWM?"


def _is_start(t):
    # SYSID START（app_cmd_sysid.c）、IDENT START（app_control.c）、
    # ACCEPT V2 START/STAGE/KEEPALIVE（app_control.c），都会让电机转。
    if t[0] in ("SYSID", "IDENT") and len(t) > 1 and t[1] == "START":
        return True
    if t[0] == "ACCEPT" and len(t) > 2 and t[2] in ("START", "STAGE", "KEEPALIVE"):
        return True
    return "OP=START" in t


def _is_ident_motion(t):
    # app_cmd_sysid.c app_cmd_sysid_handle_ident：姿态/舵机激励类子命令。
    return (t[0] == "IDENT" and len(t) > 1
            and t[1] in ("ARM", "START", "ATT", "STEP", "DOUBLET", "PRBS", "CENTER", "APPLY"))


def _is_esc_program(t):
    # 电调编程：ESC KV / ESC EDT。
    return t[0] == "ESC" and len(t) > 1 and t[1] in ("KV", "EDT")


def _is_bench_start(t):
    # 推力台：TBENCH ARM / TBENCH SET（app_cmd_thrust_bench.c）
    # 转桨：PROPCAL SPIN ARM 等（app_cmd_propcal.c），只有 PROPCAL SPIN STOP 放行。
    if t[0] == "TBENCH" and len(t) > 1 and t[1] in ("ARM", "SET"):
        return True
    if t[0] == "PROPCAL" and len(t) > 1 and t[1] == "SPIN":
        return not (len(t) > 2 and t[2] == "STOP")
    return False


NEVER_RULES = (
    ("解锁类命令（ARM）永远不允许 AI 发送", _is_arm),
    ("直接转电机命令（MOTOR SET 非 0）永远不允许 AI 发送", _is_motor_run),
    ("原始 PWM 命令永远不允许 AI 发送", _is_raw_pwm),
    ("开始辨识/开始测试（START 类）会让电机转，永远不允许 AI 发送", _is_start),
    ("IDENT 激励/舵机动作类命令永远不允许 AI 发送", _is_ident_motion),
    ("电调编程（ESC KV/EDT）永远不允许 AI 发送", _is_esc_program),
    ("推力台/转桨启动类命令永远不允许 AI 发送", _is_bench_start),
)


@dataclass(frozen=True)
class Verdict:
    kind: str       # ALLOW / DENY / DFU
    reason: str = ""


def normalize(line: str) -> str:
    return " ".join(line.strip().upper().split())


def classify(line: str) -> Verdict:
    if not isinstance(line, str) or not line.strip():
        return Verdict(DENY, "命令为空")
    if any(ord(c) < 32 or ord(c) == 127 for c in line):
        return Verdict(DENY, "命令含换行或控制字符，AI 接口一次只发一行")
    if len(line) > MAX_LINE_CHARS:
        return Verdict(DENY, f"命令超过 {MAX_LINE_CHARS} 字符")
    tokens = normalize(line).split()

    for reason, rule in NEVER_RULES:
        if rule(tokens):
            return Verdict(DENY, reason)

    if tokens[0] == "BOOT":
        if tuple(tokens) == DFU_LINE:
            return Verdict(DFU)
        return Verdict(DENY, "BOOT 类命令只放行 BOOT DFU CONFIRM（且要满足双条件）")

    # 查询：`XXX?` 或 `XXX ?`（最多两个词），不带任何写入参数。
    if len(tokens) == 1 and tokens[0].endswith("?"):
        return Verdict(ALLOW)
    if len(tokens) == 2 and tokens[1] == "?":
        return Verdict(ALLOW)

    for prefix in WHITELIST_PREFIXES:
        if tuple(tokens[:len(prefix)]) == prefix:
            return Verdict(ALLOW)
    return Verdict(DENY, "不在 AI 接口白名单内（只放行查询、PARAM/SYSID 参数类和停止类命令）")


__all__ = ["ALLOW", "DENY", "DFU", "DFU_LINE", "NEVER_RULES", "WHITELIST_PREFIXES",
           "Verdict", "classify", "normalize"]
