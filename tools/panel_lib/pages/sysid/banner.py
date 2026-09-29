"""页面顶部那条大字状态：现在处在哪一步、下一步做什么。

纯函数，不碰 Tk：输入是页面此刻知道的事实，输出是（标题, 说明, 色调）。
这样每一种状态的文案都能在没有窗口的情况下直接测。
"""

from __future__ import annotations

from dataclasses import dataclass

from .reasons import explain_end

#: 色调 -> ttk 样式。标题统一用大字，颜色只落在说明行上。
TONE_STYLES = {
    "idle": "Muted.TLabel",
    "active": "Active.TLabel",
    "pass": "Pass.TLabel",
    "warn": "Warn.TLabel",
    "fail": "Fail.TLabel",
}

_PHASES = {
    "ramp_up": ("升油门…",
                "程序正在慢慢把油门推到目标推力（约 1.5 秒）。想中止：点「停止」、推油门杆或上锁都可以。"),
    "settle": ("稳定中…", "油门已到目标，等机体稳定约 1 秒后开始激励。"),
    "ramp_down": ("降油门…", "激励已结束，程序正在把油门降回去（约 1 秒）。"),
    # 舵机单独（电机不转、全程上锁）的阶段。
    "preroll": ("预滚中…", "舵机回中静止约 0.5 秒，记录零激励基线。保持上锁；解锁会立即停止。"),
    "done": ("收尾中…", "激励已结束，舵机回中，数据正在上传。"),
    # 高度辨识（ALT）新增的两个阶段；模式未知时也能认出来。
    "climb": ("升高中…", "高度环正在把机体沿槽抬高到设定高度。"),
    "descend": ("下降中…", "激励已结束，高度环正在把机体降回去。"),
}
#: 高度辨识（ALT，槽式台架）同名阶段的说法：电机在转、机体沿槽上下动，不是舵机回中。
_ALT_PHASES = {
    "ramp_up": ("升推力…", "程序正在开环把推力升到接近悬停（约 1.5 秒，机体还压在槽底）。"
                "想中止：点「停止」、推油门杆或上锁都可以。"),
    "climb": _PHASES["climb"],
    "settle": ("稳定中…", "机体已到设定高度，保持约 2 秒后开始激励。"),
    "preroll": ("前导中…", "在设定高度记录约 0.5 秒零激励基线，随后开始激励。"),
    "descend": _PHASES["descend"],
    "ramp_down": ("回落中…", "程序正在降推力，机体回落到下限位。"),
    "done": ("收尾中…", "激励已结束，数据正在上传。"),
}
_EXCITE_DETAIL = "舵机正在按激励摆动：别碰机体，也别碰油门杆。"
_SERVO_EXCITE_DETAIL = "舵机正在按激励摆动（电机不转）：别碰机体；保持上锁，解锁会立即停止。"
_ALT_EXCITE_DETAIL = "机体正在按激励沿槽上下移动：别碰机体，也别碰油门杆。"
SERVO_KEEP_DISARMED = "保持上锁；解锁会立即停止"


@dataclass
class BannerInput:
    connected: bool = False
    manual: bool = False
    config_busy: bool = False
    start_pending: bool = False
    stopping: bool = False
    run_active: bool = False
    phase: str | None = None
    samples: int = 0
    end: dict | None = None
    analysis: str = ""
    notice: str = ""
    thr: dict | None = None
    #: 本连接上确认固件太旧（READY ver<3，或不认 `SYSID THR?`）。
    fw_outdated: bool = False
    #: 本轮模式（FF/RATE/ANGLE 或固件的 0/1/2），只影响中止原因的建议。
    mode: str | None = None
    #: 舵机单独模式（选中或正在跑）：电机不转，要求**保持上锁**。
    servo: bool = False
    #: 预计舵机摆幅太小的提醒（`amplitude_hint.amp_swing_warning`）；开始前追加在说明行，不挡开始。
    amp_warning: str = ""
    #: 高度辨识（ALT，选中或正在跑）：电机会转、机体沿槽上下移动，要程序油门。
    alt: bool = False


def _flag(thr: dict | None, key: str) -> bool | None:
    if not thr or key not in thr:
        return None
    return str(thr[key]).strip() == "1"


def describe(s: BannerInput) -> tuple[str, str, str]:
    title, detail, tone = _describe(s)
    if s.amp_warning and _before_start(s):
        detail = f"{detail}\n{s.amp_warning}" if detail else s.amp_warning
        tone = "warn" if tone == "pass" else tone
    return title, detail, tone


def _before_start(s: BannerInput) -> bool:
    """还没开跑、下一步就是（或正在）开始：摆幅提醒只在这时有用。"""
    if not s.connected or s.run_active or s.servo:
        return False
    if s.config_busy or s.start_pending:
        return True
    return not s.notice and s.end is None and not s.fw_outdated


def _describe(s: BannerInput) -> tuple[str, str, str]:
    if not s.connected:
        return ("未连接", "先在窗口上方连接飞控（USB 线最稳），连上后这里会显示遥控器状态。", "idle")
    if s.run_active:
        if s.stopping:
            return ("正在停止…", "电机正在回到遥控器油门。" if not s.manual else
                    "停止辨识不会停电机！请用遥控器收油门并上锁。", "warn")
        phase = s.phase or ("preroll" if s.servo else "excite" if s.manual else "ramp_up")
        phases = _ALT_PHASES if s.alt else _PHASES
        if phase in phases:
            title, detail = phases[phase]
            return (title, detail, "active")
        if phase not in ("excite", "idle"):
            # 页面还没有说明的新阶段名：照原样显示，别冒充激励。
            return (f"阶段 {phase}（样本 {s.samples}）",
                    "飞控报告了一个页面还不认识的阶段；想中止：点「停止」、推油门杆或上锁。", "active")
        return (f"激励中（样本 {s.samples}）",
                _SERVO_EXCITE_DETAIL if s.servo else _ALT_EXCITE_DETAIL if s.alt else _EXCITE_DETAIL,
                "active")
    if s.start_pending:
        return ("正在开始…", "已发送开始命令，等待飞控确认。", "active")
    if s.config_busy:
        return ("正在下发配置并核对…", "只改飞控内存（RAM），一两秒就好。", "active")
    if s.notice:
        return ("没能开始", s.notice, "fail")
    manual_tail = "（手动油门：记得用遥控器收油门并上锁。）" if s.manual else ""
    if s.end is not None:
        state = s.end.get("state")
        if state == "done":
            return ("完成", (s.analysis or "数据已收齐。") + manual_tail, "pass")
        what, step = explain_end(s.end.get("reason"), s.mode)
        return (f"中止：{what}", (f"下一步：{step}" if step else "") + manual_tail, "fail")
    if s.fw_outdated:
        return ("飞控固件需要更新",
                "这版固件还不支持程序控油门（需要 SYSID ver=3），请先更新固件。", "warn")
    if s.alt and s.manual:
        return ("高度辨识要程序油门",
                "取消勾选「准备」页的「遥控器手动给油门」：ALT 由程序升推力、高度环抬升机体。", "warn")
    armed, low = _flag(s.thr, "armed"), _flag(s.thr, "thr_low")
    if armed is None:
        return ("正在读取飞控状态…", "每秒自动刷新一次；一直停在这里说明飞控没回话。", "idle")
    if s.servo:
        if armed:
            return ("请先上锁", "舵机单独模式电机不转，全程要" + SERVO_KEEP_DISARMED + "。", "warn")
        return ("就绪（舵机单独），可以开始",
                "点「开始辨识」：舵机按激励摆动，电机不转。" + SERVO_KEEP_DISARMED + "。", "pass")
    if not armed:
        detail = ("请先用遥控器解锁（油门杆拉到最低）。解锁后到「高级设置」点「进入台架待机」，"
                  "把油门推稳，再点「开始辨识」。" if s.manual else
                  "请先用遥控器解锁（油门杆拉到最低）。解锁后直接点「开始辨识」，油门由程序来推。")
        return ("请先解锁（油门杆最低）", detail, "warn")
    if s.manual:
        return ("已解锁：手动油门方式",
                "到「高级设置」点「进入台架待机」，把油门推稳，再点「开始辨识」。", "idle")
    if low is False:
        return ("请把油门杆拉到最低", "程序只在油门杆在最低时接管油门。", "warn")
    if s.alt:
        return ("就绪（高度辨识），可以开始",
                "先确认槽的上下限位与测距下方地面。点「开始辨识」：程序升推力、高度环把机体抬高并稳定、"
                "激励、再下降回落，全程自动。推油门杆或上锁会立即把油门交还遥控器。", "pass")
    return ("就绪，可以开始",
            "点「开始辨识」：程序会升油门、稳定、激励、再降油门，全程自动。"
            "推油门杆或上锁会立即把油门交还遥控器。", "pass")


def rc_summary(thr: dict | None) -> str:
    armed, low = _flag(thr, "armed"), _flag(thr, "thr_low")
    if armed is None:
        return "遥控器：状态未知"
    parts = ["已解锁" if armed else "未解锁"]
    if low is not None:
        parts.append("油门杆在最低" if low else "油门杆不在最低")
    if thr and str(thr.get("auto", "0")) == "1":
        try:
            target = int(thr.get("target_cn", 0)) / 100.0
            parts.append(f"程序油门目标 {target:.1f} N")
        except (TypeError, ValueError):
            pass
    if thr and str(thr.get("capped", "0")) == "1" and thr.get("phase", "idle") != "idle":
        parts.append("已按最高油门封顶，实际推力低于目标")
    return "遥控器：" + " · ".join(parts)


__all__ = ["BannerInput", "SERVO_KEEP_DISARMED", "TONE_STYLES", "describe", "rc_summary"]
