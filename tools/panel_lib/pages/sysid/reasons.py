"""把飞控的原始口令翻成作者看得懂的中文：发生了什么 + 下一步做什么。

中止原因（`SYSID end ... reason=<token>`）和拒绝回复（`ERR sysid ...`）都在这里查表。
界面上永远先说中文，原话放在括号里——原话是给开发排查用的，不是给操作的人读的。
"""

from __future__ import annotations

#: token -> (发生了什么, 下一步)
END_REASONS: dict[str, tuple[str, str]] = {
    "complete": ("完成", ""),
    "command": ("你点了停止",
                "想再跑一轮，直接点「开始辨识」。"),
    "rc_disarm": ("飞控上锁了（或一直没解锁）",
                  "用遥控器解锁（油门杆拉到最低），再点「开始辨识」。"),
    "rc_arm": ("舵机单独模式下飞控被解锁了，已立即停止",
               "用遥控器上锁后再点「开始辨识」：舵机单独模式电机不转，全程保持上锁。"),
    "rc_lost": ("遥控信号丢失",
                "检查遥控器电量和接收机，信号恢复后重新解锁再开始。"),
    "rc_throttle_override": ("你推了油门杆，程序已把油门交还遥控器",
                             "把油门杆拉回最低，再点「开始辨识」；辨识过程中不用碰油门杆。"),
    "imu_stale": ("陀螺仪数据中断",
                  "IMU 数据没按时到。给飞控重新上电后再试；反复出现请把日志发给开发。"),
    "actuator_busy": ("其他台架/标定功能正在占用电机或舵机",
                      "先结束其他页面里正在运行的台架测试或标定，再点「开始辨识」。"),
    "thrust_stale": ("推力查表缺电压或电调转速回传",
                     "检查双向 DShot 是否开启、电调转速回传是否正常、电池电压是否在读。"),
    "thrust_low": ("推力太低",
                   "在「准备」页提高「目标合推力」；手动油门时把油门推高一些再开始。"),
    "nonfinite": ("计算中出现无效数值",
                  "多半是某路传感器读数异常。给飞控重新上电后再试；反复出现请把日志发给开发。"),
    "axis_residual": ("机体没有只绕杆转",
                      "检查「杆轴方向」选得对不对、杆有没有夹紧、机体有没有在杆上晃动或滑动。"),
    "angle_limit": ("摆角超过了保护角",
                    "在「高级设置」减小激励幅值，再重新开始。"),
    "excitation": ("激励设置有问题",
                   "到「高级设置」检查激励设置，或把数值改回默认后重试。"),
    "record_range": ("记录的数值超出量程",
                     "激励太猛导致数据溢出：在「高级设置」减小激励幅值。"),
    "controller": ("控制器计算失败",
                   "检查机体参数和控制参数；刚临时改过参数的话，先点「恢复原参数」再试。"),
    "rig": ("台架几何设置无效",
            "到「准备」页检查杆轴方向和两个距离，点「下发台架几何」确认后再开始。"),
    "solve": ("舵机角度算不出来",
              "要求的力矩超出机构能做到的范围：减小激励幅值，或提高目标合推力。"),
    "actuator_saturated": ("舵机行程或推力不够产生所需力矩",
                           "在「高级设置」减小激励幅值，或在「准备」页提高目标合推力。"),
    "control_dt": ("控制周期异常（固件问题）",
                   "请把这次的日志和数据文件夹发给开发。"),
    # 高度辨识（ALT，槽式台架）的安全中止。
    "height_invalid": ("测距（TOF）高度失效",
                       "检查测距模块有没有被遮挡、接线是否松动、下方地面是否平整且在量程内；"
                       "恢复后再点「开始辨识」。"),
    "height_window": ("高度超出允许窗口",
                      "机体离开了允许的高度范围：检查槽里有没有卡滞，在「准备」页「高度」分区核对"
                      "抬升高度与出窗余量，或在「高级设置」减小激励幅值后重试。"),
    # 下面两个是上位机自己合成的：结束报告没收到，本地把这一轮关掉。
    "link_lost": ("连接中断或飞控复位，本轮作废",
                  "检查连接线/数传，重新连上后再点「开始辨识」。"),
    "state_mismatch": ("飞控报告这一轮已经不在运行（可能复位过），本轮作废",
                       "点「读回飞控状态」确认飞控正常后，重新点「开始辨识」。"),
}

#: 摆角超限的下一步取决于模式：ANGLE 轮是角度幅值太大，FF/RATE 轮是激励幅值太大。
_ANGLE_LIMIT_STEP = {
    "ANGLE": "在「高级设置」减小「ANGLE 角度幅值」后重试；机体在杆上自然下垂的角度"
             "也计入绝对角度限位，下垂明显时幅值要留得更小。",
    "OTHER": "在「高级设置」减小激励幅值（rad/s）后重试。",
}


def explain_end(reason: str | None, mode: str | None = None) -> tuple[str, str]:
    """`(发生了什么, 下一步)`。未知口令给通用说法，但保留原话。

    `mode` 是本轮的 FF/RATE/ANGLE（也接受固件的 0/1/2），只影响摆角超限的建议。
    """
    token = (reason or "").strip()
    if token == "angle_limit":
        is_angle = str(mode) in ("ANGLE", "2")
        return END_REASONS[token][0], _ANGLE_LIMIT_STEP["ANGLE" if is_angle else "OTHER"]
    if token in END_REASONS:
        return END_REASONS[token]
    return (f"飞控中止了这一轮（原因代码 {token or '未知'}）",
            "检查机体、遥控器和连接后重新开始；反复出现请把日志发给开发。")


def cannot_analyse(reason: str | None, mode: str | None = None) -> str:
    what, step = explain_end(reason, mode)
    return (f"这一轮中止了（{what}），数据不完整，不能分析。"
            f"{step or ''}处理好后重新点「开始辨识」跑一轮。")


#: (原话前缀, 中文说明)。按顺序匹配，长前缀放前面。
_ERRORS: tuple[tuple[str, str], ...] = (
    ("ERR sysid servo mode needs disarmed",
     "舵机单独模式要保持上锁：先用遥控器上锁，再点「开始辨识」（电机不转；解锁会立即停止）。"),
    ("ERR sysid not armed",
     "飞控还没解锁：请先用遥控器解锁（油门杆拉到最低），再点「开始辨识」。"),
    ("ERR sysid throttle stick not low",
     "油门杆没在最低：把遥控器油门杆拉到最低再点开始——油门由程序来推。"),
    ("ERR sysid throttle",
     "飞控不接受这组油门设置：目标合推力要在 2 N 到机体最大总推力之间，"
     "最高油门要在 10%～95%。请在「准备」页修改。"),
    ("ERR unknown sysid subcmd THROTTLE",
     "飞控固件太旧，不认识程序控油门命令：请先更新固件（需要 SYSID ver=3）。"),
    ("ERR sysid precheck",
     "开始前自检没过：需要新鲜的电池电压和电调转速回传（检查电池和双向 DShot）；"
     "不是 USB 连接时采样率不能超过 100 Hz（到「高级设置 → 线上采样率」改成 100）；"
     "正在导出日志时不能开始。"),
    ("ERR unknown sysid subcmd PARAM",
     "飞控固件不支持只写 RAM 的参数试用，未应用，请更新固件。"),
    ("ERR sysid param only",
     "飞控只接受控制增益（coax.rate_* / coax.att_*）的试用，未应用。"),
    ("ERR usage SYSID PARAM",
     "参数试用命令格式不对，未应用。"),
    ("ERR sysid param",
     "飞控固件不支持只写 RAM 的参数试用，未应用，请更新固件。"),
    ("ERR sysid inertia unknown",
     "飞控不知道机体惯量：在「高级设置」填假定惯量，或先在机体参数里填好 Ixx。"),
    ("ERR sysid airframe invalid", "机体参数无效：先到机体参数页检查。"),
    ("ERR sysid already running", "上一轮还没结束：等它结束或点「停止」后再开始。"),
    ("ERR sysid ident running", "旧的单轴辨识还在运行：先停掉它再开始。"),
    ("ERR sysid previous run draining", "上一轮的数据还在上传：等几秒再点开始。"),
    ("ERR sysid still running", "上一轮还没结束：等它结束或点「停止」后再操作。"),
    ("ERR sysid excitation", "激励设置被飞控拒绝：到「高级设置」检查激励设置。"),
    ("ERR sysid profile", "激励剖面名称无效：到「高级设置」重新选择剖面。"),
    ("ERR sysid rig", "台架几何被飞控拒绝：检查「准备」页的角度和距离是否在合理范围。"),
    ("ERR sysid angle_deg", "保护角超出范围：到「高级设置」检查角度上限。"),
    ("ERR sysid resid_dps", "轴向残差上限超出范围：到「高级设置」检查。"),
    ("ERR sysid mode", "模式设置被拒绝：角度幅值须在 0～15°，且只能在空闲时设置；"
     "固件太旧时不认舵机单独模式（SERVO）或高度辨识模式（ALT），请更新固件。"),
    ("ERR unknown sysid subcmd ALT",
     "飞控固件太旧，不认高度辨识设置（SYSID ALT）：请先更新固件。"),
    ("ERR usage SYSID ALT",
     "高度辨识设置命令格式不对（面板和固件版本可能不一致），请更新固件。"),
    ("ERR sysid alt",
     "高度辨识开始前自检没过（按括号里的原话处理）：测距（TOF）高度无效时检查测距模块与下方地面；"
     "幅值超限时到「高级设置 → 激励编排」改小；质量或设置无效时核对机体参数和「准备」页「高度」分区。"),
    ("ERR usage SYSID RATE", "采样率超出范围：到「高级设置」检查线上采样率。"),
    ("ERR usage SYSID INERTIA", "假定惯量无效：到「高级设置」检查，或留空用机体参数。"),
)


def explain_error(line: str) -> str:
    """一条 `ERR ...` 回复的中文说明，原话附在括号里。"""
    text = line.strip()
    for prefix, meaning in _ERRORS:
        if text.startswith(prefix):
            return f"{meaning}（飞控原话：{text}）"
    return f"飞控拒绝了这条命令（飞控原话：{text}）"


__all__ = ["END_REASONS", "cannot_analyse", "explain_end", "explain_error"]
