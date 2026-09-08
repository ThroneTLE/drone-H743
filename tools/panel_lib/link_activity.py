"""Busy reasons shared with the existing keepalive policy."""


def keepalive_suppressed_reason(panel):
    if (getattr(panel, "firmware_update_pending", False)
            or getattr(panel, "firmware_update_running", False)
            or getattr(panel, "firmware_programming", False)):
        return "固件升级进行中"
    if panel.v1_worker is not None and panel.v1_worker.is_alive():
        return "IMU 校准正在独占 USB CDC"
    if getattr(getattr(panel, "transport", None), "transfer_active", False):
        return "飞行日志导出正在使用当前串口"
    return None
