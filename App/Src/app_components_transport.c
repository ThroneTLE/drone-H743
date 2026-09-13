#include "app_components.h"
#include "app_diag_binary.h"
#include "app_proto.h"
#include "app_control_internal.h"
#include "app_flight_log.h"
#include "app_imu_capture.h"
#include "app_maint_uart.h"
#include "app_messages.h"
#include "app_tasks.h"
#include "app_uart.h"
#include "app_usb_cdc.h"
#include "svc_timestamp.h"
#include <string.h>

uint32_t APP_Components_NowMs(void) { return SVC_Timestamp_Ms(); }
uint8_t APP_Components_ExportBusy(void)
{
    return APP_FlightLog_IsExportActive() || APP_IMU_Capture_IsExportActive();
}
uint8_t APP_Components_Send(const uint8_t *payload,uint16_t length)
{
    return APP_Diag_SendBinary(APP_PROTO_MSG_COMPONENTS,payload,length);
}
