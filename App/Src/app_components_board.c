/* Firmware-owned registrations. The host knows neither this list nor chip names. */
#include "app_components.h"
#include "app_baro.h"
#include "app_current.h"
#include "app_flash_service.h"
#include "app_gps.h"
#include "app_led.h"
#include "app_mag.h"
#include "app_optical_flow.h"
#include "app_sensor.h"
#include "app_stabilizer.h"
#include "app_servo_bus_guard.h"
#include "app_uart.h"
#include "app_control.h"
#include "app_control_internal.h"
#include "bsp_component_catalog.h"
#include "bsp_dshot.h"
#include "bsp_esc_protocol.h"
#include "bsp_imu.h"
#include "bsp_pwm.h"
#include "bsp_uart.h"
#include "bsp_uart_link.h"
#include "drv_elrs.h"
#include "svc_timestamp.h"

static void identity(DRV_ComponentRecord *r,uint16_t id,const char *name,const char *model,uint8_t variant)
{
    r->name=name;r->model=model;r->bus=BSP_Component_Interface(id,variant);
    r->state=DRV_COMPONENT_UNKNOWN;r->stage="not initialized";r->note="";
}
static void integer(DRV_ComponentRecord *r,const char *label,const char *unit,uint32_t value,uint8_t hex,uint8_t valid)
{
    DRV_ComponentField *f=&r->fields[r->field_count++];
    f->label=label;f->unit=unit;f->type=hex?DRV_COMPONENT_HEX:DRV_COMPONENT_U32;f->valid=valid;f->value.u=value;
}
static void number(DRV_ComponentRecord *r,const char *label,const char *unit,float value,uint8_t valid)
{
    DRV_ComponentField *f=&r->fields[r->field_count++];
    f->label=label;f->unit=unit;f->type=DRV_COMPONENT_F32;f->valid=valid;f->value.f=value;
}
static void imu(DRV_ComponentRecord *r)
{
    APP_IMU_Status s;SVC_IMU_Selection selection;APP_IMU_GetStatus(&s);BSP_IMU_GetSelection(&selection);
    identity(r,DRV_COMPONENT_IMU,"IMU",SVC_IMU_ChipName(selection.selected),(uint8_t)selection.selected);
    r->code=s.last_status;r->samples=s.sample_count;
    r->state=s.last_status?DRV_COMPONENT_FAULT:(s.initialized?DRV_COMPONENT_READY:DRV_COMPONENT_WAITING);
    r->stage=s.initialized?"initialized":"initialization";
    integer(r,"chip_id","",selection.selected_chip_id,1,selection.selected!=DRV_IMU_CHIP_NONE);
    integer(r,"init_stage","",s.init_stage,0,1);
    integer(r,"error","",(uint32_t)s.last_error,1,1);
    number(r,"temperature","C",s.temperature_cdeg*0.01f,s.sample_count!=0 && s.last_status==0);
}
static void baro(DRV_ComponentRecord *r)
{
    APP_Baro_Status s;APP_Baro_GetCachedStatus(&s);
    identity(r,DRV_COMPONENT_BARO,"气压计","SPL06",0);
    r->code=s.init_status;r->stage=s.report_done?"initialization":"not initialized";
    r->state=!s.report_done?DRV_COMPONENT_WAITING:(s.init_status?DRV_COMPONENT_FAULT:DRV_COMPONENT_READY);
    r->note="Cached identity / init result";
    integer(r,"chip_id","",s.product_id,1,s.report_done);
}
static void mag(DRV_ComponentRecord *r)
{
    APP_MAG_Status s;APP_MAG_GetStatus(&s);
    identity(r,DRV_COMPONENT_MAG,"磁力计",APP_MAG_GetTypeName(s.type),0);
    r->code=s.initialized?s.last_status:s.init_status;r->samples=s.sample_count;
    r->state=s.initialized?(s.last_status?DRV_COMPONENT_FAULT:DRV_COMPONENT_READY):
             (s.init_status?DRV_COMPONENT_FAULT:DRV_COMPONENT_WAITING);
    r->stage=s.initialized?"sampling":"initialization";
    integer(r,"address","",s.address,1,s.type!=0);
    number(r,"X","mG",(float)s.x_mgauss,s.initialized&&s.sample_count&&s.last_status==0);
    number(r,"Y","mG",(float)s.y_mgauss,s.initialized&&s.sample_count&&s.last_status==0);
    number(r,"Z","mG",(float)s.z_mgauss,s.initialized&&s.sample_count&&s.last_status==0);
}
static void flow(DRV_ComponentRecord *r)
{
    APP_OPTICAL_FLOW_Status s;APP_OpticalFlow_GetStatus(&s);
    identity(r,DRV_COMPONENT_FLOW,"光流与测距","MicoLink",0);
    r->code=s.init_status;r->samples=s.frames;r->age_ms=s.frames?s.age_ms:UINT32_MAX;
    r->state=s.health==APP_OPTICAL_FLOW_HEALTH_FAILED?DRV_COMPONENT_FAULT:
             (s.initialized&&s.valid?DRV_COMPONENT_READY:DRV_COMPONENT_WAITING);
    r->stage=s.initialized?"receiving":"initialization";
    number(r,"height","m",s.height_m,s.distance_valid);
    number(r,"VX","m/s",s.vx_m_s,s.velocity_valid);
    number(r,"VY","m/s",s.vy_m_s,s.velocity_valid);
    integer(r,"checksum_err","",s.checksum_errors,0,1);
}
static void current(DRV_ComponentRecord *r)
{
    APP_CurrentSnapshot s;APP_Current_GetSnapshot(&s);
    identity(r,DRV_COMPONENT_CURRENT,"电流计","AM32 Curr",0);
    r->code=s.adc_status;r->age_ms=s.age_ms;r->samples=s.samples;
    r->state=s.adc_status>=2||s.reading.saturated?DRV_COMPONENT_FAULT:
             (s.reading.valid?DRV_COMPONENT_READY:DRV_COMPONENT_WAITING);
    r->stage=s.reading.saturated?"ADC saturated":"ADC sampling";
    r->note=s.reading.calibrated?"calibrated flag set":"nominal conversion / uncalibrated";
    number(r,"current","A",s.reading.current_a,s.reading.valid);
    number(r,"ADC","V",s.reading.adc_v,s.reading.valid);
    integer(r,"raw","count",s.reading.raw,0,s.reading.valid);
    integer(r,"errors","",s.errors,0,1);
}
static void params(DRV_ComponentRecord *r)
{
    const APP_ControlConfig *s=(const APP_ControlConfig *)app_control_internal_config_view();
    identity(r,DRV_COMPONENT_PARAMS,"参数存储","Internal Flash",0);
    r->code=s->last_flash_status;r->state=s->last_flash_status?DRV_COMPONENT_FAULT:DRV_COMPONENT_READY;
    r->stage=s->flash_valid?"stored config":"default / RAM";
    integer(r,"stored_valid","",s->flash_valid,0,1);integer(r,"loaded","",s->loaded_from_flash,0,1);
}
static void logs(DRV_ComponentRecord *r)
{
    identity(r,DRV_COMPONENT_LOG,"日志存储","SD raw blocks",0);
    uint8_t ready=APP_FlashService_IsLogStorageReady();
    r->state=ready?DRV_COMPONENT_READY:DRV_COMPONENT_WAITING;r->stage=ready?"initialized":"not ready";
    integer(r,"ready","",ready,0,1);
}
static void esc(DRV_ComponentRecord *r)
{
    identity(r,DRV_COMPONENT_ESC,"电调输出",BSP_PWM_EscProtocol(),0);
    r->note="Command / TX state; no ESC acknowledgement";
#if BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300
    BSP_DShotSnapshot s;BSP_DShot_GetSnapshot(&s);
    r->state=s.fault?DRV_COMPONENT_FAULT:(s.timer_clock_hz?DRV_COMPONENT_READY:DRV_COMPONENT_WAITING);
    r->stage=s.enabled_mask?"outputs enabled":"outputs disabled";r->samples=s.completed;
    integer(r,"upper_code","",s.code[0],0,s.submitted!=0);
    integer(r,"lower_code","",s.code[1],0,s.submitted!=0);
    integer(r,"enabled","mask",s.enabled_mask,1,1);integer(r,"errors","",s.errors,0,1);
#else
    r->state=BSP_PWM_IsInitialized()?DRV_COMPONENT_READY:DRV_COMPONENT_WAITING;r->stage="PWM command";
    integer(r,"upper","us",BSP_PWM_GetEscPulse(1),0,1);integer(r,"lower","us",BSP_PWM_GetEscPulse(2),0,1);
#endif
}
static void servo(DRV_ComponentRecord *r)
{
    uint8_t pwm=APP_ServoBusGuard_IsPwmMode();
    identity(r,DRV_COMPONENT_SERVO,"舵机输出",pwm?"PWM":"bus servo",pwm);
    r->state=pwm&&BSP_PWM_IsInitialized()?DRV_COMPONENT_READY:DRV_COMPONENT_UNKNOWN;
    r->stage="configured output";r->note="Command values, not measured position";
    integer(r,"alpha","us",BSP_PWM_GetServoPulse(1),0,pwm);
    integer(r,"beta","us",BSP_PWM_GetServoPulse(2),0,pwm);
}
static void rc(DRV_ComponentRecord *r)
{
    APP_Stabilizer_ArmStatus status;APP_Stabilizer_GetArmStatus(&status);
    identity(r,DRV_COMPONENT_RC,"遥控接收","CRSF / ELRS",0);
    r->samples=DRV_ELRS_GetRcFrames();
    if (r->samples) { r->age_ms=SVC_Timestamp_Ms()-DRV_ELRS_GetLastRcMs(); }
    r->state=!status.published?DRV_COMPONENT_UNKNOWN:(status.rc_link_ok?DRV_COMPONENT_READY:
             (status.rc_link_seen?DRV_COMPONENT_FAULT:DRV_COMPONENT_WAITING));
    r->stage=r->samples?"RC frames":"no RC frames";
    integer(r,"CRC errors","",DRV_ELRS_GetCrcErrors(),0,1);
}
static void uart(DRV_ComponentRecord *r)
{
    uint32_t bytes,lines,overflow,errors;APP_UART_GetStats(&bytes,&lines,&overflow,&errors);
    identity(r,DRV_COMPONENT_UART,"数传接口","Serial link",0);
    r->state=BSP_UartLink_RxIsRunning(BSP_UART_ROLE_TELEMETRY)?DRV_COMPONENT_READY:DRV_COMPONENT_WAITING;
    r->stage="receive state";r->samples=lines;
    integer(r,"bytes","B",bytes,0,1);integer(r,"errors","",errors,0,1);integer(r,"overflow","",overflow,0,1);
}
static void bt(DRV_ComponentRecord *r)
{
    identity(r,DRV_COMPONENT_BT,"维护接口","Bluetooth UART",0);
    r->state=BSP_UartLink_RxIsRunning(BSP_UART_ROLE_MAINT)?DRV_COMPONENT_READY:DRV_COMPONENT_WAITING;
    r->stage="receive state";r->note="UART readiness, not radio connection proof";
}
static void led(DRV_ComponentRecord *r)
{
    APP_LED_Debug s;APP_LED_GetDebug(&s);
    identity(r,DRV_COMPONENT_LED,"状态灯","RGB LED",0);
    r->samples=s.ticks;r->state=s.ticks?DRV_COMPONENT_READY:DRV_COMPONENT_WAITING;r->stage="software modulation";
    integer(r,"source","",s.source,0,1);integer(r,"active_low","",s.active_low,0,1);
}
static void gps(DRV_ComponentRecord *r)
{
    APP_GPS_Status s;APP_GPS_GetStatus(&s);
    identity(r,DRV_COMPONENT_GPS,"GNSS","UBX / NMEA",0);
    r->code=s.init_status;r->samples=s.packets;r->age_ms=s.bytes?SVC_Timestamp_Ms()-s.last_rx_ms:UINT32_MAX;
    r->state=s.init_status?DRV_COMPONENT_FAULT:(s.valid_fix?DRV_COMPONENT_READY:DRV_COMPONENT_WAITING);
    r->stage=s.initialized?"waiting / receiving":"initialization";
    integer(r,"satellites","",s.num_sv,0,s.bytes!=0);integer(r,"fix_valid","",s.valid_fix,0,1);
}
void APP_Components_RegisterBoard(void)
{
    (void)APP_Components_Register(DRV_COMPONENT_IMU,imu);
    (void)APP_Components_Register(DRV_COMPONENT_BARO,baro);
    (void)APP_Components_Register(DRV_COMPONENT_MAG,mag);
    (void)APP_Components_Register(DRV_COMPONENT_FLOW,flow);
    (void)APP_Components_Register(DRV_COMPONENT_CURRENT,current);
    (void)APP_Components_Register(DRV_COMPONENT_PARAMS,params);
    (void)APP_Components_Register(DRV_COMPONENT_LOG,logs);
    (void)APP_Components_Register(DRV_COMPONENT_ESC,esc);
    (void)APP_Components_Register(DRV_COMPONENT_SERVO,servo);
    (void)APP_Components_Register(DRV_COMPONENT_RC,rc);
    (void)APP_Components_Register(DRV_COMPONENT_UART,uart);
    (void)APP_Components_Register(DRV_COMPONENT_BT,bt);
    (void)APP_Components_Register(DRV_COMPONENT_LED,led);
    /* No external NOR on this board. GPS registers only when its task starts. */
}
void APP_Components_RegisterGps(void) { (void)APP_Components_Register(DRV_COMPONENT_GPS,gps); }
