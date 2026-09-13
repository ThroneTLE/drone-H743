#include "app_battery.h"
#include "app_current.h"
#include "app_control.h"
#include "app_diag_binary.h"
#include "app_proto.h"
#include "app_stabilizer.h"
#include <math.h>
#include <string.h>
#include <limits.h>
static uint32_t config_generation;
static void put32(uint8_t *p,uint32_t value)
{
    for (unsigned i=0;i<4;i++) { p[i]=(uint8_t)(value>>(i*8U)); }
}
static uint8_t report(uint32_t nonce,uint8_t result_flags)
{
    APP_BatterySnapshot b;APP_CurrentSnapshot c;uint32_t accepted,rejected;
    APP_Battery_GetSnapshot(&b);APP_Current_GetSnapshot(&c);APP_Battery_GetTxStats(&accepted,&rejected);
    APP_Stabilizer_ArmStatus arm;APP_Stabilizer_GetArmStatus(&arm);
    uint8_t payload[60]={1U,0U,b.state.config.cells,b.state.adc_status};
    payload[1]=(uint8_t)(result_flags | b.state.valid | (b.state.low<<1) | (b.can_arm<<2) |
                        (b.state.saturated<<3) | ((c.reading.valid!=0U)<<4));
    int32_t current_ma=INT32_MIN;
    if (c.reading.valid && isfinite(c.reading.current_a) && fabsf(c.reading.current_a)<1000000.0f) {
        double milliamperes=(double)c.reading.current_a*1000.0;
        current_ma=(int32_t)(milliamperes+(milliamperes>=0.0?0.5:-0.5));
    }
    if (current_ma==INT32_MIN) { payload[1]&=(uint8_t)~16U; }
    const uint32_t words[]={nonce,b.age_ms,b.state.voltage_mv,b.state.raw,b.state.samples,b.state.errors,
        b.state.config.low_cell_mv,b.state.config.recover_cell_mv,(uint32_t)current_ma,c.age_ms,
        accepted,rejected,config_generation,arm.published?(arm.armed?2U:1U):0U};
    for (unsigned i=0;i<14;i++) { put32(payload+4+i*4,words[i]); }
    return APP_Diag_SendBinary(APP_PROTO_MSG_BATTERY,payload,sizeof(payload));
}
uint8_t APP_Battery_SendReport(uint32_t nonce) { return report(nonce,0U); }
static uint8_t integer(const char *text,uint32_t *value)
{
    if (!text || !*text) { return 0; }
    uint32_t result=0;
    while (*text) {
        if (*text<'0' || *text>'9' || result>(UINT32_MAX-(uint32_t)(*text-'0'))/10U) { return 0; }
        result=result*10U+(uint32_t)(*text++-'0');
    }
    *value=result;return 1;
}
uint8_t APP_Battery_Command(char **tokens,uint32_t count)
{
    if (!tokens || !count || !tokens[0]) { return 0; }
    if (strcmp(tokens[0],"BATTERY?")==0) {
        uint32_t nonce=0;
        if (count>2 || (count==2 && !integer(tokens[1],&nonce))) {
            APP_Control_QueueText("ERR BATTERY code=ARG\r\n");return 1;
        }
        (void)report(nonce,0U);return 1;
    }
    if (strcmp(tokens[0],"BATTERY")!=0) { return 0; }
    uint32_t nonce=0,cells=0,low=0,recover=0;
    uint8_t good=(count==6 && tokens[1]!=NULL && strcmp(tokens[1],"SET")==0 && integer(tokens[2],&nonce) &&
                 integer(tokens[3],&cells) && integer(tokens[4],&low) && integer(tokens[5],&recover));
    if (good && cells<=UINT8_MAX && low<=UINT16_MAX && recover<=UINT16_MAX &&
        APP_Battery_Configure((DRV_BatteryConfig){(uint8_t)cells,(uint16_t)low,(uint16_t)recover})) {
        config_generation++;(void)report(nonce,32U);
    } else {
        APP_Control_QueueText("ERR BATTERY id=%lu code=CONFIG_REJECTED\r\n",(unsigned long)nonce);
        (void)report(nonce,64U);
    }
    return 1;
}
