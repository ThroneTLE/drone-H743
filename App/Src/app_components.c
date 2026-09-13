#include "app_components.h"
#include "bsp_critical.h"
#include <string.h>

typedef struct { uint16_t id; APP_ComponentSample sample; } Entry;
static Entry entries[DRV_COMPONENT_MAX_COUNT];
static uint16_t used;
static uint8_t initialized, registration_error;

static uint8_t register_unlocked(uint16_t id, APP_ComponentSample sample)
{
    if (!id || !sample) { registration_error=1;return 0; }
    for (unsigned i=0;i<used;i++) {
        if(entries[i].id==id) {
            if (entries[i].sample==sample) { return 1; }
            registration_error=1;return 0;
        }
    }
    if (used==DRV_COMPONENT_MAX_COUNT) { registration_error=1;return 0; }
    entries[used].id=id;entries[used].sample=sample;++used;return 1;
}
uint8_t APP_Components_Register(uint16_t id, APP_ComponentSample sample)
{
    uint32_t lock=BSP_Critical_Enter();
    uint8_t ok=register_unlocked(id,sample);
    BSP_Critical_Exit(lock);return ok;
}
void APP_Components_Init(void)
{
    uint32_t lock=BSP_Critical_Enter();
    if (!initialized) { initialized=1;APP_Components_RegisterBoard(); }
    BSP_Critical_Exit(lock);
}
static uint8_t emit(uint8_t kind,uint32_t nonce,uint16_t index,uint16_t count,const DRV_ComponentRecord *r,uint32_t trailer,uint32_t *crc)
{
    uint8_t payload[DRV_COMPONENT_MAX_PAYLOAD];uint16_t len;
    if (!DRV_Component_Encode(kind,nonce,index,count,r,trailer,payload,sizeof(payload),&len)) { return 0; }
    if (crc) { *crc=DRV_Component_CrcUpdate(*crc,payload,len); }
    if (!APP_Components_Send(payload,len)) { return 0; }
    return 1;
}
void APP_Components_Report(uint32_t nonce)
{
    APP_Components_Init();
    Entry snapshot[DRV_COMPONENT_MAX_COUNT];
    uint32_t lock=BSP_Critical_Enter();
    uint16_t count=used;uint8_t invalid=registration_error;
    memcpy(snapshot,entries,count*sizeof(Entry));BSP_Critical_Exit(lock);
    if (invalid || APP_Components_ExportBusy()) {
        (void)emit(DRV_COMPONENT_ERROR,nonce,0,count,NULL,invalid?2U:1U,NULL);return;
    }
    uint32_t crc=0xFFFFFFFFU;
    uint32_t start=APP_Components_NowMs();
    if (!emit(DRV_COMPONENT_BEGIN,nonce,0,count,NULL,0,NULL)) { return; }
    for (uint16_t i=0;i<count;i++) {
        if ((uint32_t)(APP_Components_NowMs()-start)>300U) {
            (void)emit(DRV_COMPONENT_ERROR,nonce,i,count,NULL,5U,NULL);return;
        }
        DRV_ComponentRecord r={0};r.age_ms=UINT32_MAX;
        lock=BSP_Critical_Enter();snapshot[i].sample(&r);BSP_Critical_Exit(lock);r.id=snapshot[i].id;
        if (!emit(DRV_COMPONENT_RECORD,nonce,i,count,&r,0,&crc)) {
            (void)emit(DRV_COMPONENT_ERROR,nonce,i,count,NULL,3U,NULL);return;
        }
    }
    (void)emit(DRV_COMPONENT_END,nonce,count,count,NULL,~crc,NULL);
}
uint8_t APP_Components_Command(char **tokens,uint32_t count)
{
    if (!tokens || !count || !tokens[0] || strcmp(tokens[0],"REGISTRY?")!=0) { return 0; }
    uint32_t nonce=0;
    if (count>2) { (void)emit(DRV_COMPONENT_ERROR,0,0,0,NULL,4U,NULL);return 1; }
    if (count==2) {
        const char *p=tokens[1];
        if (!p || !*p) { (void)emit(DRV_COMPONENT_ERROR,0,0,0,NULL,4U,NULL);return 1; }
        while (*p) {
            if (*p<'0' || *p>'9' || nonce>(UINT32_MAX-(uint32_t)(*p-'0'))/10U) {
                (void)emit(DRV_COMPONENT_ERROR,0,0,0,NULL,4U,NULL);return 1;
            }
            nonce=nonce*10U+(uint32_t)(*p++-'0');
        }
    }
    APP_Components_Report(nonce);return 1;
}
