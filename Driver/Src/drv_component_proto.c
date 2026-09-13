#include "drv_component_proto.h"
#include <string.h>
#include <math.h>

typedef struct { uint8_t *p; uint16_t cap, pos; uint8_t ok; } Writer;
static void put(Writer *w, uint32_t value, unsigned bytes)
{
    if (!w->ok || w->pos + bytes > w->cap) { w->ok=0; return; }
    for (unsigned i=0; i<bytes; ++i) { w->p[w->pos++]=(uint8_t)(value>>(8U*i)); }
}
static void string(Writer *w, const char *text, unsigned limit)
{
    unsigned n=0;
    if (text == NULL) { text=""; }
    while (n<=limit && text[n]) { ++n; }
    if (n>limit) { w->ok=0; return; }
    put(w,n,1);
    for (unsigned i=0;i<n;i++) { put(w,(uint8_t)text[i],1); }
}

uint8_t DRV_Component_Encode(uint8_t kind, uint32_t nonce, uint16_t index,
                            uint16_t count, const DRV_ComponentRecord *r,
                            uint32_t trailer, uint8_t *out, uint16_t capacity,
                            uint16_t *length)
{
    if (length == NULL) { return 0; }
    *length=0;
    if (out == NULL || count>DRV_COMPONENT_MAX_COUNT || kind<1 || kind>4) { return 0; }
    if ((kind==DRV_COMPONENT_BEGIN && index!=0) ||
        (kind==DRV_COMPONENT_END && index!=count) || index>count) { return 0; }
    Writer w={out,capacity<DRV_COMPONENT_MAX_PAYLOAD?capacity:DRV_COMPONENT_MAX_PAYLOAD,0,1};
    if (kind==DRV_COMPONENT_RECORD && (r==NULL || index>=count || r->id==0 ||
        r->state>DRV_COMPONENT_FAULT || r->field_count>DRV_COMPONENT_MAX_FIELDS ||
        r->name==NULL || !r->name[0])) { return 0; }
    put(&w,DRV_COMPONENT_VERSION,1);put(&w,kind,1);put(&w,count,2);put(&w,nonce,4);put(&w,index,2);
    if (kind==DRV_COMPONENT_RECORD) {
        put(&w,r->id,2);put(&w,r->state,1);put(&w,r->field_count,1);
        put(&w,(uint32_t)r->code,4);put(&w,r->age_ms,4);put(&w,r->samples,4);
        string(&w,r->name,32);string(&w,r->model,24);string(&w,r->bus,24);
        string(&w,r->stage,20);string(&w,r->note,64);
        for (unsigned i=0;i<r->field_count;i++) {
            const DRV_ComponentField *f=&r->fields[i];uint32_t bits;
            if (f->label==NULL || !f->label[0] || f->type<1 || f->type>4 || f->valid>1 ||
                (f->type==DRV_COMPONENT_F32 && f->valid && !isfinite(f->value.f))) { return 0; }
            for (unsigned j=0;j<i;j++) {
                if (strcmp(f->label,r->fields[j].label)==0) { return 0; }
            }
            string(&w,f->label,16);string(&w,f->unit,8);
            put(&w,f->type,1);put(&w,f->valid,1);
            memcpy(&bits,&f->value,sizeof(bits));put(&w,bits,4);
        }
    } else if (kind==DRV_COMPONENT_END || kind==DRV_COMPONENT_ERROR) { put(&w,trailer,4); }
    if (!w.ok) { return 0; }
    *length=w.pos;return 1;
}

uint32_t DRV_Component_CrcUpdate(uint32_t crc, const uint8_t *bytes, size_t size)
{
    for (size_t i=0;i<size;i++) {
        crc^=bytes[i];
        for (unsigned bit=0;bit<8;bit++) { crc=(crc>>1)^((crc&1U)?0xEDB88320U:0U); }
    }
    return crc;
}
