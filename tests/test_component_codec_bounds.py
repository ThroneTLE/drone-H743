"""Real C registry bounds, cancellation and immutable enumeration contracts."""
from pathlib import Path
import subprocess
import shutil

ROOT=Path(__file__).resolve().parents[1]


def test_codec_and_manager_boundaries(tmp_path):
    code=r'''
#include "app_components.h"
#include <assert.h>
#include <math.h>
#include <string.h>
static unsigned depth,count,mode,records,last_kind;
static uint32_t now;
uint32_t BSP_Critical_Enter(void){return depth++;}
void BSP_Critical_Exit(uint32_t d){depth=d;}
uint32_t APP_Components_NowMs(void){return now;}
uint8_t APP_Components_ExportBusy(void){return 0;}
static void late(DRV_ComponentRecord *r){assert(depth);r->name="Late";}
static void early(DRV_ComponentRecord *r){
    assert(depth);r->name="Early";
    assert(APP_Components_Register(200,late)); /* Appears in NEXT snapshot only. */
}
void APP_Components_RegisterBoard(void){assert(APP_Components_Register(1,early));}
uint8_t APP_Components_Send(const uint8_t *p,uint16_t n){
    assert(!depth&&n>=10);count++;last_kind=p[1];
    if(mode==1&&count==1)now+=301;
    if(p[1]==2){records++;if(mode==2)return 0;}
    return 1;
}
int main(void){
    struct {uint8_t before, data[247], after;} out={0x31,{0},0x72};
    uint16_t n;DRV_ComponentRecord r={.id=1,.name="Sensor"};
    assert(DRV_Component_Encode(2,42,0,1,&r,0,out.data,sizeof(out.data),&n));
    unsigned required=n;
    for(unsigned size=0;size<required;size++){
        memset(out.data,0xcc,sizeof(out.data));
        assert(!DRV_Component_Encode(2,42,0,1,&r,0,out.data,size,&n)&&n==0);
        for(unsigned i=size;i<sizeof(out.data);i++)assert(out.data[i]==0xcc);
    }
    assert(out.before==0x31&&out.after==0x72);
    assert(!DRV_Component_Encode(1,0,1,2,0,0,out.data,247,&n));
    assert(!DRV_Component_Encode(3,0,0,1,0,0,out.data,247,&n));
    assert(!DRV_Component_Encode(1,0,0,17,0,0,out.data,247,&n));
    assert(!DRV_Component_Encode(2,0,1,1,&r,0,out.data,247,&n));
    assert(!DRV_Component_Encode(2,0,0,1,&r,0,NULL,247,&n));
    r.name="01234567890123456789012345678901234";
    assert(!DRV_Component_Encode(2,0,0,1,&r,0,out.data,247,&n));r.name="Sensor";
    r.field_count=1;r.fields[0]=(DRV_ComponentField){.label="x",.type=3,.valid=1,.value.f=NAN};
    assert(!DRV_Component_Encode(2,0,0,1,&r,0,out.data,247,&n));
    r.fields[0].valid=0;assert(DRV_Component_Encode(2,0,0,1,&r,0,out.data,247,&n));
    r.field_count=2;r.fields[1]=r.fields[0];
    assert(!DRV_Component_Encode(2,0,0,1,&r,0,out.data,247,&n));
    APP_Components_Report(1);assert(records==1&&last_kind==3&&count==3);
    records=count=0;APP_Components_Report(2);assert(records==2&&last_kind==3&&count==4);
    records=count=0;mode=1;APP_Components_Report(3);assert(!records&&last_kind==4&&count==2);
    records=count=0;mode=2;APP_Components_Report(4);assert(records==1&&last_kind==4&&count==3);
    char *null_token[]={NULL};assert(!APP_Components_Command(null_token,1));
    assert(!APP_Components_Command(NULL,0));return 0;
}
'''
    (tmp_path/'test.c').write_text(code)
    command=[shutil.which('gcc'),'-std=c11','-Wall','-Wextra','-Werror']
    for directory in ('App/Inc','Driver/Inc','BSP/Inc'):command+=['-I',str(ROOT/directory)]
    command+=[str(tmp_path/'test.c'),str(ROOT/'Driver/Src/drv_component_proto.c'),str(ROOT/'App/Src/app_components.c'),'-lm','-o',str(tmp_path/'test.exe')]
    result=subprocess.run(command,capture_output=True,text=True)
    assert result.returncode==0,result.stdout+result.stderr
    subprocess.run([str(tmp_path/'test.exe')],check=True)
