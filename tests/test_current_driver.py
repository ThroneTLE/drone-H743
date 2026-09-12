"""Compile the real current conversion driver; no ADC/board access."""
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]

HARNESS = r"""
#include "drv_current.h"
#include <math.h>
#include <float.h>
#include <stdio.h>
#include <string.h>
#define CHECK(c) do { if(!(c)) { fprintf(stderr,"line %d: %s\n",__LINE__,#c); return 1; } } while(0)
int main(void) {
    DRV_CurrentConfig c=DRV_Current_Am32_55A_Default();
    DRV_CurrentReading r;
    CHECK(!c.calibrated && c.adc_max==65535 && fabsf(c.sensitivity_v_per_a-0.01275f)<1e-8f);
    CHECK(DRV_Current_Convert(&c,0,&r)==DRV_CURRENT_OK && r.valid && r.current_a==0);
    /* 1.275 V / 12.75 mV/A = 100 A, within one ADC quantum. */
    unsigned raw=(unsigned)roundf(1.275f/3.3f*65535.0f);
    CHECK(DRV_Current_Convert(&c,raw,&r)==DRV_CURRENT_OK);
    CHECK(fabsf(r.current_a-100.0f)<0.004f && !r.calibrated);
    float prev=-1.0f;
    for(unsigned i=0;i<65535;i++) {
        CHECK(DRV_Current_Convert(&c,i,&r)==DRV_CURRENT_OK);
        CHECK(r.current_a>prev && r.valid); prev=r.current_a;
    }
    CHECK(DRV_Current_Convert(&c,65535,&r)==DRV_CURRENT_SATURATED);
    CHECK(!r.valid && r.saturated && isnan(r.current_a) && r.adc_v==c.reference_v);
    c.reference_v=3.0f; c.input_scale=2.0f; c.zero_offset_v=0.1f; c.calibrated=1;
    CHECK(DRV_Current_Convert(&c,32768,&r)==DRV_CURRENT_OK);
    CHECK(fabsf(r.current_a-((32768.0f/65535.0f*3*2-.1f)/.01275f))<1e-4f && r.calibrated);
    CHECK(DRV_Current_Convert(&c,0,&r)==DRV_CURRENT_OK && r.current_a<0);
    DRV_CurrentReading before; memset(&r,0x5A,sizeof(r)); memcpy(&before,&r,sizeof(r));
    CHECK(DRV_Current_Convert(NULL,0,&r)==DRV_CURRENT_INVALID);
    CHECK(DRV_Current_Convert(&c,65536,&r)==DRV_CURRENT_INVALID);
    CHECK(DRV_Current_Convert(&c,0,NULL)==DRV_CURRENT_INVALID);
    CHECK(memcmp(&before,&r,sizeof(r))==0);
    for(unsigned k=0;k<8;k++) {
        c=DRV_Current_Am32_55A_Default();
        if(k==0)c.sensitivity_v_per_a=0;
        if(k==1)c.reference_v=NAN;
        if(k==2)c.zero_offset_v=INFINITY;
        if(k==3)c.input_scale=-1;
        if(k==4)c.adc_max=0;
        if(k==5)c.adc_max=65536;
        if(k==6)c.calibrated=2;
        if(k==7)c.sensitivity_v_per_a=FLT_MIN*0.25f;
        CHECK(DRV_Current_Convert(&c,65534,&r)==DRV_CURRENT_INVALID);
        CHECK(memcmp(&before,&r,sizeof(r))==0);
    }
    puts("current conversion: nominal, calibrated, full range, clipping and invalid inputs passed");
    return 0;
}
"""


def test_current_conversion_on_real_c(tmp_path):
    source=tmp_path/"current.c"; source.write_text(HARNESS)
    exe=tmp_path/"current.exe"
    result=subprocess.run([shutil.which("gcc"),"-std=c11","-O2","-Wall","-Wextra","-Werror",
        "-I",str(ROOT/"Driver/Inc"),str(source),str(ROOT/"Driver/Src/drv_current.c"),"-lm","-o",str(exe)],
        capture_output=True,text=True)
    assert result.returncode==0,result.stdout+result.stderr
    result=subprocess.run([str(exe)],capture_output=True,text=True)
    assert result.returncode==0,result.stdout+result.stderr


def test_current_driver_has_no_hardware_dependency():
    source=(ROOT/"Driver/Src/drv_current.c").read_text()
    assert "HAL_" not in source and "FreeRTOS" not in source
    assert "0.01275f" in source and "65535U" in source
