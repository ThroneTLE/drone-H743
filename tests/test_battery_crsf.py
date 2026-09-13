"""Actual ELRS battery packing and DMA admission, including full unavailable values."""
from pathlib import Path
import subprocess
import shutil

ROOT=Path(__file__).resolve().parents[1]


def c_function(text,signature):
    start=text.index(signature);brace=text.index('{',start);depth=0
    for end in range(brace,len(text)):
        depth += (text[end]=='{')-(text[end]=='}')
        if depth==0:return text[start:end+1]
    raise AssertionError(signature)


def test_real_crsf_battery_bytes_and_busy_admission(tmp_path):
    source=(ROOT/'App/Src/app_elrs.c').read_text(encoding='utf-8')
    functions='\n'.join(c_function(source,sig) for sig in ('static uint8_t StartTxDma(',
        'static uint8_t SendTelemetryFrame(','uint8_t APP_ELRS_SendTelemetryBattery(','void APP_ELRS_OnTxComplete('))
    code=r'''
#include "drv_elrs.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
#define BSP_UART_ROLE_RC 1
static uint8_t dma_started,tx_busy,tx_len,tx_buf[CRSF_MAX_FRAME_SIZE];
static uint32_t rx_errors,depth,starts;static int tx_ok=1;
uint32_t BSP_Critical_Enter(void){return depth++;}
void BSP_Critical_Exit(uint32_t old){depth=old;}
int BSP_UartLink_HasTxDma(int role){assert(role==1&&depth);return 1;}
void BSP_Cache_CleanDCache(const void *p,uint32_t n){assert(p==tx_buf&&n==12&&depth);}
int BSP_UartLink_TransmitDma(int role,const uint8_t *p,uint16_t n){assert(role==1&&p==tx_buf&&n==12&&depth);starts++;return tx_ok;}
''' + functions + r'''
int main(int argc,char **argv){
    assert(argc==2);assert(!APP_ELRS_SendTelemetryBattery(120,474,0xFFFFFF,0xFF));assert(!starts);
    dma_started=1;assert(APP_ELRS_SendTelemetryBattery(120,474,0xFFFFFF,0xFF));
    FILE *f=fopen(argv[1],"wb");assert(f);fwrite(tx_buf,1,tx_len,f);fclose(f);
    uint8_t old[64];memcpy(old,tx_buf,64);
    assert(!APP_ELRS_SendTelemetryBattery(110,0,0,0));assert(starts==1&&!memcmp(old,tx_buf,64));
    APP_ELRS_OnTxComplete();tx_ok=0;assert(!APP_ELRS_SendTelemetryBattery(0xFFFF,0xFFFF,0xFFFFFF,0xFF));
    assert(!tx_busy&&rx_errors==1&&!depth);return 0;
}
'''
    (tmp_path/'test.c').write_text(code,encoding='utf-8')
    command=[shutil.which('gcc'),'-std=c11','-Wall','-Wextra','-Werror','-I',str(ROOT/'Driver/Inc'),
             str(tmp_path/'test.c'),str(ROOT/'Driver/Src/drv_elrs.c'),'-o',str(tmp_path/'test.exe')]
    result=subprocess.run(command,capture_output=True,text=True);assert result.returncode==0,result.stdout+result.stderr
    subprocess.run([str(tmp_path/'test.exe'),str(tmp_path/'frame.bin')],check=True)
    wire=(tmp_path/'frame.bin').read_bytes()
    payload=bytes.fromhex('08 00 78 01 da ff ff ff ff')
    crc=0
    for byte in payload:
        crc^=byte
        for _ in range(8):crc=((crc<<1)^0xd5)&255 if crc&128 else (crc<<1)&255
    assert wire==bytes([0xc8,10])+payload+bytes([crc])
