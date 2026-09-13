"""CURRENT decimal contract without target floating printf."""
from pathlib import Path
import subprocess
import shutil

ROOT = Path(__file__).resolve().parents[1]


def test_fixed_current_format_on_host(tmp_path):
    source = r'''
#include "app_current_format.h"
#include <assert.h>
#include <string.h>
int main(void){
    struct { char before; char text[16]; char after; } b={'x',"",'y'};
    assert(APP_Current_FormatFixed(b.text,0.604257f,5));assert(!strcmp(b.text,"0.60426"));
    assert(APP_Current_FormatFixed(b.text,47.39272f,3));assert(!strcmp(b.text,"47.393"));
    assert(APP_Current_FormatFixed(b.text,-0.125f,3));assert(!strcmp(b.text,"-0.125"));
    assert(APP_Current_FormatFixed(b.text,0.0f,3));assert(!strcmp(b.text,"0.000"));
    assert(!APP_Current_FormatFixed(b.text,NAN,3));assert(!strcmp(b.text,"nan"));
    assert(!APP_Current_FormatFixed(b.text,INFINITY,5));assert(!strcmp(b.text,"nan"));
    assert(!APP_Current_FormatFixed(b.text,1e30f,3));assert(!strcmp(b.text,"nan"));
    assert(b.before=='x' && b.after=='y');return 0;
}'''
    (tmp_path/'test.c').write_text(source)
    result = subprocess.run([shutil.which('gcc'),'-std=c11','-O2','-Wall','-Wextra','-Werror',
                             '-I',str(ROOT/'App/Inc'),str(tmp_path/'test.c'),'-lm','-o',str(tmp_path/'test.exe')],capture_output=True,text=True)
    assert result.returncode == 0, result.stdout+result.stderr
    subprocess.run([str(tmp_path/'test.exe')],check=True)


def test_current_report_does_not_require_printf_float():
    source=(ROOT/'App/Src/app_current.c').read_text(encoding='utf-8').split('void APP_Current_Report(void)')[1]
    assert 'APP_Current_FormatFixed' in source
    assert 'adc_v=%s current_a=%s' in source
    assert '%.5f' not in source and '%.3f' not in source
