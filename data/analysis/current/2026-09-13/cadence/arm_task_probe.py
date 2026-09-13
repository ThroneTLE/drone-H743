"""Exercise actual linked message-task entries for 60 simulated seconds, no board.

PYTHONPATH=.tmp/arm-emulator python .../arm_task_probe.py build/Debug/drone-H743.elf
ADC and OS sleep are injected; the background worker is never scheduled.
"""
import json
import re
import struct
import sys
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_HOOK_CODE
from unicorn.arm_const import *
from elftools.elf.elffile import ELFFile

with open(sys.argv[1],'rb') as stream:
    elf=ELFFile(stream)
    symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
    cpu=Uc(UC_ARCH_ARM,UC_MODE_THUMB);cpu.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_A15)
    for address,size in ((0x08000000,0x200000),(0x20000000,0x20000),(0x21000000,0x10000),
                         (0x24000000,0x100000),(0x30000000,0x80000),(0x38000000,0x10000)):
        cpu.mem_map(address,size)
    for segment in elf.iter_segments():
        if segment['p_type']=='PT_LOAD' and segment['p_filesz']:
            cpu.mem_write(segment['p_vaddr'],segment.data())
cpu.reg_write(UC_ARM_REG_C1_C0_2,0xf<<20);cpu.reg_write(UC_ARM_REG_FPEXC,0x40000000)
cpu.mem_write(symbols['uartTxQueueHandle'],struct.pack('<I',1))
now=0;reads=0;initializations=0;output=[]
names=('BSP_Current_Init','BSP_Current_Read','APP_USB_CDC_Write','osMessageQueuePut',
       'osMessageQueueGet','APP_UART_NotifyTxPending','BSP_Critical_Enter','BSP_Critical_Exit','osDelay')
hooks={symbols[name]&~1:name for name in names}
stop=0x081fff00;cpu.mem_write(stop,b'\x00\xbe')


def trace(cpu,address,size,data):
    global now,reads,initializations
    if address==stop:cpu.emu_stop();return
    name=hooks.get(address)
    if name is None:return
    if name=='BSP_Current_Init':initializations+=1
    if name=='BSP_Current_Read':
        reads+=1;cpu.mem_write(cpu.reg_read(UC_ARM_REG_R0),struct.pack('<I',0))
    if name=='osDelay':
        ticks=cpu.reg_read(UC_ARM_REG_R0);assert ticks==20
        now+=ticks;cpu.mem_write(symbols['uwTick'],struct.pack('<I',now))
    if name=='APP_USB_CDC_Write':
        output.append(bytes(cpu.mem_read(cpu.reg_read(UC_ARM_REG_R0),cpu.reg_read(UC_ARM_REG_R1))).decode('ascii'))
    cpu.reg_write(UC_ARM_REG_R0,0);cpu.reg_write(UC_ARM_REG_PC,cpu.reg_read(UC_ARM_REG_LR))


cpu.hook_add(UC_HOOK_CODE,trace)


def call(name):
    cpu.reg_write(UC_ARM_REG_SP,0x2100fff0);cpu.reg_write(UC_ARM_REG_LR,stop|1)
    cpu.emu_start(symbols[name]|1,stop,count=1000000)
    assert cpu.reg_read(UC_ARM_REG_PC)==stop,name


call('APP_Task_Message_Init')
for _ in range(3000):call('APP_Task_Message_Step')
call('APP_Current_Report')
assert reads==3000 and initializations==1 and now==60000
line=next(line for line in output if line.startswith('CURRENT '))
fields=dict(re.findall(r'(\w+)=(\S+)',line))
assert fields['samples']=='3000' and fields['age_ms']=='20' and fields['valid']=='1'
assert fields['current_a']=='0.000' and fields['errors']=='0'
print(json.dumps({'scope':'actual ARM ELF task entries; simulated ADC zero and OS time; no hardware',
                  'elf':sys.argv[1],'background_steps':0,'elapsed_ms':now,'adc_reads':reads,
                  'adc_initializations':initializations,'report':line},indent=2))
