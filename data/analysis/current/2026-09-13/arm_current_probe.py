import sys,struct,json
from pathlib import Path
sys.path.insert(0,str(Path('D:/stm32hal/drone-H743/.tmp/arm-emulator')))
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import *
from elftools.elf.elffile import ELFFile
elf=ELFFile(open(sys.argv[1],'rb')); syms={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
u=Uc(UC_ARCH_ARM,UC_MODE_THUMB)
u.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_A15)
for addr,size in [(0x08000000,0x200000),(0x20000000,0x20000),(0x21000000,0x10000),(0x24000000,0x100000),(0x30000000,0x80000),(0x38000000,0x10000)]:u.mem_map(addr,size)
for seg in elf.iter_segments():
 if seg['p_type']=='PT_LOAD' and seg['p_filesz']:u.mem_write(seg['p_vaddr'],seg.data())
u.reg_write(UC_ARM_REG_C1_C0_2,0xf<<20)
u.reg_write(UC_ARM_REG_FPEXC,0x40000000)
u.mem_write(syms['uartTxQueueHandle'],struct.pack('<I',1))
u.mem_write(syms['uwTick'],struct.pack('<I',100))
output=[]
returns={syms[n]&~1:n for n in ['BSP_Current_Init','BSP_Current_Read','APP_USB_CDC_Write','osMessageQueuePut','osMessageQueueGet','APP_UART_NotifyTxPending','BSP_Critical_Enter','BSP_Critical_Exit']}
stop=0x081fff00
u.mem_write(stop,b'\x00\xbe')
def code(u,addr,size,data):
 if addr==stop:u.emu_stop();return
 name=returns.get(addr)
 if not name:return
 if name=='BSP_Current_Read':u.mem_write(u.reg_read(UC_ARM_REG_R0),struct.pack('<I',12000))
 if name=='APP_USB_CDC_Write':output.append(bytes(u.mem_read(u.reg_read(UC_ARM_REG_R0),u.reg_read(UC_ARM_REG_R1))).decode('ascii','replace'))
 u.reg_write(UC_ARM_REG_R0,0);u.reg_write(UC_ARM_REG_PC,u.reg_read(UC_ARM_REG_LR))
u.hook_add(UC_HOOK_CODE,code)
for name in ['APP_Current_Init','APP_Current_Step','APP_Current_Report']:
 u.reg_write(UC_ARM_REG_SP,0x2100fff0);u.reg_write(UC_ARM_REG_LR,stop|1)
 try:u.emu_start(syms[name]|1,stop,count=1000000)
 except Exception as e:print('FAIL',name,hex(u.reg_read(UC_ARM_REG_PC)),str(e));raise
print(json.dumps({'source':sys.argv[1],'scope':'ARM instruction emulation; ADC/transport mocked; no board operations','printf_float_linked':bool(syms.get('_printf_float',0)),'output':output},indent=2))
