"""Run actual linked ARM registry providers; no peripheral address space is mapped.

Usage: PYTHONPATH=<Unicorn installation> python arm_component_probe.py <ELF> <output.bin>
ADC is injected as 12000; the registry transport is captured. This is not board evidence.
"""
import sys
import struct
import json
from pathlib import Path
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_HOOK_CODE
from unicorn.arm_const import *
from elftools.elf.elffile import ELFFile

with open(sys.argv[1], 'rb') as stream:
    elf = ELFFile(stream)
    symbols = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
    cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
    cpu.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_A15)
    for addr, size in [(0x08000000, 0x200000), (0x20000000, 0x20000),
                       (0x21000000, 0x10000), (0x24000000, 0x100000),
                       (0x30000000, 0x80000), (0x38000000, 0x10000)]:
        cpu.mem_map(addr, size)
    for segment in elf.iter_segments():
        if segment['p_type'] == 'PT_LOAD' and segment['p_filesz']:
            cpu.mem_write(segment['p_vaddr'], segment.data())
cpu.reg_write(UC_ARM_REG_C1_C0_2, 0xf << 20)
cpu.reg_write(UC_ARM_REG_FPEXC, 0x40000000)
cpu.mem_write(symbols['uwTick'], struct.pack('<I', 100))
frames = []
hooks = {symbols[name] & ~1: name for name in (
    'BSP_Current_Init', 'BSP_Current_Read', 'APP_Components_Send',
    'BSP_Critical_Enter', 'BSP_Critical_Exit')}
stop = 0x081fff00
cpu.mem_write(stop, b'\x00\xbe')


def trace(cpu, address, size, _):
    if address == stop:
        cpu.emu_stop()
        return
    name = hooks.get(address)
    if name is None:
        return
    if name == 'BSP_Current_Read':
        cpu.mem_write(cpu.reg_read(UC_ARM_REG_R0), struct.pack('<I', 12000))
    if name == 'APP_Components_Send':
        frames.append(bytes(cpu.mem_read(cpu.reg_read(UC_ARM_REG_R0), cpu.reg_read(UC_ARM_REG_R1))))
    cpu.reg_write(UC_ARM_REG_R0, int(name == 'APP_Components_Send'))
    cpu.reg_write(UC_ARM_REG_PC, cpu.reg_read(UC_ARM_REG_LR))


cpu.hook_add(UC_HOOK_CODE, trace)
for name in ('APP_Current_Init', 'APP_Current_Step', 'APP_Components_Report'):
    cpu.reg_write(UC_ARM_REG_SP, 0x2100fff0)
    cpu.reg_write(UC_ARM_REG_LR, stop | 1)
    cpu.reg_write(UC_ARM_REG_R0, 42)
    cpu.emu_start(symbols[name] | 1, stop, count=1000000)
    assert cpu.reg_read(UC_ARM_REG_PC) == stop, f'instruction budget exceeded in {name}'

sys.path.insert(0, str(Path.cwd()))
from tools.panel_lib.component_registry import RegistryTransaction
from tools.panel_lib.proto import PROTO_DIR_FROM_FC, PROTO_MSG_COMPONENTS
from tools.panel_lib.transport import build_proto_frame
transaction = RegistryTransaction(42)
rows = None
for frame in frames:
    rows = transaction.feed(frame)
assert rows is not None and len(rows) == 13
assert not any(r.model in ('GD25Q32', 'ICM42688', 'M9N') for r in rows)
current = next(r for r in rows if r.id == 5)
assert current.fields[2].value == 12000
wire = b''.join(build_proto_frame(PROTO_DIR_FROM_FC, PROTO_MSG_COMPONENTS, frame) for frame in frames)
Path(sys.argv[2]).write_bytes(wire)
print(json.dumps({'scope': 'ARM ELF providers; cached uninitialized states + injected ADC; no board',
                  'elf': sys.argv[1], 'packets': len(frames), 'wire_bytes': len(wire),
                  'rows': [dict(id=r.id, name=r.name, model=r.model, bus=r.bus, state=r.state,
                                fields=[dict(label=f.label,value=f.value) for f in r.fields]) for r in rows]},
                 ensure_ascii=False, indent=2))
