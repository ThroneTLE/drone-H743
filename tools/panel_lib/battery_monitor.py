"""Strict battery diagnostic v1 decoder; integer wire units, no Tk or hardware knowledge."""
from dataclasses import dataclass
import struct

FRAME=struct.Struct('<BBBB14I')


@dataclass(frozen=True)
class BatterySnapshot:
    flags: int
    cells: int
    adc_status: int
    nonce: int
    age_ms: int
    voltage_mv: int
    raw: int
    samples: int
    errors: int
    low_mv: int
    recover_mv: int
    current_ma: int | None
    current_age_ms: int
    tx_accepted: int
    tx_rejected: int
    config_generation: int
    arm_state: int

    @property
    def valid(self):return bool(self.flags&1)
    @property
    def low(self):return bool(self.flags&2)
    @property
    def can_arm(self):return bool(self.flags&4)


def decode_battery(payload):
    if len(payload)!=FRAME.size:raise ValueError('电池回包长度错误')
    version,flags,cells,adc,*values=FRAME.unpack(payload)
    if version!=1 or flags&128 or not 1<=cells<=12 or adc>3:raise ValueError('电池回包版本或标志错误')
    if flags&32 and flags&64:raise ValueError('配置确认标志冲突')
    nonce,age,mv,raw,n,errors,low,recover,current,current_age,tx,drop,generation,arm=values
    if raw>65535 or mv>200000 or not 2500<=low<=4100 or not low<recover<=4400 or arm>2:
        raise ValueError('电池回包字段超界')
    if flags&1 and (age>250 or not n or adc or flags&8):raise ValueError('电压有效性矛盾')
    if flags&4 and (not flags&1 or flags&2):raise ValueError('电池解锁判据矛盾')
    if current>=0x80000000:current-=0x100000000
    if flags&16:
        if current==-2147483648 or current_age>250:raise ValueError('电流有效性矛盾')
    else:current=None
    return BatterySnapshot(flags,cells,adc,nonce,age,mv,raw,n,errors,low,recover,current,
                           current_age,tx,drop,generation,arm)
