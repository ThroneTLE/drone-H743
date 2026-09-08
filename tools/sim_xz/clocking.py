"""Accumulate wall time without assuming sub-millisecond OS timer precision."""
import math


class SimulationClock:
    def __init__(self, engine, now):
        self.engine = engine
        self.last = now
        self.credit = 0.0

    def advance(self, now, scale):
        elapsed = now-self.last
        self.last = now
        if not math.isfinite(elapsed) or elapsed < 0 or elapsed > 1:
            self.engine.running = False
            self.engine.pause_reason = '系统时钟停顿，仿真已暂停，请复位'
            self.credit = 0.0
            return
        if not self.engine.running:
            self.credit = 0.0
            return
        self.credit += elapsed*scale
        dt = self.engine.dt_s
        while self.credit + 1e-12 >= dt and self.engine.running:
            self.engine.step()
            self.credit -= dt
