from tools.sim_xz.clocking import SimulationClock


class Engine:
    dt_s = .001
    running = True
    pause_reason = ''
    def __init__(self): self.steps = 0
    def step(self): self.steps += 1


def test_windows_coarse_timer_preserves_simulation_speed():
    engine = Engine()
    clock = SimulationClock(engine, 0)
    # A 15.625 ms polling interval must advance 1 s, not just 64 ms.
    for tick in range(1, 65): clock.advance(tick/64, 1)
    assert engine.steps == 1000


def test_slow_motion_and_pause_do_not_change_integration_step():
    engine = Engine()
    clock = SimulationClock(engine, 0)
    for tick in range(1, 65): clock.advance(tick/64, .25)
    assert engine.steps == 250
    engine.running = False
    clock.advance(1.5, 1)
    engine.running = True
    clock.advance(1.516, 1)
    assert engine.steps == 266


def test_resume_from_long_os_suspend_pauses_without_large_catchup():
    engine = Engine()
    clock = SimulationClock(engine, 0)
    clock.advance(30, 1)
    assert not engine.running and engine.steps == 0
