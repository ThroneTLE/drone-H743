import math
import pytest
from tools.sim_xz.actuators import TiltActuator
from tools.sim_xz.controller_bridge import ControllerBridge, ControllerOutput
from tools.sim_xz.physics import XZPlant


def test_delay_and_static_gain_match_fopdt_response():
    actuator=TiltActuator(gain=.8,delay=.04,tau_increase=.07,tau_decrease=.1)
    actual=0.
    for step in range(10):
        _,actual=actuator.step(actual,.1,step*.01,.01)
        if step<4: assert actual==0
    assert actual==pytest.approx(.08*(1-math.exp(-.06/.07)),abs=1e-12)


def test_directional_response_and_reset_discard_pending_commands():
    actuator=TiltActuator(delay=0,tau_increase=.05,tau_decrease=.2)
    _,up=actuator.step(0,.1,0,.01)
    actuator.reset()
    _,down=actuator.step(0,-.1,0,.01)
    assert up>abs(down)
    actuator.delay=.2
    actuator.step(0,.5,.1,.01)
    actuator.reset()
    assert not actuator.pending


def test_runtime_model_uses_pitch_servo_fit_and_physical_thrust_limit():
    bridge=ControllerBridge(instance_tag='actuator_model')
    plant=XZPlant.from_bridge(bridge)
    model=bridge.physical_model()
    assert plant.tilt_actuator.gain==model['tilt_gain']
    assert plant.tilt_actuator.delay==model['tilt_delay_s']
    assert plant.tilt_actuator.tau_decrease==model['tilt_tau_decrease_s']
    assert plant.pitch_effectiveness==model['pitch_effectiveness']
    assert 0 < plant.pitch_effectiveness < 1
    plant.reset(thrust_n=plant.hover_thrust_n)
    command=ControllerOutput(100,100,0,0,(0,0,0),(0,0,0),(0,0,0),(0,0,0))
    for _ in range(1000): plant.step(command,.001)
    assert plant.state.thrust_n <= model['max_total_thrust_n']+1e-6
    assert plant.state.thrust_upper_n + plant.state.thrust_lower_n == pytest.approx(plant.state.thrust_n)
