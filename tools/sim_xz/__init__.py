"""Public simulator API; metadata imports must not initialize its runtime.

The panel also runs as tools/drone_tcp_panel.py, where only tools/ is on
sys.path. Reading sim_xz.control_catalog there must not import a protocol
module that belongs to the separately launched tools.sim_xz process.
"""
from importlib import import_module

__all__ = [
    "ControllerBridge", "ControllerOutput", "ExperimentKind", "SimulationEngine",
    "SimulationState", "SimulatorProtocol", "XZPlant", "run_ab", "run_experiment",
]

_EXPORTS = {
    "ControllerBridge": "controller_bridge",
    "ControllerOutput": "controller_bridge",
    "ExperimentKind": "experiments",
    "SimulationEngine": "experiments",
    "run_ab": "experiments",
    "run_experiment": "experiments",
    "SimulationState": "physics",
    "XZPlant": "physics",
    "SimulatorProtocol": "protocol",
}


def __getattr__(name):
    module = _EXPORTS.get(name)
    if module is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    value = getattr(import_module(f".{module}", __name__), name)
    globals()[name] = value
    return value
