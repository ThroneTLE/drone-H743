"""Real C six-decimal replies must confirm equivalent user numeric text."""
from pathlib import Path

import pytest

from tools.panel_lib.parameter_editor import ParameterEditorMixin
from tools.sim_xz.controller_bridge import ControllerBridge


class EditorProbe(ParameterEditorMixin):
    def __init__(self):
        self.param_states = {}

    def _render_param_state(self, state):
        pass

    def _sync_pid_quick_var(self, name, value):
        pass

    def _parameter_set_status(self, *args, **kwargs):
        pass


@pytest.mark.parametrize("requested_text", ("0.2", "2e-1", "+0.2000"))
def test_equivalent_real_c_reply_clears_pending(requested_text):
    source = (Path(__file__).resolve().parents[1] / "App/Src/app_control.c").read_text(encoding="utf-8")
    assert '"%s%d.%06u"' in source  # Actual firmware wire precision.
    bridge = ControllerBridge(instance_tag="numeric_echo")
    bridge.reset_params()
    name = "coax.rate_pitch_ki"
    assert bridge.set_param(name, float(requested_text))
    reply = f"{bridge.get_param(name):.6f}"
    editor = EditorProbe()
    editor._set_param(name, requested_text, "local", True)
    editor._mark_param_pending(name, requested_text)
    editor._set_param(name, reply, "PARAM", False)
    state = editor.param_states[name]
    assert state.pending is None
    assert state.draft is None
    assert state.target == reply


@pytest.mark.parametrize("later_draft", ("0.3", "0.1"))
def test_old_reply_does_not_confirm_new_target_or_overwrite_new_draft(later_draft):
    editor = EditorProbe()
    name = "coax.rate_pitch_ki"
    editor._set_param(name, "0.2", "local", True)
    editor._mark_param_pending(name, "0.2")
    editor._set_param(name, later_draft, "local", True)
    editor._set_param(name, "0.100000", "PARAM", False)
    assert editor.param_states[name].pending == "0.2"
    assert editor.param_states[name].draft == later_draft
    editor._set_param(name, "0.200000", "PARAM", False)
    assert editor.param_states[name].pending is None
    assert editor.param_states[name].draft == later_draft

