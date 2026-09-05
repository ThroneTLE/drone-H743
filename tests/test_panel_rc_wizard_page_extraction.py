"""S6 RC page extraction contract."""

from __future__ import annotations

import ast
import hashlib
import os
from pathlib import Path
import subprocess
import sys

from tools import drone_tcp_panel as legacy_panel
from tools.panel_lib.pages import mechanical as mechanical_page
from tools.panel_lib.pages import rc_wizard as rc_page


ROOT = Path(__file__).resolve().parents[1]
LEGACY_PANEL_PATH = ROOT / "tools" / "drone_tcp_panel.py"
MECHANICAL_PAGE_PATH = ROOT / "tools" / "panel_lib" / "pages" / "mechanical.py"
FLOW_PAGE_PATH = ROOT / "tools" / "panel_lib" / "pages" / "flow_ranging.py"
RC_PAGE_PATH = ROOT / "tools" / "panel_lib" / "pages" / "rc_wizard.py"

METHOD_AST_SHA256 = {
    "_build_rc_page": "688346a18b5466c90a90c83d71d79d645a89e5c975b46ec79514234545c9b3f2",
    "_rc_row_entry": "4ee254afe89559f7af7b9f520d7f80a5bd1a4a73e8ca4fefd316417c0db70e75",
    "_rc_collect_map": "636036f6c51260ac6791a3ee94d8c2cf803ca85e8589ac72c16cfd2e1826a79e",
    "_rc_set_row": "8c3b9319326db09eb172f6936df5e59cfe0b0f9688c8d7ff3cc08c09d8150c74",
    "_rc_deadband": "c9a83d1e572198f1b1905db7e1ce57f30496537deda8d112652ce1ee27a177dc",
    "_rc_handle_live_line": "48f51b3cb1ae68c695a43b1b856380620e11f0975256a27524f77afd3a0a59ea",
    "_rc_handle_map_line": "6fb804b34e5c6fb178718f004da4a54c0bde5e0ef9e867253aa62eeb89ffb94e",
    "_rc_state_text": "586b2e9c6b3dcb948fb10b7fb0be393aac64f2a0aafc82520fac5225c9bd30c2",
    "_rc_accumulate": "47cc9a67daf9e48ae5238ba9927794eeb9a98bee6f57f4166390cd07d74aa16b",
    "_rc_wizard_start": "09db4b04c1f3317cfec6b1c7c25475295c6b8952ee2cb4413835f4ad5cdba909",
    "_rc_wizard_cancel": "9f925447b0c0dc7df6ef8f2e36b80d0e39fd41feffef889bfa0fb7df5b6d2a41",
    "_rc_wizard_skip": "17956d150bc908df50a2b3be2a4de84636f6e0fc30d8850edb994a143f9eb535",
    "_rc_wizard_back": "fbd47930f8822ab5fbc6959c90731a8344f65c4fb1ca90e6b7d92caff75b4126",
    "_rc_wizard_advance": "5f85d966e2ab6fdfb8aa70f30033c1657e3c951f8e0fd0cd223646cb46dab5d9",
    "_rc_wizard_reference": "cfcff824bb2fdf11156bd139ab9a20f056e5f44e44384916d4fe8913f1d7c939",
    "_rc_wizard_trace": "e76be4f0db345b198fb34798448fece69ccd7bd5525f438b686cddffe899bfec",
    "_rc_wizard_feed": "9984944b62b1005b8a1ab83a61d6afb92d44c0107057c2ba4bba99fdde64ef5e",
    "_rc_wizard_finish": "4ec916b93bccf6fe2d326ba238c2dc343b7e101879128a10205659e02172fdee",
    "_rc_wizard_refresh": "a70ea685f194d8f6e7960190d048ce9ab7d42c95d5280af205ffbd64b033f9c0",
    "_rc_start_detect": "8ade73dc84bda553b7b02ee94411b32a528a01512be47d1be09b0481a636c6b6",
    "_rc_finish_detect": "45133bd52accdbcc4430289b207aaca93a1631345a7182354f90f2bea384cf9b",
    "_rc_capture_center": "3294f7c9ff1bee58acdb9f77781616d9a88e9ec5ee8e40ca498a9aefb68bbe1d",
    "_rc_toggle_sweep": "c689f093891d9fb1c67c2256f6bf0473c3ffe95a60f5d45527df41ed3a225fba",
    "_rc_apply_sweep": "2da0db4385c8bc63d7f6a434103592d5f67a5b38e40091cab051faf9ce5c872f",
    "_rc_live_ok": "89ab286b6df7e6c39c322b88d701dbcde56357afd9a80e5fdfc26d2c7d37fe06",
    "_rc_request_map": "93a23ba25b9010636fbc842dc33d328acfbfed9cc776a76515a5596572504c80",
    "_rc_send_map": "e34f5674ee207b768e2cbd197d90ee6c71c081fbfa9fabbbd12ce6e1794a531f",
    "_rc_apply_ram": "91dcbfccafb1830f7a5d53f0efce249ed5510efdcb20ea1c5ed4b95852d5c4c7",
    "_rc_commit": "2b95790de8a54c10d0285ffb740b31bb2dca3c0605487097ea4663bd61097058",
    "_rc_reset_defaults": "a74fbdfd02711c20c0a9f4096ff837e7e09ba028d7558c8a58cd5c5496006e5f",
    "_rc_refresh_controls": "b0692115e190bac92ed12511d4d46147a27272f6d334fa8a88e2bb8ca900d982",
    "_rc_render": "0cde3d45ca3da6ac6c5f94f974aeed200f057523dbff5cc1564e8d3616405af5",
    "_rc_draw_sticks": "19cb76e377271868efed5fa5c2437cd11c25fb9af054626ff3a62c1bf0c36f4f",
    "_rc_axis_value": "b5bf68124a476f0c1d9c86d98c642d33c5fbdbdcf3f97fbc4746ef31b1e49a4d",
    "_rc_throttle_value": "1140e9b78ef07b29277290125918886ccbce9f7b328aa68d9acd8f641bfa22c1",
}
HELPER_AST_SHA256 = {
    "rc_channel_travel": "68eb48e5f2b09a592f647e4ecd724965b9ff4d27bb80ad8fa3a4e3db83dd44b0",
    "rc_detect_channel": "dbe6e78918547798cc19c4c1759e85dccdd7836f15f295e17b56654ad9cf3a52",
    "rc_map_is_valid": "c990773ca262c36b8792e7f00e36dbf1e96c1d70fa1721fe19c0ef154caa7364",
    "rc_wizard_dominant": "7ad17afac17f871a530822473b7849b3a7803ff7bc5bc60710c913ebdf73a88a",
    "rc_wizard_step_ready": "465efa4bbef25d7f8beed87638fb4db7ebeefe17130855b43624d0c11102b619",
    "rc_wizard_window_stable": "e7dd057ac1bc4c35977669641908fa2efd467443e371791e95cef27278a54d75",
    "rc_wizard_gate_open": "3be17d27fa504289ccfd92e6c659a46dcd01c410a85bdb214614d10b34711f61",
    "rc_wizard_build_map": "14d618f1ff92b8dfd2fcbe6947f35c1ee093bddad0d8e445e68444f5a191a995",
    "rc_normalize": "3787e126d444ff0bb6658bd943aeea668764fb4fd9eaa08682209434b7b183b4",
}
OWNED_CONSTANTS = {
    "RC_CHANNEL_COUNT",
    "RC_DETECT_DOMINANCE",
    "RC_DETECT_MIN_TRAVEL_US",
    "RC_FUNCTIONS",
    "RC_LIVE_FRESH_S",
    "RC_MIN_SPAN_US",
    "RC_US_MAX",
    "RC_US_MIN",
    "RC_WIZARD_CENTER_MARGIN",
    "RC_WIZARD_DOMINANCE",
    "RC_WIZARD_HOLD_FRAMES",
    "RC_WIZARD_HOLD_TOLERANCE_US",
    "RC_WIZARD_MIN_DEVIATION_US",
    "RC_WIZARD_NON_CENTERING",
    "RC_WIZARD_STEPS",
}
UNTOUCHED_PAGE_AST_SHA256 = {
    # Effective parent 27149729 changed only this builder's label style after
    # the requested 8b8085c6 baseline; the RC hashes above remain pinned to 8b.
    "_build_mechanical_calibration_page": "a361b5ff4e5dc1f5dc69cd3a7877e795abdca1ec4d6c8d029700902d7f2e5154",
    "_mechanical_row_values": "91112bfc4ed0c275a94a5bab26e39ff0caf68b8e3377991765cd5f9cdeb2ab82",
    "_mechanical_local_target": "9f151f75ecdee6a72f283491518fb793abbaee323a1616a1b84032e16d7bd156",
    "_mechanical_target_matches_local": "7a959f7bd8f9fcf114d521e56f1369342bad5a2e2f1557053f8c55f421e99427",
    "_mechanical_read_target": "e5f4c3d944d72bc3aac93eb6e70e278b95a9560412ee2dac47350c2f0ea59cbf",
    "_mechanical_apply_target": "bc9b9b3f31e451919c580c89e44b089a7bbd2d2e41aec5fe5d079fe8aa9bfef6",
    "_mechanical_revert_target": "0536ff47b7aacfd7de2938baf6b8e2d4d4c75b76dc9cd801da1a93a878967e26",
    "_mechanical_commit_target": "dc5c31afddf057cdd754628527d6ae4825bab0dd12348e0e9be1bbd0daa29e4c",
    "_mechanical_handle_target_line": "d97d2de7c02932f7163bad3a37caeb4696d0b2a2eacf82427ba71aa24aaa5230",
    "_mechanical_move": "5d375849112026204ab10ebff4eaf4a7d99ff1f287bab24a48e00e0c78d48183",
    "_mechanical_nudge_center": "44b1f0da89a2b39e291c8520f994d197fccd99760d25a700469bd25d821633bf",
    "_mechanical_jog_stop": "bd1fc4be0f2a5c43eb1968f7929e4de85ee93fb1183307f4fe80231926295170",
    "_mechanical_save_evidence": "090a8861825e49b5b1c3440138e9260fb09d8b80e093eecaa07852939574e9f2",
    "_build_flow_range_calibration_page": "b82cfc6332ac43ec89dcea660fc61a76080e7bd4fe203c1ad7a9d18fcbd7b448",
    "_flow_range_request_once": "2ff7e3a41f240d644b4e729c14f68ef8f563784a0157d7f5c9908cb0c61c3428",
    "_flow_cal_start": "59f98967d8d888edbfb924652470a982f9428b45711c45c93ecc8329fbe30147",
    "_flow_cal_stop": "7f0514ad7a8b81f308c642468e4cfe6f8191b833ea4779a5aa0e292104355d9a",
    "_flow_cal_analyze_stage": "49fd8a48b303597d37b42694f58ddd861e1232b1af6ef6708676bd0357f1db69",
    "_flow_cal_result_summary": "7af89dde1f64b009dc2bba0988b26211734ed9e533ba1b1832e2bb639fe84f72",
    "_flow_cal_refresh_tree": "08aa97e663548e84313924767a17b976c148e61974fce1e58f5225a5f268e747",
    "_flow_cal_save_report": "5796e238f117707a1272c649853af1973dacaa8a9fdfd45d2da775c5432054f0",
}


def parsed(path: Path) -> ast.Module:
    return ast.parse(path.read_text(encoding="utf-8"))


def class_methods(path: Path, class_name: str) -> dict[str, ast.FunctionDef]:
    owner = next(
        node
        for node in parsed(path).body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    return {
        node.name: node
        for node in owner.body
        if isinstance(node, ast.FunctionDef)
    }


def top_level_functions(path: Path) -> dict[str, ast.FunctionDef]:
    return {
        node.name: node
        for node in parsed(path).body
        if isinstance(node, ast.FunctionDef)
    }


def top_level_assignments(path: Path) -> set[str]:
    names: set[str] = set()
    for node in parsed(path).body:
        if isinstance(node, (ast.Assign, ast.AnnAssign)) and isinstance(
            node.value, ast.Attribute
        ):
            continue
        targets = node.targets if isinstance(node, ast.Assign) else (
            [node.target] if isinstance(node, ast.AnnAssign) else []
        )
        names.update(target.id for target in targets if isinstance(target, ast.Name))
    return names


def class_assignments(path: Path, class_name: str) -> set[str]:
    owner = next(
        node
        for node in parsed(path).body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    names: set[str] = set()
    for node in owner.body:
        if isinstance(node, ast.Assign):
            names.update(
                target.id for target in node.targets if isinstance(target, ast.Name)
            )
    return names


def ast_sha256(node: ast.AST) -> str:
    payload = ast.dump(node, include_attributes=False).encode()
    return hashlib.sha256(payload).hexdigest()


def test_method_ownership() -> None:
    owned = class_methods(RC_PAGE_PATH, "RcWizardPageMixin")
    legacy = class_methods(LEGACY_PANEL_PATH, "DronePanel")

    assert set(owned) == set(METHOD_AST_SHA256)
    assert set(METHOD_AST_SHA256).isdisjoint(legacy)
    assert class_assignments(RC_PAGE_PATH, "RcWizardPageMixin") == {"_RC_STATE_TEXT"}
    assert "_RC_STATE_TEXT" not in class_assignments(LEGACY_PANEL_PATH, "DronePanel")


def test_method_ast() -> None:
    owned = class_methods(RC_PAGE_PATH, "RcWizardPageMixin")

    assert {
        name: ast_sha256(owned[name]) for name in METHOD_AST_SHA256
    } == METHOD_AST_SHA256
    assert len(owned["_build_rc_page"].body) == 72


def test_helper_ownership() -> None:
    owned_functions = top_level_functions(RC_PAGE_PATH)
    legacy_functions = top_level_functions(LEGACY_PANEL_PATH)

    assert {
        name: ast_sha256(owned_functions[name]) for name in HELPER_AST_SHA256
    } == HELPER_AST_SHA256
    assert set(HELPER_AST_SHA256).isdisjoint(legacy_functions)
    assert OWNED_CONSTANTS <= top_level_assignments(RC_PAGE_PATH)
    assert OWNED_CONSTANTS.isdisjoint(top_level_assignments(LEGACY_PANEL_PATH))


def test_forwarding() -> None:
    assert legacy_panel.RcWizardPageMixin is rc_page.RcWizardPageMixin
    for name in METHOD_AST_SHA256:
        assert getattr(legacy_panel.DronePanel, name) is getattr(
            rc_page.RcWizardPageMixin, name
        )
    for name in rc_page.__all__:
        assert getattr(legacy_panel, name) is getattr(rc_page, name), name


def test_protected_pages() -> None:
    legacy = class_methods(LEGACY_PANEL_PATH, "DronePanel")
    mechanical = class_methods(MECHANICAL_PAGE_PATH, "MechanicalPageMixin")
    # 增量11 之后光流页归 flow_ranging.py；哈希不变，只是换了查找位置。
    flow = class_methods(FLOW_PAGE_PATH, "FlowRangingPageMixin")
    owners = {**legacy, **mechanical, **flow}

    assert {
        name: ast_sha256(owners[name]) for name in UNTOUCHED_PAGE_AST_SHA256
    } == UNTOUCHED_PAGE_AST_SHA256
    assert legacy_panel.MechanicalPageMixin is mechanical_page.MechanicalPageMixin


def test_palette() -> None:
    assert all(
        rc_page.UI_PALETTE[name] == legacy_panel.UI_PALETTE[name]
        for name in rc_page.UI_PALETTE
    )


def test_direct_import() -> None:
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import drone_tcp_panel as panel; "
                "import panel_lib.pages.rc_wizard as owner; "
                "assert panel.RcWizardPageMixin is owner.RcWizardPageMixin; "
                "assert panel.DronePanel._rc_commit is owner.RcWizardPageMixin._rc_commit; "
                "assert panel.rc_map_is_valid is owner.rc_map_is_valid"
            ),
        ],
        cwd=ROOT / "tools",
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=20,
    )
    assert result.returncode == 0, result.stdout
