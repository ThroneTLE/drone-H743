from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_baro_ok_requires_real_spl06_who_am_i_not_only_spi_success() -> None:
    control = read("App/Src/app_cmd_diag.c")

    assert "status->product_id == BSP_SPL06_ID_VALUE" in control
    assert "status->split_id == BSP_SPL06_ID_VALUE" in control
    assert "status->txrx_id == BSP_SPL06_ID_VALUE" in control
    assert 'return "who_id";' in control
    assert "exp=0x10" in control
