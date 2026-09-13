"""Overview RC interface follows the actual MicoAir UART binding."""
from pathlib import Path
import re
ROOT=Path(__file__).resolve().parents[1]


def test_rc_metadata_matches_the_real_uart_role():
    binding=(ROOT/'BSP/Src/bsp_uart_link.c').read_text(encoding='utf-8')
    uart=re.search(r'static const BSP_UartLinkBinding rc\s*=\s*\{\s*&huart(\d+)',binding).group(1)
    catalog=(ROOT/'BSP/Src/bsp_component_catalog.c').read_text(encoding='utf-8')
    assert f'case DRV_COMPONENT_RC:return "USART{uart}";' in catalog
