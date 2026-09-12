"""Report which App/Services files still touch hardware directly.

Three signals, all measured on code with comments and string literals removed:

  * peripheral instance names (`huart1`, `SPI2`, `TIM4`, `GPIOC`, ...)
  * HAL / CubeMX header includes
  * `HAL_*` / `__HAL_*` calls

Comments naming hardware are fine -- "this board puts it on UART8" is an
explanation of where the binding lives.  A string literal naming hardware is
fine too; `"UART1 rx_bytes=%lu"` is a wire-format label, not a dependency.
Only code counts.

The same three signals are enforced as a regression guard in
tests/test_hardware_decoupling.py; this script is the interactive view.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]

INSTANCE = re.compile(
    r"\b(?:h?(?:uart|usart)\d|UART\d|USART\d|SPI\d|hspi\d|I2C\d|hi2c\d"
    r"|TIM\d+|htim\d+|SDMMC\d|hsd\d|GPIO[A-K]|EXTI\d*|ADC\d|hadc\d)\b"
)
HAL_INCLUDE = re.compile(
    r'#\s*include\s+[<"](?:main\.h|usart\.h|spi\.h|i2c\.h|tim\.h|dma\.h|gpio\.h'
    r'|sdmmc\.h|adc\.h|usbd_[^">]*|stm32h7xx[^">]*)[">]'
)
HAL_CALL = re.compile(r"\b(HAL_[A-Za-z0-9_]+|__HAL_[A-Za-z0-9_]+)\s*\(")
CMSIS = re.compile(r"\b(__disable_irq|__enable_irq|__get_PRIMASK|__set_PRIMASK"
                   r"|__DMB|__DSB|__ISB|__WFI"
                   r"|SCB|SysTick|NVIC_[A-Za-z]+)\b")


def strip_comments(source: str) -> str:
    source = re.sub(r"/\*.*?\*/", " ", source, flags=re.S)
    return re.sub(r"//[^\n]*", " ", source)


def strip_code(source: str) -> str:
    """Drop comments and string literals -- only real code should count.

    Includes must be matched *before* this runs: the header name lives in a
    string literal, so blanking literals would hide every `#include "usart.h"`.
    """
    return re.sub(r'"(?:[^"\\]|\\.)*"', '""', strip_comments(source))


def scan(layers=("App", "Services")):
    rows = []
    for layer in layers:
        for path in sorted((ROOT / layer).rglob("*.[ch]")):
            raw = path.read_text(encoding="utf-8", errors="replace")
            code = strip_code(raw)
            instances = sorted(set(INSTANCE.findall(code)))
            include = bool(HAL_INCLUDE.search(strip_comments(raw)))
            calls = sorted(set(HAL_CALL.findall(code)))
            cmsis = sorted(set(CMSIS.findall(code)))
            if instances or include or calls or cmsis:
                rel = path.relative_to(ROOT).as_posix()
                rows.append({
                    "path": rel, "instances": instances, "include": include,
                    "calls": calls, "cmsis": cmsis,
                    "weight": len(instances) * 10 + len(calls) + len(cmsis),
                })
    rows.sort(key=lambda r: (-r["weight"], r["path"]))
    return rows


def main():
    rows = scan()
    print(f"{'file':<34} {'inc':>3} {'inst':>4} {'HAL':>4} {'cmsis':>5}  detail")
    for row in rows:
        detail = ", ".join(row["instances"] + row["calls"][:5] + row["cmsis"][:3])
        if len(row["calls"]) > 5:
            detail += f" (+{len(row['calls']) - 5} HAL)"
        print(f"{row['path']:<34} {'Y' if row['include'] else '-':>3} "
              f"{len(row['instances']):>4} {len(row['calls']):>4} "
              f"{len(row['cmsis']):>5}  {detail}")
    print(f"\n{len(rows)} of App/Services files still touch hardware directly")
    return 0


if __name__ == "__main__":
    sys.exit(main())
