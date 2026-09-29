"""ARM STATUS 那一行的最坏长度必须装得进发送缓冲（连同 \\r\\n）。

为什么单独一条测试：`APP_Control_QueueText` 用 vsnprintf 写进
APP_UART_TX_TEXT_SIZE（256）字节的缓冲，超长时**静默截断**。这一行的最后
两样恰好是 `battery_ok=` 和 `\\r\\n`：一截断，电池条件没了，上位机也等不到
行尾。常态下这行只有 200 出头，只有几样"最长"同时出现才会越界——
block=frame_migration（坐标迁移没做完）+ airframe_missing 报最长的那个字段名——
台架上几乎撞不到，所以只能算出来钉住。2026-09-27 之前的格式（各项按 uint8 原样
打印、行尾前还带十位数的 t_ms）最坏 292 字符，正是这样静默截断的。

算法：从真实源码里取出格式串和每个转换对应的实参表达式，逐个换成它**按类型/
按取值集合能取到的最长值**，再用同一个格式串拼出整行量长度。
  * `%s`：block 取 arm_block_name 的全部返回值；airframe_missing 取
    DRV_Airframe_FirstInvalidName 能返回的全部名字（必填表 + 倾转转轴的
    :near/:sign 复合名）。
  * `%u`：经 arm_flag() 归一的只可能是 0/1；直接转型的 uint8_t（blinks）按 255 算。
  * `%lu`：uint32 的 now_ms 按 4294967295 算（现行格式已不带它）。
认不出上界的实参直接判失败——宁可让加字段的人来这里补一行，也不要悄悄放过。
"""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ARM_CMD = ROOT / "App" / "Src" / "app_cmd_arm.c"
AIRFRAME_SRC = ROOT / "Driver" / "Src" / "drv_airframe_params.c"
MESSAGES_H = ROOT / "App" / "Inc" / "app_messages.h"
STABILIZER_H = ROOT / "App" / "Inc" / "app_stabilizer.h"
AIRFRAME_H = ROOT / "Driver" / "Inc" / "drv_airframe_params.h"

C_ESCAPES = {"\\r": "\r", "\\n": "\n", '\\"': '"', "\\\\": "\\"}


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    open_brace = source.index("{", start)
    depth = 0
    for index in range(open_brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[open_brace + 1:index]
    raise AssertionError(f"unterminated body: {signature}")


def _split_call_arguments(text: str) -> list[str]:
    """把 `f(a, b(c, d), "x,y")` 的实参按顶层逗号切开（跳过字符串与括号）。"""
    args: list[str] = []
    depth = 0
    current: list[str] = []
    in_string = False
    index = 0
    while index < len(text):
        ch = text[index]
        if in_string:
            current.append(ch)
            if ch == "\\":
                current.append(text[index + 1])
                index += 2
                continue
            if ch == '"':
                in_string = False
        elif ch == '"':
            in_string = True
            current.append(ch)
        elif ch == "(":
            depth += 1
            current.append(ch)
        elif ch == ")":
            if depth == 0:
                args.append("".join(current).strip())
                return args
            depth -= 1
            current.append(ch)
        elif ch == "," and depth == 0:
            args.append("".join(current).strip())
            current = []
        else:
            current.append(ch)
        index += 1
    raise AssertionError("unterminated call")


def _c_string_literal_concat(expression: str) -> str:
    pieces = re.findall(r'"((?:[^"\\]|\\.)*)"', expression)
    assert pieces, expression
    text = "".join(pieces)
    return re.sub(r"\\[rn\"\\]", lambda m: C_ESCAPES[m.group(0)], text)


def _status_call() -> tuple[str, list[str]]:
    body = _function_body(_read(ARM_CMD), "void app_control_report_arm(void)")
    call = body.split("APP_Control_QueueText(", 1)[1]
    args = _split_call_arguments(call)
    return _c_string_literal_concat(args[0]), args[1:]


def _block_names() -> list[str]:
    body = _function_body(_read(ARM_CMD), "static const char *arm_block_name(")
    return re.findall(r'return "([^"]+)";', body)


def _airframe_invalid_names() -> list[str]:
    """DRV_Airframe_FirstInvalidName 能返回的全部名字。"""
    source = _read(AIRFRAME_SRC)
    table = source.split("airframe_check_nonzero[] = {", 1)[1].split("};", 1)[0]
    names = re.findall(r'"(airframe\.[a-z0-9_]+)"', table)
    body = _function_body(source, "const char *DRV_Airframe_FirstInvalidName(void)")
    names += re.findall(r'"(airframe\.[a-z0-9_]+:[a-z]+)"', body)
    assert len(names) >= 10, names
    return names


def _uint8_status_fields() -> set[str]:
    header = _read(STABILIZER_H)
    block = header.split("} APP_Stabilizer_ArmStatus;", 1)[0]
    block = block.rsplit("typedef struct {", 1)[1]
    return set(re.findall(r"uint8_t\s+([a-z_0-9]+)\s*;", block))


def _worst_value(conversion: str, argument: str):
    if conversion == "s":
        literals = re.findall(r'"([^"]*)"', argument)
        if "arm_block_name(" in argument:
            return max(_block_names() + literals, key=len)
        if re.search(r"\bmissing\b", argument):
            return max(_airframe_invalid_names() + literals, key=len)
        raise AssertionError(f"%s argument without a known value set: {argument}")
    if conversion == "u":
        if re.fullmatch(r"arm_flag\(status\.[a-z_0-9]+\)", argument):
            return 1
        cast = re.fullmatch(r"\(unsigned int\)status\.([a-z_0-9]+)", argument)
        if cast and cast.group(1) in _uint8_status_fields():
            return 255
        raise AssertionError(f"%u argument without a known bound: {argument}")
    if conversion == "lu":
        if argument == "(unsigned long)status.now_ms":
            return 4294967295
        raise AssertionError(f"%lu argument without a known bound: {argument}")
    raise AssertionError(f"unexpected conversion %{conversion}")


def _worst_case_line() -> str:
    fmt, args = _status_call()
    conversions = re.findall(r"%(lu|u|s)", fmt)
    assert len(re.findall(r"%", fmt)) == len(conversions), fmt
    assert len(conversions) == len(args), (conversions, args)
    values = tuple(_worst_value(c, a) for c, a in zip(conversions, args))
    return fmt % values


def test_arm_flag_really_normalises_to_zero_or_one() -> None:
    """`%u` 按 1 位计的前提：arm_flag 只可能返回 0 或 1。"""
    body = _function_body(_read(ARM_CMD), "static unsigned int arm_flag(")
    assert re.sub(r"\s+", " ", body).strip() == "return (value != 0U) ? 1U : 0U;"


def test_worst_case_status_line_fits_the_tx_buffer_with_crlf() -> None:
    size = int(re.search(r"#define APP_UART_TX_TEXT_SIZE (\d+)U", _read(MESSAGES_H)).group(1))
    line = _worst_case_line()

    # vsnprintf 需要 1 字节放 '\0'；返回值 >= size 即截断。
    assert len(line) <= size - 1, (len(line), line)
    assert line.endswith(" battery_ok=1\r\n"), line
    # 最坏情形确实用上了最长的原因名与字段名——否则上面的判据是在量一个假的最坏值。
    assert "block=frame_migration " in line
    assert "airframe_missing=airframe.thrust_point_to_cg_z_m " in line
    assert " blinks=255 " in line


def test_every_invalid_name_fits_on_its_own() -> None:
    """逐个名字代入，而不是只信"最长那个"：原因词改长时这里会点名是哪一个。"""
    size = int(re.search(r"#define APP_UART_TX_TEXT_SIZE (\d+)U", _read(MESSAGES_H)).group(1))
    fmt, args = _status_call()
    conversions = re.findall(r"%(lu|u|s)", fmt)
    for name in _airframe_invalid_names():
        values = []
        for conversion, argument in zip(conversions, args):
            if conversion == "s" and re.search(r"\bmissing\b", argument):
                values.append(name)
            else:
                values.append(_worst_value(conversion, argument))
        line = fmt % tuple(values)
        assert len(line) <= size - 1, (name, len(line))


def _joined_comment(text: str) -> str:
    """把多行块注释拼成一句（去掉行首的 ` * `），好按整句匹配数字。"""
    return re.sub(r"\s*\n\s*\*\s?", "", text)


def test_comments_quote_the_computed_worst_case() -> None:
    """注释里写的最坏长度/余量必须等于算出来的——两处注释曾经各说各的（255 对 250）。

    加字段的人先读注释判断还剩多少余量；注释说错了，要么以为没余量不敢加，
    要么以为还有余量直接越界。
    """
    size = int(re.search(r"#define APP_UART_TX_TEXT_SIZE (\d+)U", _read(MESSAGES_H)).group(1))
    worst = len(_worst_case_line())
    margin = (size - 1) - worst

    body = _function_body(_read(ARM_CMD), "void app_control_report_arm(void)")
    budget = next(c for c in re.findall(r"/\*(.*?)\*/", body, re.S) if "行预算" in c)
    stated = [int(n) for n in re.findall(r"(\d+) 字符。", _joined_comment(budget))]
    assert stated == [worst], (stated, worst)
    longest = max(_airframe_invalid_names(), key=len)
    named = re.search(r"（(airframe\.[a-z0-9_:]+)，(\d+) 字符）", _joined_comment(budget))
    assert named is not None, budget
    assert (named.group(1), int(named.group(2))) == (longest, len(longest))

    header = _joined_comment(_read(AIRFRAME_H))
    found = re.search(r"最坏整行\D{0,8}?(\d+) 字符、余量 (\d+)", header)
    assert found is not None, "drv_airframe_params.h 的行预算注释要写出最坏长度与余量"
    assert (int(found.group(1)), int(found.group(2))) == (worst, margin)
