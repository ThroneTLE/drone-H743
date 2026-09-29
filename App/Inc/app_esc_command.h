#ifndef APP_ESC_COMMAND_H
#define APP_ESC_COMMAND_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 给电调发 DShot 特殊命令（1..47）的窗口。
 *
 * ──────────────── 为什么这件事需要一个状态机 ────────────────
 *
 * 特殊命令不是"发一帧就算数"：电调要连着收到**同一条命令若干帧**才认。中间夹一帧
 * 油门，计数就从头开始。所以它必须占住 500 Hz 提交点连续若干拍，而不能由文本任务
 * 见缝插针地调一次 Submit——那样发出去的是"命令、油门、命令、油门…"，电调一条都不认。
 *
 * ──────────────── 为什么只开放 EDT 这两条 ────────────────
 *
 * 1..47 里躺着 7/8/20/21（改电机转向）和 12（写电调 Flash）。一个"能发任意命令号"
 * 的文本入口，意味着敲错一个数字就把飞机的转向改了、还存进了电调——那不是重启能
 * 恢复的。所以 `Request()` **只认 EDT 开关这两条**，命令号是编译期常量，不从命令行取。
 * 转向设置仍不开放。唯一例外是下面单列的 `RequestMotorKv()`：作者 2026-09-23
 * 授权的固定序列，只写 AM32 的“电机 KV”这一个字节，帧序列编译期写死，
 * 命令行只能给出范围受限的 KV 值。
 *
 * ──────────────── 边界 ────────────────
 *
 * 纯状态机：不含 HAL、不含 RTOS、不做 I/O。`Step()` 跑在 500 Hz 提交点上，
 * 由 app_stabilizer.c 在真正写执行器之前询问"这一拍该发命令吗"。
 */

/* DShot 特殊命令号。只有这两条，理由见上。 */
#define APP_ESC_COMMAND_EDT_ENABLE  13U
#define APP_ESC_COMMAND_EDT_DISABLE 14U

/*
 * 连发帧数。DShot 规范要求特殊命令重复若干帧，常见实现取 6~10；这里取 10，
 * 因为多发几帧的代价只有 20 ms（500 Hz），而少发的代价是"命令悄悄没生效"，
 * 而后者在现场表现为"EDT 打开了但没有电流"——一个会让人回头去查接收侧的假象。
 */
#define APP_ESC_COMMAND_REPEATS 10U

/*
 * ──────────────── AM32 电机 KV 写入（固定序列） ────────────────
 *
 * AM32 按电调里设置的 KV 限制低转速时的最大占空比（low_rpm_throttle_limit，默认开）。
 * 出厂 KV=2220，配 KV1300 大桨时，44k eRPM 处上限只有约 52.6%，实测推力在 50% 油门
 * 后不再上升（2026-09-23 推力台实录）。微空 55A 手册要求低 KV 电机手动改这一项。
 *
 * AM32 >= 2.18 的 DShot 编程模式（Src/dshot.c，v2.18..v2.21 一致）：
 *   36 连发**恰好** 6 帧进入编程模式——第 7 帧就会被当成“位置”；
 *   下一帧 = eepromBuffer 位置，再下一帧 = 新值，再一帧 37 = 写入 RAM 缓冲；
 *   然后 12 连发 6 帧 = 存 Flash。电调要求电机停转且已解锁（收到过零油门）。
 * 期间任何一帧别的值都会被吞成位置/数值，所以整段必须连续、不能插油门帧。
 * 2.17 及更早没有编程模式：36/26/值/37 都只是不足 6 帧的命令，被忽略；
 * 12x6 保存的是未改动的设置。新 KV 在电调重新上电时才读取。
 */
#define APP_ESC_PROGRAM_ENTER          36U
#define APP_ESC_PROGRAM_ENTER_REPEATS   6U  /* 不能多：第 7 帧会成为位置 */
#define APP_ESC_PROGRAM_COMMIT         37U
#define APP_ESC_SAVE_SETTINGS          12U
#define APP_ESC_SAVE_REPEATS            6U
#define APP_ESC_EEPROM_MOTOR_KV        26U  /* AM32 v2.18..v2.21 eepromBuffer 偏移 */
#define APP_ESC_KV_MIN                300U  /* KV<300 时 AM32 会整个关掉低速限幅 */
#define APP_ESC_KV_MAX               1900U  /* 字节 47：再大就不是 1..47 的命令帧 */
#define APP_ESC_COMMAND_MAX_FRAMES     16U

typedef enum {
    APP_ESC_COMMAND_IDLE = 0U,
    APP_ESC_COMMAND_SENDING,
    APP_ESC_COMMAND_DONE,
    APP_ESC_COMMAND_ABORTED
} APP_EscCommandState;

typedef enum {
    APP_ESC_COMMAND_ABORT_NONE = 0U,
    APP_ESC_COMMAND_ABORT_INHIBIT,   /* 解锁/验收/标定/点电机抢走了执行器 */
    APP_ESC_COMMAND_ABORT_OUTPUT     /* 提交失败，命令序列已经断了 */
} APP_EscCommandAbortReason;

void APP_EscCommand_Reset(void);

/*
 * 请求发一条命令。只接受上面那两个常量，其余一律拒绝（返回 0）。
 * 已经在发的时候再请求也返回 0：一条发到一半被另一条顶掉，两条都不会生效。
 */
uint8_t APP_EscCommand_Request(uint16_t command);

/*
 * AM32 存储的 KV 字节：KV = 字节x40 + 20。只接受 300..1900 且落在该网格上的值，
 * 其余返回 0（不四舍五入：写进电调的必须就是请求的那个数）。
 */
uint8_t APP_EscCommand_MotorKvByte(uint16_t kv, uint8_t *out_byte);

/*
 * 排队写电调 KV 的固定 15 帧序列：36x6、26、KV 字节、37、12x6。
 * KV 非法或正在发送时返回 0。执行、抢占、失败语义与 `Request()` 相同。
 */
uint8_t APP_EscCommand_RequestMotorKv(uint16_t kv);

/* 当前（或最近一次）请求计划发送的帧数。 */
uint8_t APP_EscCommand_PlannedFrames(void);

/*
 * 500 Hz 节拍。`inhibit` 非零表示有更高优先级的东西正在用执行器，此时立即中止——
 * 命令帧会占住输出，而"正在解锁飞行"永远比"给电调发配置"重要。
 */
void APP_EscCommand_Step(uint8_t inhibit);

/*
 * 这一拍要不要发命令帧。返回非零时 `out_command` 是该发的命令号。
 * 调用方发完必须调 `Consume()` 或 `Fail()`，否则计数不前进。
 */
uint8_t APP_EscCommand_Pending(uint16_t *out_command);

/* 这一帧发出去了。 */
void APP_EscCommand_Consume(void);

/* 这一帧没发出去。序列断了就整条作废，不做"补发"——补发出去的是不连续的帧。 */
void APP_EscCommand_Fail(void);

uint8_t APP_EscCommand_IsActive(void);
APP_EscCommandState APP_EscCommand_GetState(void);
uint8_t APP_EscCommand_LastAbortReason(void);
uint16_t APP_EscCommand_LastCommand(void);
uint32_t APP_EscCommand_SentFrames(void);

const char *APP_EscCommand_StateName(APP_EscCommandState state);
const char *APP_EscCommand_AbortReasonName(uint8_t reason);

#ifdef __cplusplus
}
#endif

#endif /* APP_ESC_COMMAND_H */
