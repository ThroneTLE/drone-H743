#ifndef APP_ACTION_H
#define APP_ACTION_H

#include <stdint.h>

/*
 * Action 安全契约 —— 跨时间、独占执行器的任务的统一生命周期。
 *
 * 解决的问题：上位机不能持有危险动作的时间轴。
 *
 *   反模式：上位机 send("SERVO 1 500"); sleep(2); send("SERVO 1 1500")
 *           这两行之间进程被 kill / USB 被拔 / 用户 Ctrl+C
 *           -> 舵机永远停在 500，堵转发热，固件一无所知。
 *
 * 判据很简单：**上位机进程被 kill -9，物理世界会不会停在危险态？**
 * 会 -> 这个动作必须是 Action，它的时间轴、超时、急停必须闭环在固件端。
 *
 * ---------------------------------------------------------------------------
 * 为什么这个模块不碰 HAL / RTOS / printf
 * ---------------------------------------------------------------------------
 * 因为它的正确性无法在真机上验证 —— 恰恰是超时、断线、急停这些路径，最难
 * 在实机上复现，而本项目当前无法实机烧录。所以时间靠参数注入（每个入口都收
 * now_ms），执行器靠回调注入（APP_ActionDescriptor），模块内部不调用任何
 * HAL_GetTick / osDelay / 队列 / 互斥量。
 *
 * 代价是调用方要负责传时间；收益是整个状态机可以用 host gcc 编译，把
 * "第 1.5 秒拔掉 USB，第 4 秒舵机必须回中" 这种场景写成确定性单元测试。
 * 参考 tests/test_action_contract.py —— 那是这个模块唯一的验收依据。
 *
 * ---------------------------------------------------------------------------
 * 生命周期
 * ---------------------------------------------------------------------------
 *
 *              START (通过前置互锁)
 *   IDLE ─────────────────────────────> RUNNING
 *     ^                                   │
 *     │                                   ├── step 报告完成 ──> COMPLETED
 *     │                                   ├── step 报告失败 ──> FAILED
 *     │                                   ├── 超时 ──────────> FAILED (TIMEOUT)
 *     │                                   ├── 心跳丢失 ──────> FAILED (LINK_LOST)
 *     │                                   └── CANCEL ────────> CANCELLED
 *     │                                             │
 *     └───────────── APP_Action_Reset ──────────────┘
 *
 * 不变量（由测试逐条锁定）：
 *   I1. 只要离开 RUNNING，safe_reset 一定被调用，且**恰好一次**。
 *       无论是正常完成、超时、断线、取消还是执行器报错 —— 没有例外路径。
 *   I2. 终态下继续 Tick 不会再次调用 safe_reset。
 *   I3. 前置互锁拒绝的 START 不会调用 begin，也不会调用 safe_reset
 *       （没启动的东西不需要复位，多余的复位本身就是一次意外的执行器动作）。
 *   I4. 同一 action_id 重复 START 是幂等的，不会重启任务。
 *       链路抖动时上位机重发 START 是常态，重启一次舵机扫频就是事故。
 *   I5. 时间比较全部走无符号差值，HAL_GetTick 在 2^32 ms 回绕时不会误判超时。
 */

/* 任务状态。数值会上报给上位机，只可追加。 */
typedef enum {
    APP_ACTION_STATE_IDLE = 0,
    APP_ACTION_STATE_RUNNING = 1,
    APP_ACTION_STATE_COMPLETED = 2,
    APP_ACTION_STATE_FAILED = 3,
    APP_ACTION_STATE_CANCELLED = 4
} APP_ActionState;

/* 失败原因。数值会上报给上位机，只可追加。 */
typedef enum {
    APP_ACTION_ERR_NONE = 0,
    APP_ACTION_ERR_ARMED = 1,        /* 已解锁，禁止启动任何 Action */
    APP_ACTION_ERR_BUSY = 2,         /* 已有其它 Action 在跑 */
    APP_ACTION_ERR_PRECONDITION = 3, /* 任务自身的前置条件不满足 */
    APP_ACTION_ERR_BEGIN = 4,        /* begin 回调报错 */
    APP_ACTION_ERR_EFFECTOR = 5,     /* step 回调报错 */
    APP_ACTION_ERR_TIMEOUT = 6,      /* 达到 timeout_ms 硬上限 */
    APP_ACTION_ERR_LINK_LOST = 7,    /* 心跳丢失 */
    APP_ACTION_ERR_CANCELLED = 8,    /* 上位机主动取消 */
    APP_ACTION_ERR_INVALID = 9       /* 参数非法 */
} APP_ActionError;

/* step 回调的返回值。 */
#define APP_ACTION_STEP_RUNNING 0
#define APP_ACTION_STEP_DONE    1
#define APP_ACTION_STEP_FAILED  (-1)

/*
 * 任务描述符。每个具体任务（舵机扫频、电机点动、IMU 校准…）提供一份。
 *
 * safe_reset 是整个契约的核心：它必须在任何时刻调用都是安全的，必须是
 * 无条件的（不能"检查状态后再决定要不要断能"），且必须不会失败。它是断线和
 * 超时路径上唯一的保护，没有第二道防线。
 */
typedef struct {
    const char *name;

    /* timeout_ms 的硬上限。上位机请求超过这个值一律被钳到这里。 */
    uint32_t max_timeout_ms;

    /* 上位机未指定时使用的超时。 */
    uint32_t default_timeout_ms;

    /*
     * 心跳超时。0 表示该任务不要求心跳。
     * 非 0 时，上位机必须周期性调用 APP_Action_Heartbeat，否则任务中止。
     * 这一层比 timeout_ms 更快：拔线场景下不必等满整个 timeout。
     */
    uint32_t heartbeat_timeout_ms;

    /* 可选。返回 0 表示前置条件满足；非 0 表示拒绝启动。 */
    uint8_t (*precondition)(void *ctx);

    /* 必需。返回 0 表示启动成功。 */
    uint8_t (*begin)(void *ctx, uint32_t now_ms);

    /* 必需。返回 APP_ACTION_STEP_*，并写出 0..100 的进度。 */
    int8_t (*step)(void *ctx, uint32_t now_ms, uint8_t *progress_percent);

    /* 必需。无条件把执行器带回安全态，不允许失败。 */
    void (*safe_reset)(void *ctx);
} APP_ActionDescriptor;

typedef struct {
    const APP_ActionDescriptor *desc;
    void *ctx;

    APP_ActionState state;
    APP_ActionError error;

    uint32_t action_id;
    uint32_t started_ms;
    uint32_t timeout_ms;
    uint32_t last_heartbeat_ms;
    uint8_t  progress_percent;

    /* 诊断计数，供测试与现场排查确认 I1/I2 成立。 */
    uint32_t safe_reset_count;
} APP_ActionRunner;

/* 把 runner 置为 IDLE。不会调用任何回调。 */
void APP_Action_Init(APP_ActionRunner *runner);

/*
 * 启动一个 Action。
 *
 * armed 非 0 时无条件拒绝（互锁），这是最高优先级的检查，先于一切。
 * timeout_ms 传 0 表示用描述符的默认值；超过 max_timeout_ms 会被钳位。
 *
 * 返回 APP_ACTION_ERR_NONE 表示已进入 RUNNING。
 */
APP_ActionError APP_Action_Start(APP_ActionRunner *runner,
                                 const APP_ActionDescriptor *desc,
                                 void *ctx,
                                 uint32_t action_id,
                                 uint32_t timeout_ms,
                                 uint32_t now_ms,
                                 uint8_t armed);

/* 刷新心跳。action_id 不匹配当前任务时忽略（防止陈旧心跳续命）。 */
void APP_Action_Heartbeat(APP_ActionRunner *runner,
                          uint32_t action_id,
                          uint32_t now_ms);

/* 主动取消。非 RUNNING 时无副作用。 */
void APP_Action_Cancel(APP_ActionRunner *runner, uint32_t now_ms);

/*
 * 周期推进。必须由一个**不会被通信阻塞**的固定周期任务调用 —— 如果把它挂在
 * 处理上位机命令的任务上，那个任务因 UART/Flash 阻塞时看门狗会跟着一起死，
 * 断线保护就是假的。
 */
void APP_Action_Tick(APP_ActionRunner *runner, uint32_t now_ms);

/* 从终态回到 IDLE。RUNNING 时不做任何事（必须先 Cancel）。 */
void APP_Action_Reset(APP_ActionRunner *runner);

/* 状态查询。 */
APP_ActionState APP_Action_GetState(const APP_ActionRunner *runner);
APP_ActionError APP_Action_GetError(const APP_ActionRunner *runner);
uint8_t APP_Action_GetProgress(const APP_ActionRunner *runner);
const char *APP_Action_StateName(APP_ActionState state);
const char *APP_Action_ErrorName(APP_ActionError error);

#endif /* APP_ACTION_H */
