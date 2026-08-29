#include "app_action.h"

#include <stddef.h>

/*
 * 无符号差值。HAL_GetTick 在 2^32 ms（约 49.7 天）回绕，直接写
 * (now > started + timeout) 在回绕点会漏判超时 —— 那意味着舵机永远不回中。
 * 无符号减法天然处理回绕，只要真实经过时间小于 2^32 ms。
 */
static uint32_t app_action_elapsed(uint32_t now_ms, uint32_t since_ms)
{
    return now_ms - since_ms;
}

/*
 * 离开 RUNNING 的唯一出口。
 *
 * 所有终止路径（完成/失败/超时/断线/取消）都必须经过这里，safe_reset 才能
 * 保证"恰好一次"。任何绕过这个函数直接改 state 的代码都是 bug。
 */
static void app_action_finish(APP_ActionRunner *runner,
                              APP_ActionState state,
                              APP_ActionError error)
{
    if (runner->state != APP_ACTION_STATE_RUNNING) {
        return;
    }

    runner->state = state;
    runner->error = error;

    if ((runner->desc != NULL) && (runner->desc->safe_reset != NULL)) {
        runner->desc->safe_reset(runner->ctx);
        runner->safe_reset_count++;
    }
}

void APP_Action_Init(APP_ActionRunner *runner)
{
    if (runner == NULL) {
        return;
    }

    runner->desc = NULL;
    runner->ctx = NULL;
    runner->state = APP_ACTION_STATE_IDLE;
    runner->error = APP_ACTION_ERR_NONE;
    runner->action_id = 0U;
    runner->started_ms = 0U;
    runner->timeout_ms = 0U;
    runner->last_heartbeat_ms = 0U;
    runner->progress_percent = 0U;
    runner->safe_reset_count = 0U;
}

APP_ActionError APP_Action_Start(APP_ActionRunner *runner,
                                 const APP_ActionDescriptor *desc,
                                 void *ctx,
                                 uint32_t action_id,
                                 uint32_t timeout_ms,
                                 uint32_t now_ms,
                                 uint8_t armed)
{
    uint32_t effective_timeout;

    if ((runner == NULL) || (desc == NULL) ||
        (desc->begin == NULL) || (desc->step == NULL) ||
        (desc->safe_reset == NULL)) {
        return APP_ACTION_ERR_INVALID;
    }

    /*
     * 互锁先于一切。飞行中误触自检会让执行器失控或让姿态基准归零，所以这个
     * 检查必须在幂等判断之前 —— 哪怕是"重发同一个 action_id"，只要现在已解锁
     * 就一律拒绝。
     */
    if (armed != 0U) {
        return APP_ACTION_ERR_ARMED;
    }

    if (runner->state == APP_ACTION_STATE_RUNNING) {
        /*
         * 幂等：链路抖动时上位机重发 START 是常态。同一个 action_id 视为
         * 同一次请求，直接确认成功而不重启任务 —— 重启一次舵机扫频就是事故。
         */
        if ((runner->action_id == action_id) && (runner->desc == desc)) {
            return APP_ACTION_ERR_NONE;
        }
        return APP_ACTION_ERR_BUSY;
    }

    if ((desc->precondition != NULL) && (desc->precondition(ctx) != 0U)) {
        return APP_ACTION_ERR_PRECONDITION;
    }

    effective_timeout = (timeout_ms == 0U) ? desc->default_timeout_ms : timeout_ms;
    if ((desc->max_timeout_ms != 0U) && (effective_timeout > desc->max_timeout_ms)) {
        effective_timeout = desc->max_timeout_ms;
    }

    runner->desc = desc;
    runner->ctx = ctx;
    runner->action_id = action_id;
    runner->started_ms = now_ms;
    runner->last_heartbeat_ms = now_ms;
    runner->timeout_ms = effective_timeout;
    runner->progress_percent = 0U;
    runner->error = APP_ACTION_ERR_NONE;

    if (desc->begin(ctx, now_ms) != 0U) {
        /*
         * begin 失败也要复位：它可能已经动了一部分执行器才失败。
         * 先置 RUNNING 再走 finish，是为了让这条路径同样满足 I1。
         */
        runner->state = APP_ACTION_STATE_RUNNING;
        app_action_finish(runner, APP_ACTION_STATE_FAILED, APP_ACTION_ERR_BEGIN);
        return APP_ACTION_ERR_BEGIN;
    }

    runner->state = APP_ACTION_STATE_RUNNING;
    return APP_ACTION_ERR_NONE;
}

void APP_Action_Heartbeat(APP_ActionRunner *runner,
                          uint32_t action_id,
                          uint32_t now_ms)
{
    if ((runner == NULL) || (runner->state != APP_ACTION_STATE_RUNNING)) {
        return;
    }

    /*
     * 只接受当前任务的心跳。否则上一个任务残留在链路上的心跳包会给新任务
     * 续命，断线保护就被悄悄绕过了。
     */
    if (runner->action_id != action_id) {
        return;
    }

    runner->last_heartbeat_ms = now_ms;
}

void APP_Action_Cancel(APP_ActionRunner *runner, uint32_t now_ms)
{
    (void)now_ms;

    if (runner == NULL) {
        return;
    }

    app_action_finish(runner, APP_ACTION_STATE_CANCELLED, APP_ACTION_ERR_CANCELLED);
}

void APP_Action_Tick(APP_ActionRunner *runner, uint32_t now_ms)
{
    int8_t step_result;
    uint8_t progress;

    if ((runner == NULL) || (runner->state != APP_ACTION_STATE_RUNNING)) {
        return;
    }

    /*
     * 超时与断线先于 step 检查。
     *
     * 顺序是有意的：一个卡死或耗时过长的 step 不应该有机会把超时推迟一个周期。
     * 保护路径必须先跑。
     */
    if ((runner->timeout_ms != 0U) &&
        (app_action_elapsed(now_ms, runner->started_ms) >= runner->timeout_ms)) {
        app_action_finish(runner, APP_ACTION_STATE_FAILED, APP_ACTION_ERR_TIMEOUT);
        return;
    }

    if ((runner->desc->heartbeat_timeout_ms != 0U) &&
        (app_action_elapsed(now_ms, runner->last_heartbeat_ms) >=
         runner->desc->heartbeat_timeout_ms)) {
        app_action_finish(runner, APP_ACTION_STATE_FAILED, APP_ACTION_ERR_LINK_LOST);
        return;
    }

    progress = runner->progress_percent;
    step_result = runner->desc->step(runner->ctx, now_ms, &progress);
    if (progress > 100U) {
        progress = 100U;
    }
    runner->progress_percent = progress;

    if (step_result == APP_ACTION_STEP_DONE) {
        runner->progress_percent = 100U;
        app_action_finish(runner, APP_ACTION_STATE_COMPLETED, APP_ACTION_ERR_NONE);
    } else if (step_result != APP_ACTION_STEP_RUNNING) {
        app_action_finish(runner, APP_ACTION_STATE_FAILED, APP_ACTION_ERR_EFFECTOR);
    }
}

void APP_Action_Reset(APP_ActionRunner *runner)
{
    if ((runner == NULL) || (runner->state == APP_ACTION_STATE_RUNNING)) {
        return;
    }

    runner->state = APP_ACTION_STATE_IDLE;
    runner->error = APP_ACTION_ERR_NONE;
    runner->progress_percent = 0U;
}

APP_ActionState APP_Action_GetState(const APP_ActionRunner *runner)
{
    return (runner == NULL) ? APP_ACTION_STATE_IDLE : runner->state;
}

APP_ActionError APP_Action_GetError(const APP_ActionRunner *runner)
{
    return (runner == NULL) ? APP_ACTION_ERR_NONE : runner->error;
}

uint8_t APP_Action_GetProgress(const APP_ActionRunner *runner)
{
    return (runner == NULL) ? 0U : runner->progress_percent;
}

const char *APP_Action_StateName(APP_ActionState state)
{
    switch (state) {
    case APP_ACTION_STATE_IDLE:
        return "IDLE";
    case APP_ACTION_STATE_RUNNING:
        return "RUNNING";
    case APP_ACTION_STATE_COMPLETED:
        return "COMPLETED";
    case APP_ACTION_STATE_FAILED:
        return "FAILED";
    case APP_ACTION_STATE_CANCELLED:
        return "CANCELLED";
    default:
        return "UNKNOWN";
    }
}

const char *APP_Action_ErrorName(APP_ActionError error)
{
    switch (error) {
    case APP_ACTION_ERR_NONE:
        return "none";
    case APP_ACTION_ERR_ARMED:
        return "armed";
    case APP_ACTION_ERR_BUSY:
        return "busy";
    case APP_ACTION_ERR_PRECONDITION:
        return "precondition";
    case APP_ACTION_ERR_BEGIN:
        return "begin";
    case APP_ACTION_ERR_EFFECTOR:
        return "effector";
    case APP_ACTION_ERR_TIMEOUT:
        return "timeout";
    case APP_ACTION_ERR_LINK_LOST:
        return "link_lost";
    case APP_ACTION_ERR_CANCELLED:
        return "cancelled";
    case APP_ACTION_ERR_INVALID:
        return "invalid";
    default:
        return "unknown";
    }
}
