#ifndef APP_PROP_SPIN_H
#define APP_PROP_SPIN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 标定用的"上位机点电机"窗口：一次只转一路，靠心跳续命。
 *
 * ──────────────── 为什么需要它 ────────────────
 *
 * 要知道哪个 ESC 通道接的是上桨还是下桨、每路往哪边转，只有一个办法：
 * 让它转一下，用眼睛看。在此之前固件里没有任何一条能从上位机让电机转的路径
 * （`APP_CONTROL_ALLOW_RAW_MOTOR_COMMANDS` 一直是 0，且**保持为 0**），
 * 于是旋向只能靠调参现象反推——那正是 `drv_prop_map.h` 开头讲的那个坑。
 *
 * ──────────────── 为什么是心跳而不是超时 ────────────────
 *
 * "发一条命令转 3 秒然后自己停"这种写法，在上位机崩溃、USB 拔掉、笔记本睡眠、
 * 或者操作者转身走开时，电机照转不误——因为停下来这件事**不需要任何人还活着**。
 * 心跳把它反过来：**转下去才需要有人活着。**上位机每 100 ms 重发一次油门命令，
 * 超过 `APP_PROP_SPIN_TIMEOUT_MS` 没听到就关窗。连接断了、面板卡了、进程被杀了，
 * 结果都一样是停。
 *
 * ──────────────── 超时必须由控制环检查 ────────────────
 *
 * `APP_PropSpin_Step()` 跑在 500 Hz 的执行器提交点上，不在收命令的文本任务里。
 * 这不是风格问题：文本任务会因为 Flash 擦写、大段打印而阻塞几十毫秒甚至更久，
 * 而**判断"该停了"的那段代码如果和收命令的代码在同一个任务里，它们会一起卡住**。
 * 放在控制环里，唯一能让电机停不下来的情形是控制环本身死了——而那时看门狗会复位。
 *
 * ──────────────── 本模块的边界 ────────────────
 *
 * 纯状态机：不含 HAL、不含 RTOS、不做 I/O、不碰 GPIO。输出是"哪一路给多少百分比"，
 * 由 `app_stabilizer.c` 在提交点翻译成 `BSP_PWM_SetEscPercent`。因此它可以在 PC 上
 * 整个跑一遍（`tests/test_prop_spin_safety.py`），包括超时与抢占。
 */

#define APP_PROP_SPIN_CHANNEL_COUNT 2U

/*
 * 心跳超时。100 ms 的重发周期下留三拍余量：Tk 的 after 回调在重绘忙时会晚到，
 * 阈值贴着周期设会让窗口在正常使用中随机关闭，而"随机停"会训练操作者去忽略它。
 */
#define APP_PROP_SPIN_TIMEOUT_MS 300U

/*
 * ──────────────── 油门上限：一条硬顶 + 一条每次自己报的线 ────────────────
 *
 * 看旋向只需要电机转起来，不需要推力，所以**不要求**开窗的人说话时默认就是 20%
 * （约 1268 µs 等效命令）。但这一页后来也被用来在台架上测电流——那件事需要把
 * 油门推到能产生可观电流的程度，20% 够不着。
 *
 * 于是拆成两层：
 *   - `HARD_MAX`   编译期硬顶，任何命令都越不过去；
 *   - `DEFAULT_MAX` 不说话时给的上限，保持原来的 20%。
 *
 * 想要更高必须在 `PROPCAL SPIN ARM` 里**显式写出来**。这比"把常量从 20 改成 100"
 * 强的地方在于：默认路径的保护一点没少，而抬高上限成了一个留痕的、当次有效的
 * 动作——它跟着窗口一起关，下次开窗又回到 20。
 *
 * 上限仍然由固件裁决而不是只写在界面上：界面是可以被绕过的，而这条命令能让桨转。
 */
#define APP_PROP_SPIN_HARD_MAX_PERCENT    100U
#define APP_PROP_SPIN_DEFAULT_MAX_PERCENT  20U

typedef enum {
    APP_PROP_SPIN_STOP_NONE = 0U,
    APP_PROP_SPIN_STOP_REQUEST = 1U,    /* 上位机主动停 */
    APP_PROP_SPIN_STOP_HEARTBEAT = 2U,  /* 心跳超时：连接断了 / 面板卡了 */
    APP_PROP_SPIN_STOP_INHIBIT = 3U,    /* 解锁、验收、舵机标定或辨识抢走了输出 */
    APP_PROP_SPIN_STOP_REJECTED = 4U,   /* 命令非法，窗口随之关闭 */
} APP_PropSpinStopReason;

typedef struct {
    uint8_t active;
    uint8_t channel;                              /* 1..2；0 = 还没选 */
    uint8_t percent[APP_PROP_SPIN_CHANNEL_COUNT]; /* 下标 0 = ESC 通道 1 */
    /* 本次窗口的上限。窗口关着时是 DEFAULT_MAX，不是上一次用过的值——
     * 让界面在停下来之后自动缩回 20%，而不是把高上限留在那里等人误拖。 */
    uint8_t max_percent;
} APP_PropSpinOutput;

void APP_PropSpin_Reset(void);

/*
 * 开窗，并钉死**本次**的油门上限（1..HARD_MAX；越界一律不开窗、也不动已开的窗口）。
 *
 * 已经开着时返回 1（幂等），这样上位机重连后重发一次 ARM 不会误报失败。但此时
 * 上限**只降不升**：一个已经开着的窗口不该靠重发一条命令拿到更大的权限，那会让
 * "先用 20% 确认安全、再偷偷抬到 100%"变成一条无声的路径。想抬高必须先 Close。
 */
uint8_t APP_PropSpin_Open(uint32_t now_ms, uint8_t max_percent);

/*
 * 一次心跳兼油门命令。**只有它能续命**，而且只有合法命令才续：
 * 非法通道/超限油门不刷新心跳，还会当场关窗——一个在乱发命令的上位机，
 * 没有理由继续被信任着让电机转。
 */
uint8_t APP_PropSpin_Command(uint32_t now_ms, uint8_t channel, uint8_t percent);

void APP_PropSpin_Close(uint32_t now_ms, uint8_t reason);

/*
 * 500 Hz 节拍。`inhibit` 非零表示有更高优先级的东西正在用执行器
 * （已解锁 / 验收 / 舵机标定 / 系统辨识），此时立即关窗。
 * `now_ms` 用 32 位毫秒计数，差值比较，跨 49 天回绕安全。
 */
void APP_PropSpin_Step(uint32_t now_ms, uint8_t inhibit);

uint8_t APP_PropSpin_IsActive(void);

/* 窗口关着时 active=0、channel=0、两路 percent 都是 0。调用方照发即可。 */
void APP_PropSpin_GetOutput(APP_PropSpinOutput *out);

uint8_t  APP_PropSpin_LastStopReason(void);
uint32_t APP_PropSpin_HeartbeatAgeMs(uint32_t now_ms);
const char *APP_PropSpin_StopReasonName(uint8_t reason);

#ifdef __cplusplus
}
#endif

#endif /* APP_PROP_SPIN_H */
