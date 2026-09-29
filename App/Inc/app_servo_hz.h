#ifndef APP_SERVO_HZ_H
#define APP_SERVO_HZ_H

#include <stdint.h>

/*
 * `SERVOHZ` 命令族：PWM 舵机帧率的运行时切换（2026-09-28，50 Hz 与 333 Hz 台架 A/B）。
 *
 *   SERVOHZ / SERVOHZ ?      -> SERVOHZ hz=<帧率> frame_us=<帧长>
 *   SERVOHZ <50..333>        -> 同上（生效后）；只在上锁且光杆辨识没在跑时收
 *   拒绝                      -> SERVOHZ event=rejected reason=armed|sysid|range|pwm
 *
 * 只在 RAM：上电回到编译期默认（BSP_PWM_SERVO_FRAME_HZ，2026-09-28 起 333 Hz）。帧率变了，舵机指令"等下一帧"的延迟就变了（50 Hz 平均约 10 ms，
 * 333 Hz 约 1.5 ms），辨识出来的纯延迟随之不同——所以光杆辨识开跑的 `SYSID BACKLASH`
 * 溯源行尾带 `servo_hz=`，每一轮都记下当时的帧率。
 */
uint8_t APP_ServoHz_Command(char **tokens, uint32_t count);

#endif /* APP_SERVO_HZ_H */
