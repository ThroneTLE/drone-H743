/*
 * 磁力计服务层实现。纯函数 + 纯数据，不含 HAL / RTOS / I/O。
 * 设计与符号陷阱的说明在 svc_mag.h，此处不重复。
 */

#include "svc_mag.h"

/*
 * 第一步：芯片轴 -> **FRD** 机体轴，语义与 ArduPilot 的 ROTATION_* 完全一致。
 * 与 Services/Src/svc_imu.c 里的同名表逐条一致（两个模块各自独立维护自己的
 * 8 项子集，因为磁力计和 IMU 完全可能装在板子的不同角落、朝向也不同）。
 *
 * 各式子的来源（把基本旋转依次作用于向量，而不是背表）：
 *   YAW_90   : (x, y) -> (-y,  x)
 *   YAW_180  : (x, y) -> (-x, -y)
 *   YAW_270  : (x, y) -> ( y, -x)
 *   ROLL_180 : (y, z) -> (-y, -z)
 * 组合 ROLL_180_YAW_n 是"先 ROLL_180 再 YAW_n"，例如
 *   ROLL_180        -> ( x, -y, -z)
 *   再 YAW_270      -> (-y, -x, -z)   即 ROLL_180_YAW_270
 */
static DRV_FRAME_Vector3f rotate_chip_to_frd(SVC_MAG_Rotation rotation,
                                             DRV_FRAME_Vector3f v)
{
    DRV_FRAME_Vector3f out = v;

    switch (rotation) {
    case SVC_MAG_ROTATION_NONE:
        break;
    case SVC_MAG_ROTATION_YAW_90:
        out.x = -v.y; out.y =  v.x; out.z =  v.z;
        break;
    case SVC_MAG_ROTATION_YAW_180:
        out.x = -v.x; out.y = -v.y; out.z =  v.z;
        break;
    case SVC_MAG_ROTATION_YAW_270:
        out.x =  v.y; out.y = -v.x; out.z =  v.z;
        break;
    case SVC_MAG_ROTATION_ROLL_180:
        out.x =  v.x; out.y = -v.y; out.z = -v.z;
        break;
    case SVC_MAG_ROTATION_ROLL_180_YAW_90:
        out.x =  v.y; out.y =  v.x; out.z = -v.z;
        break;
    case SVC_MAG_ROTATION_PITCH_180:
        out.x = -v.x; out.y =  v.y; out.z = -v.z;
        break;
    case SVC_MAG_ROTATION_ROLL_180_YAW_270:
        out.x = -v.y; out.y = -v.x; out.z = -v.z;
        break;
    default:
        /*
         * 未知朝向按"不旋转"处理，而不是返回零或随便挑一个。
         * 姿态会明显不对、一眼能看出来；返回零反而像"传感器坏了"，更难定位。
         */
        break;
    }

    return out;
}

DRV_FRAME_Vector3f SVC_MAG_RotateToFlu(SVC_MAG_Rotation rotation,
                                       DRV_FRAME_Vector3f chip_axis_value)
{
    /* 两步走，第二步复用坐标契约里既有的适配器，不另起一套符号约定。 */
    DRV_FRAME_Vector3f frd = rotate_chip_to_frd(rotation, chip_axis_value);
    return DRV_FRAME_FrdToFlu(frd);
}

SVC_MAG_Rotation SVC_MAG_DefaultRotation(void)
{
    /* 见 svc_mag.h 里本函数声明上方的完整推导；本板两条 hwdef 记录都是
     * ROTATION_NONE，没有"按芯片种类切换"的必要。 */
    return SVC_MAG_ROTATION_NONE;
}
