#ifndef APP_NAV_ESTIMATOR_H
#define APP_NAV_ESTIMATOR_H

#include "drv_nav_ekf.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * R-M5-5：速度估计器本体已经搬到 Services/Src/svc_flow_nav.c，这里只剩一个
 * 诊断快照适配器。Publish 不再由调用方传状态进来，改为自己去 Service 取——
 * 稳定环因此不必再见到任何 DRV_NAV_EKF_* 符号。
 *
 * R-F6-1（只钉现状）：本模块不持有、也不重新定义坐标系——它转发的
 * DRV_NAV_EKF_Diagnostics 里的 vel_m_s[2] 轴序与 Services/Inc/svc_flow_nav.h
 * 的 SVC_FLOW_NAV_FuseInput/State 一致（机体系 X 前 / Y 右正，legacy 约定），
 * 不是导航系坐标。
 */
void APP_NavEstimator_PublishVelocityEKF(void);
void APP_NavEstimator_GetVelocityEKF(
    DRV_NAV_EKF_Diagnostics *diagnostics);

#ifdef __cplusplus
}
#endif

#endif
