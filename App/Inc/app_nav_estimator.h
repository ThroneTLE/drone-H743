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
 */
void APP_NavEstimator_PublishVelocityEKF(void);
void APP_NavEstimator_GetVelocityEKF(
    DRV_NAV_EKF_Diagnostics *diagnostics);

#ifdef __cplusplus
}
#endif

#endif
