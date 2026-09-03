#include "app_nav_estimator.h"

#include "svc_flow_nav.h"

#include <stddef.h>

static DRV_NAV_EKF_Diagnostics nav_estimator_velocity_ekf;

void APP_NavEstimator_PublishVelocityEKF(void)
{
    SVC_FlowNav_GetEkfDiagnostics(&nav_estimator_velocity_ekf);
}

void APP_NavEstimator_GetVelocityEKF(
    DRV_NAV_EKF_Diagnostics *diagnostics)
{
    if (diagnostics == NULL) {
        return;
    }

    *diagnostics = nav_estimator_velocity_ekf;
}
