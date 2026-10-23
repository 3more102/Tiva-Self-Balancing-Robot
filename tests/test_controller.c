#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "controller.h"

static void close_to(float a, float b, float tol) { assert(fabsf(a - b) < tol); }

int main(void)
{
    TiltEstimator est;
    BalanceController ctrl;
    tilt_estimator_init(&est, 0.5f, 0.0f);
    close_to(tilt_estimator_update(&est, 0.0f, 1.0f, 0.0f, 0.005f), 0.0f, 0.001f);
    close_to(tilt_estimator_update(&est, 0.0f, 1.0f, 10.0f, 0.005f), 0.049505f, 0.002f);
    tilt_estimator_init(&est, 0.5f, 0.0f);
    close_to(tilt_estimator_update(&est, 0.173648f, 0.984808f, 0.0f, 0.005f), 10.0f, 0.02f);
    assert(isnan(tilt_estimator_update(&est, 0.0f, 1.0f, NAN, 0.005f)));

    balance_controller_init(&ctrl, 0.02f, 0.01f, 0.001f);
    close_to(balance_controller_update(&ctrl, 5.0f, 0.0f, 0.005f), -0.10025f, 0.001f);
    assert(balance_controller_update(&ctrl, 1000.0f, 0.0f, 0.005f) >= -0.7001f);
    assert(balance_controller_update(&ctrl, 1000.0f, 0.0f, 0.005f) <= 0.7001f);
    balance_controller_reset(&ctrl);
    close_to(ctrl.integral, 0.0f, 0.0001f);
    assert(isnan(balance_controller_update(&ctrl, NAN, 0.0f, 0.005f)));

    assert(balance_is_safe(0.0f, true, true));
    assert(!balance_is_safe(25.0f, true, true));
    assert(!balance_is_safe(0.0f, false, true));
    assert(!balance_is_safe(0.0f, true, false));
    assert(!balance_is_safe(NAN, true, true));
    puts("PASS: host-side controller and safety checks");
    return 0;
}
