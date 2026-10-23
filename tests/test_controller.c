#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "controller.h"

static void close_to(float a, float b, float tol)
{
    assert(isfinite(a) && fabsf(a - b) < tol);
}

static void test_estimator(void)
{
    TiltEstimator est;
    tilt_estimator_init(&est, 0.5f, 0.0f);
    close_to(tilt_estimator_update(&est, 0.0f, 1.0f, 0.0f, 0.005f), 0.0f, 0.001f);
    close_to(tilt_estimator_update(&est, 0.0f, 1.0f, 10.0f, 0.005f), 0.049505f, 0.002f);
    tilt_estimator_init(&est, 0.5f, 0.0f);
    close_to(tilt_estimator_update(&est, 0.173648f, 0.984808f, 0.0f, 0.005f),
             10.0f, 0.02f);
    assert(isnan(tilt_estimator_update(&est, 0.0f, 1.0f, NAN, 0.005f)));
    assert(isnan(tilt_estimator_update(&est, 0.0f, 0.0f, 0.0f, 0.005f)));
    assert(isnan(tilt_estimator_update(&est, 0.0f, 1.0f, 0.0f, 0.0f)));
    assert(isnan(tilt_estimator_update(&est, 0.0f, 1.0f, 0.0f, 0.025f)));
}

static void test_controller(void)
{
    BalanceController pid;
    balance_controller_init(&pid, 0.02f, 0.01f, 0.001f);
    close_to(balance_controller_update(&pid, 5.0f, 0.0f, 0.005f),
             -0.10025f, 0.001f);
    for (int i = 0; i < 200; ++i) {
        float u = balance_controller_update(&pid, 1000.0f, 0.0f, 0.005f);
        assert(u >= -0.7001f && u <= 0.7001f);
    }
    assert(fabsf(pid.integral) < 1.0f); /* conditional anti-windup */
    balance_controller_reset(&pid);
    close_to(pid.integral, 0.0f, 0.0001f);
    assert(isnan(balance_controller_update(&pid, NAN, 0.0f, 0.005f)));
    assert(isnan(balance_controller_update(&pid, 0.0f, 0.0f, 0.050f)));
    assert(balance_is_safe(0.0f, true, true));
    assert(!balance_is_safe(25.0f, true, true));
    assert(!balance_is_safe(0.0f, false, true));
    assert(!balance_is_safe(0.0f, true, false));
    assert(!balance_is_safe(NAN, true, true));
}

static void test_arming(void)
{
    BalanceSafety safety;
    balance_safety_init(&safety);
    assert(safety.mode == BALANCE_DISARMED);
    for (int i = 0; i < 60; ++i)
        assert(!balance_safety_update(&safety, 0.0f, true, true));
    assert(!balance_safety_update(&safety, 0.0f, false, true));
    for (int i = 0; i < (int)BALANCE_ARM_SAMPLE_COUNT-1; ++i)
        assert(!balance_safety_update(&safety, 2.0f, true, true));
    assert(safety.mode == BALANCE_ARMING);
    assert(balance_safety_update(&safety, 2.0f, true, true));
    assert(safety.mode == BALANCE_ARMED);
    assert(balance_safety_update(&safety, 12.0f, true, true));
    assert(!balance_safety_update(&safety, 25.0f, true, true));
    assert(safety.mode == BALANCE_FAULT);
    assert(!balance_safety_update(&safety, 0.0f, true, true));
    assert(!balance_safety_update(&safety, 0.0f, false, true));
    assert(safety.mode == BALANCE_DISARMED);
    for (int i = 0; i < 12; ++i)
        assert(!balance_safety_update(&safety, 2.0f, true, true));
    assert(!balance_safety_update(&safety, 9.0f, true, true));
    assert(safety.mode == BALANCE_DISARMED);
    assert(safety.upright_samples == 0U);

    assert(!balance_safety_update(&safety, NAN, true, true));
    assert(safety.mode == BALANCE_FAULT);
    assert(!balance_safety_update(&safety, 0.0f, false, false));
    assert(safety.mode == BALANCE_FAULT);
    assert(!balance_safety_update(&safety, 0.0f, false, true));
    assert(safety.mode == BALANCE_DISARMED);
}

int main(void)
{
    test_estimator();
    test_controller();
    test_arming();
    puts("PASS: estimator, PID, arming, fault/reset and safety regression tests");
    return 0;
}
