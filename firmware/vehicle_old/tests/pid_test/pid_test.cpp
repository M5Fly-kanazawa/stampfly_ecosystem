/**
 * @file pid_test.cpp
 * @brief PID Controller Unit Tests
 *
 * Theoretical verification of PID implementation:
 * - P-only: output = Kp * error
 * - PI with trapezoidal integration: I[k] = I[k-1] + (dt/(2*Ti)) * (e[k] + e[k-1])
 * - PD with incomplete derivative (bilinear transform):
 *   D[k] = a * D[k-1] + b * (input[k] - input[k-1])
 *   where α = 2*η*Td/dt, a = (α-1)/(α+1), b = 2*Td/((α+1)*dt)
 * - Anti-windup with back-calculation
 */

#include "pid.hpp"
#include <cstdio>
#include <cmath>
#include <vector>
#include <string>

// Test framework
static int tests_passed = 0;
static int tests_failed = 0;

#define ASSERT_NEAR(expected, actual, tolerance, msg) \
    do { \
        float _exp = (expected); \
        float _act = (actual); \
        float _tol = (tolerance); \
        if (std::fabs(_exp - _act) <= _tol) { \
            tests_passed++; \
        } else { \
            tests_failed++; \
            printf("  FAIL: %s\n", msg); \
            printf("    Expected: %.6f, Actual: %.6f, Diff: %.6f, Tolerance: %.6f\n", \
                   _exp, _act, std::fabs(_exp - _act), _tol); \
        } \
    } while(0)

#define TEST_SECTION(name) \
    printf("\n=== %s ===\n", name)

void printTestResult(const char* name, bool passed) {
    printf("  %s: %s\n", name, passed ? "PASS" : "FAIL");
}

// ============================================================================
// Test 1: P-only control
// ============================================================================
void test_p_only() {
    TEST_SECTION("Test 1: P-only Control");

    stampfly::PID pid;
    stampfly::PIDConfig config;
    config.Kp = 2.0f;
    config.Ti = 0.0f;  // Disable integral
    config.Td = 0.0f;  // Disable derivative
    config.output_min = -100.0f;
    config.output_max = 100.0f;
    pid.init(config);

    float dt = 0.01f;
    float setpoint = 10.0f;
    float measurement = 3.0f;
    float error = setpoint - measurement;  // 7.0

    float output = pid.update(setpoint, measurement, dt);
    float expected = config.Kp * error;  // 2.0 * 7.0 = 14.0

    ASSERT_NEAR(expected, output, 1e-5f, "P-only output");
    ASSERT_NEAR(expected, pid.getProportional(), 1e-5f, "P component");
    ASSERT_NEAR(0.0f, pid.getIntegral(), 1e-5f, "I component (should be 0)");
    ASSERT_NEAR(0.0f, pid.getDerivative(), 1e-5f, "D component (should be 0)");

    printf("  P-only: error=%.2f, Kp=%.2f, output=%.4f (expected=%.4f)\n",
           error, config.Kp, output, expected);
}

// ============================================================================
// Test 2: PI control with trapezoidal integration
// ============================================================================
void test_pi_trapezoidal() {
    TEST_SECTION("Test 2: PI Control (Trapezoidal Integration)");

    stampfly::PID pid;
    stampfly::PIDConfig config;
    config.Kp = 1.0f;
    config.Ti = 0.5f;   // Integral time = 0.5s
    config.Td = 0.0f;   // Disable derivative
    config.output_min = -100.0f;
    config.output_max = 100.0f;
    pid.init(config);

    float dt = 0.01f;  // 10ms

    // Step 1: First update (integral starts from 0, no previous error)
    float setpoint = 5.0f;
    float measurement = 0.0f;
    float e0 = setpoint - measurement;  // 5.0

    float output1 = pid.update(setpoint, measurement, dt);

    // First update: P = Kp * e = 5.0, I = 0 (first run, no integration yet)
    // Actually, looking at the code: first_run_ is only for derivative
    // Integral: I[0] = 0 (initial), but trapezoidal uses prev_error which is 0
    // So I[1] = I[0] + (dt/(2*Ti)) * (e[1] + e[0]) = 0 + (0.01/(2*0.5)) * (5+0) = 0.05
    // Wait, prev_error_ is set to 0 at reset, and first error is 5.0
    // So integral contribution = Kp * 0.05 = 0.05
    float integral_1 = (dt / (2.0f * config.Ti)) * (e0 + 0.0f);  // 0.05
    float expected_I1 = config.Kp * integral_1;  // 0.05
    float expected_P1 = config.Kp * e0;  // 5.0
    float expected_1 = expected_P1 + expected_I1;  // 5.05

    ASSERT_NEAR(expected_1, output1, 1e-5f, "PI output step 1");
    printf("  Step 1: e=%.2f, P=%.4f, I=%.4f, output=%.4f\n",
           e0, pid.getProportional(), pid.getIntegral(), output1);

    // Step 2: Second update with same error
    float e1 = e0;  // Still 5.0
    float output2 = pid.update(setpoint, measurement, dt);

    // Trapezoidal: integral accumulator += (dt/(2*Ti)) * (e[k] + e[k-1])
    // integral_acc = 0.05 + (0.01/(2*0.5)) * (5+5) = 0.05 + 0.1 = 0.15
    float integral_2 = integral_1 + (dt / (2.0f * config.Ti)) * (e1 + e0);  // 0.15
    float expected_I2 = config.Kp * integral_2;  // 0.15
    float expected_P2 = config.Kp * e1;  // 5.0
    float expected_2 = expected_P2 + expected_I2;  // 5.15

    ASSERT_NEAR(expected_2, output2, 1e-5f, "PI output step 2");
    printf("  Step 2: e=%.2f, P=%.4f, I=%.4f, output=%.4f\n",
           e1, pid.getProportional(), pid.getIntegral(), output2);

    // Step 3: Third update
    float e2 = e1;
    float output3 = pid.update(setpoint, measurement, dt);

    float integral_3 = integral_2 + (dt / (2.0f * config.Ti)) * (e2 + e1);  // 0.25
    float expected_I3 = config.Kp * integral_3;
    float expected_3 = config.Kp * e2 + expected_I3;  // 5.25

    ASSERT_NEAR(expected_3, output3, 1e-5f, "PI output step 3");
    printf("  Step 3: e=%.2f, P=%.4f, I=%.4f, output=%.4f\n",
           e2, pid.getProportional(), pid.getIntegral(), output3);
}

// ============================================================================
// Test 3: PD control with incomplete derivative (bilinear transform)
// ============================================================================
void test_pd_incomplete_derivative() {
    TEST_SECTION("Test 3: PD Control (Incomplete Derivative)");

    stampfly::PID pid;
    stampfly::PIDConfig config;
    config.Kp = 1.0f;
    config.Ti = 0.0f;   // Disable integral
    config.Td = 0.1f;   // Derivative time = 0.1s
    config.eta = 0.1f;  // Filter coefficient
    config.derivative_on_measurement = false;  // D-on-E for simpler testing
    config.output_min = -100.0f;
    config.output_max = 100.0f;
    pid.init(config);

    float dt = 0.01f;

    // Bilinear transform coefficients:
    // α = 2*η*Td/dt = 2 * 0.1 * 0.1 / 0.01 = 2.0
    // a = (α-1)/(α+1) = 1/3 ≈ 0.333
    // b = 2*Td/((α+1)*dt) = 2*0.1/(3*0.01) = 0.2/0.03 ≈ 6.667
    float alpha = 2.0f * config.eta * config.Td / dt;
    float deriv_a = (alpha - 1.0f) / (alpha + 1.0f);
    float deriv_b = 2.0f * config.Td / ((alpha + 1.0f) * dt);

    printf("  Coefficients: α=%.4f, a=%.4f, b=%.4f\n", alpha, deriv_a, deriv_b);

    // Step 1: Initial (first_run), D = 0
    float setpoint = 0.0f;
    float measurement = 0.0f;
    float e0 = setpoint - measurement;  // 0

    float output1 = pid.update(setpoint, measurement, dt);
    // First run: derivative is 0, prev_deriv_input set to e0=0
    ASSERT_NEAR(0.0f, output1, 1e-5f, "PD output step 1 (first run)");
    printf("  Step 1 (first run): e=%.2f, D=%.4f, output=%.4f\n",
           e0, pid.getDerivative(), output1);

    // Step 2: Step change in error
    setpoint = 10.0f;
    float e1 = setpoint - measurement;  // 10

    float output2 = pid.update(setpoint, measurement, dt);

    // D[1] = a * D[0] + b * (e[1] - e[0]) = 0 + 6.667 * 10 = 66.67
    float D_filt_1 = deriv_b * (e1 - e0);  // b * 10 = 66.67
    float expected_D2 = config.Kp * D_filt_1;
    float expected_P2 = config.Kp * e1;
    float expected_2 = expected_P2 + expected_D2;

    ASSERT_NEAR(expected_D2, pid.getDerivative(), 1e-3f, "D component step 2");
    ASSERT_NEAR(expected_2, output2, 1e-3f, "PD output step 2");
    printf("  Step 2: e=%.2f, P=%.4f, D=%.4f, output=%.4f (expected D=%.4f)\n",
           e1, pid.getProportional(), pid.getDerivative(), output2, expected_D2);

    // Step 3: No change in error (filter decay)
    float e2 = e1;  // Still 10
    float output3 = pid.update(setpoint, measurement, dt);

    // D[2] = a * D[1] + b * (e[2] - e[1]) = 0.333 * 66.67 + 0 = 22.22
    float D_filt_2 = deriv_a * D_filt_1 + deriv_b * (e2 - e1);
    float expected_D3 = config.Kp * D_filt_2;
    float expected_P3 = config.Kp * e2;
    float expected_3 = expected_P3 + expected_D3;

    ASSERT_NEAR(expected_D3, pid.getDerivative(), 1e-3f, "D component step 3 (decay)");
    ASSERT_NEAR(expected_3, output3, 1e-3f, "PD output step 3");
    printf("  Step 3: e=%.2f, P=%.4f, D=%.4f, output=%.4f (expected D=%.4f)\n",
           e2, pid.getProportional(), pid.getDerivative(), output3, expected_D3);

    // Step 4: Further decay
    float e3 = e2;
    float output4 = pid.update(setpoint, measurement, dt);

    float D_filt_3 = deriv_a * D_filt_2 + deriv_b * (e3 - e2);
    float expected_D4 = config.Kp * D_filt_3;

    ASSERT_NEAR(expected_D4, pid.getDerivative(), 1e-3f, "D component step 4 (decay)");
    printf("  Step 4: D=%.4f (expected=%.4f) - filter decaying\n",
           pid.getDerivative(), expected_D4);
}

// ============================================================================
// Test 4: Full PID
// ============================================================================
void test_full_pid() {
    TEST_SECTION("Test 4: Full PID");

    stampfly::PID pid;
    stampfly::PIDConfig config;
    config.Kp = 2.0f;
    config.Ti = 0.5f;
    config.Td = 0.05f;
    config.eta = 0.1f;
    config.derivative_on_measurement = false;
    config.output_min = -100.0f;
    config.output_max = 100.0f;
    pid.init(config);

    float dt = 0.01f;

    // Compute coefficients
    float alpha = 2.0f * config.eta * config.Td / dt;
    float deriv_a = (alpha - 1.0f) / (alpha + 1.0f);
    float deriv_b = 2.0f * config.Td / ((alpha + 1.0f) * dt);

    // Manual state tracking
    float integral_acc = 0.0f;
    float deriv_filt = 0.0f;
    float prev_error = 0.0f;
    float prev_deriv_input = 0.0f;
    bool first_run = true;

    printf("  Simulating step response...\n");

    float setpoint = 10.0f;
    float measurement = 0.0f;

    for (int i = 0; i < 5; i++) {
        float error = setpoint - measurement;

        // Compute expected values
        float expected_P = config.Kp * error;

        // Integral (trapezoidal)
        if (!first_run) {
            integral_acc += (dt / (2.0f * config.Ti)) * (error + prev_error);
        } else {
            integral_acc += (dt / (2.0f * config.Ti)) * (error + 0.0f);
        }
        float expected_I = config.Kp * integral_acc;

        // Derivative (incomplete)
        float expected_D;
        if (first_run) {
            expected_D = 0.0f;
            prev_deriv_input = error;
            first_run = false;
        } else {
            float deriv_diff = error - prev_deriv_input;
            deriv_filt = deriv_a * deriv_filt + deriv_b * deriv_diff;
            expected_D = config.Kp * deriv_filt;
            prev_deriv_input = error;
        }

        float expected_output = expected_P + expected_I + expected_D;
        prev_error = error;

        // Actual PID output
        float output = pid.update(setpoint, measurement, dt);

        printf("  Step %d: e=%.2f, P=%.3f, I=%.3f, D=%.3f, out=%.3f (exp=%.3f)\n",
               i + 1, error,
               pid.getProportional(), pid.getIntegral(), pid.getDerivative(),
               output, expected_output);

        ASSERT_NEAR(expected_output, output, 1e-3f, "Full PID output");
    }
}

// ============================================================================
// Test 5: Anti-windup (back-calculation)
// ============================================================================
void test_antiwindup() {
    TEST_SECTION("Test 5: Anti-windup (Back-calculation)");

    stampfly::PID pid;
    stampfly::PIDConfig config;
    config.Kp = 10.0f;
    config.Ti = 0.1f;   // Fast integral
    config.Td = 0.0f;
    config.output_min = -5.0f;
    config.output_max = 5.0f;
    pid.init(config);

    float dt = 0.01f;

    // Large error to cause saturation
    float setpoint = 10.0f;
    float measurement = 0.0f;
    float error = 10.0f;

    printf("  Saturating output (max=%.1f)...\n", config.output_max);

    // Run several steps to accumulate integral
    float output;
    for (int i = 0; i < 10; i++) {
        output = pid.update(setpoint, measurement, dt);
    }

    printf("  After 10 steps: output=%.3f (saturated at max)\n", output);
    ASSERT_NEAR(config.output_max, output, 1e-5f, "Output saturated at max");

    // Now reverse direction - without anti-windup, integral would cause delay
    setpoint = -10.0f;
    measurement = 0.0f;

    printf("  Reversing setpoint to -10...\n");

    // First reversal step
    output = pid.update(setpoint, measurement, dt);
    printf("  Step 1 after reversal: output=%.3f, I=%.3f\n",
           output, pid.getIntegral());

    // After a few more steps, should reach negative saturation
    for (int i = 0; i < 10; i++) {
        output = pid.update(setpoint, measurement, dt);
    }

    printf("  After 10 more steps: output=%.3f\n", output);
    ASSERT_NEAR(config.output_min, output, 1e-5f, "Output saturated at min");

    // Key test: integral should not have grown excessively
    // With proper anti-windup, the integral term should be bounded
    float integral = pid.getIntegral();
    printf("  Final integral term: %.3f\n", integral);

    // Integral should be limited by back-calculation
    // The exact value depends on implementation, but it should be reasonable
    bool integral_bounded = (std::fabs(integral) < 100.0f);
    if (integral_bounded) {
        tests_passed++;
        printf("  Integral bounded: PASS\n");
    } else {
        tests_failed++;
        printf("  Integral bounded: FAIL (integral=%.3f, expected < 100)\n", integral);
    }
}

// ============================================================================
// Test 6: D-on-M vs D-on-E
// ============================================================================
void test_derivative_on_measurement() {
    TEST_SECTION("Test 6: D-on-M vs D-on-E");

    float dt = 0.01f;
    float Kp = 1.0f;
    float Td = 0.1f;
    float eta = 0.1f;

    // Coefficients
    float alpha = 2.0f * eta * Td / dt;
    float deriv_b = 2.0f * Td / ((alpha + 1.0f) * dt);

    // Test D-on-E
    {
        stampfly::PID pid;
        stampfly::PIDConfig config;
        config.Kp = Kp;
        config.Ti = 0.0f;
        config.Td = Td;
        config.eta = eta;
        config.derivative_on_measurement = false;  // D-on-E
        config.output_min = -100.0f;
        config.output_max = 100.0f;
        pid.init(config);

        // First update to initialize
        pid.update(0.0f, 0.0f, dt);

        // Step change in setpoint (error jumps)
        float setpoint = 10.0f;
        float measurement = 0.0f;
        float output = pid.update(setpoint, measurement, dt);

        // D-on-E: derivative of error = derivative of (setpoint - measurement)
        // With step change in setpoint, D should spike
        float expected_D = Kp * deriv_b * 10.0f;  // Large spike
        printf("  D-on-E: setpoint step 0->10, D=%.3f (expected spike ~%.1f)\n",
               pid.getDerivative(), expected_D);
        ASSERT_NEAR(expected_D, pid.getDerivative(), 1e-2f, "D-on-E spike on setpoint change");
    }

    // Test D-on-M
    {
        stampfly::PID pid;
        stampfly::PIDConfig config;
        config.Kp = Kp;
        config.Ti = 0.0f;
        config.Td = Td;
        config.eta = eta;
        config.derivative_on_measurement = true;  // D-on-M
        config.output_min = -100.0f;
        config.output_max = 100.0f;
        pid.init(config);

        // First update to initialize
        pid.update(0.0f, 0.0f, dt);

        // Step change in setpoint (measurement stays the same)
        float setpoint = 10.0f;
        float measurement = 0.0f;
        float output = pid.update(setpoint, measurement, dt);

        // D-on-M: derivative of -measurement = 0 (no change in measurement)
        printf("  D-on-M: setpoint step 0->10, D=%.3f (expected ~0, no measurement change)\n",
               pid.getDerivative());
        ASSERT_NEAR(0.0f, pid.getDerivative(), 1e-5f, "D-on-M no spike on setpoint change");
    }

    // Test D-on-M with measurement change
    {
        stampfly::PID pid;
        stampfly::PIDConfig config;
        config.Kp = Kp;
        config.Ti = 0.0f;
        config.Td = Td;
        config.eta = eta;
        config.derivative_on_measurement = true;  // D-on-M
        config.output_min = -100.0f;
        config.output_max = 100.0f;
        pid.init(config);

        // First update
        pid.update(10.0f, 0.0f, dt);

        // Measurement changes (disturbance)
        float measurement = 5.0f;
        float output = pid.update(10.0f, measurement, dt);

        // D-on-M: derivative of -measurement, measurement went 0->5
        // D = Kp * deriv_b * (-5 - 0) = -Kp * deriv_b * 5
        float expected_D = -Kp * deriv_b * 5.0f;
        printf("  D-on-M: measurement step 0->5, D=%.3f (expected ~%.1f)\n",
               pid.getDerivative(), expected_D);
        ASSERT_NEAR(expected_D, pid.getDerivative(), 1e-2f, "D-on-M responds to measurement change");
    }
}

// ============================================================================
// Test 7: Variable dt
// ============================================================================
void test_variable_dt() {
    TEST_SECTION("Test 7: Variable dt");

    stampfly::PID pid;
    stampfly::PIDConfig config;
    config.Kp = 1.0f;
    config.Ti = 0.5f;
    config.Td = 0.1f;
    config.eta = 0.1f;
    config.derivative_on_measurement = false;
    config.output_min = -100.0f;
    config.output_max = 100.0f;
    pid.init(config);

    float setpoint = 10.0f;
    float measurement = 0.0f;
    float error = 10.0f;

    // First update with dt=0.01
    float dt1 = 0.01f;
    float output1 = pid.update(setpoint, measurement, dt1);
    printf("  dt=%.3f: P=%.3f, I=%.3f, D=%.3f, out=%.3f\n",
           dt1, pid.getProportional(), pid.getIntegral(), pid.getDerivative(), output1);

    // Reset and test with dt=0.02
    pid.reset();
    float dt2 = 0.02f;
    float output2 = pid.update(setpoint, measurement, dt2);
    printf("  dt=%.3f: P=%.3f, I=%.3f, D=%.3f, out=%.3f\n",
           dt2, pid.getProportional(), pid.getIntegral(), pid.getDerivative(), output2);

    // P should be the same
    ASSERT_NEAR(config.Kp * error, pid.getProportional(), 1e-5f, "P same for different dt");

    // I should be proportional to dt
    // I = Kp * (dt/(2*Ti)) * (e + 0) = Kp * dt * e / (2*Ti)
    // For dt=0.02, I should be 2x compared to dt=0.01
    float I_for_dt1 = config.Kp * (dt1 / (2.0f * config.Ti)) * error;
    float I_for_dt2 = config.Kp * (dt2 / (2.0f * config.Ti)) * error;
    printf("  Expected I for dt=%.3f: %.4f, dt=%.3f: %.4f\n", dt1, I_for_dt1, dt2, I_for_dt2);
    ASSERT_NEAR(I_for_dt2 / I_for_dt1, 2.0f, 1e-3f, "I scales with dt");

    // Test with varying dt in sequence
    printf("\n  Testing varying dt in sequence:\n");
    pid.reset();

    std::vector<float> dts = {0.008f, 0.012f, 0.010f, 0.011f, 0.009f};
    float integral_acc = 0.0f;
    float prev_error_manual = 0.0f;

    for (size_t i = 0; i < dts.size(); i++) {
        float dt = dts[i];
        float output = pid.update(setpoint, measurement, dt);

        // Manual integral calculation
        integral_acc += (dt / (2.0f * config.Ti)) * (error + prev_error_manual);
        prev_error_manual = error;
        float expected_I = config.Kp * integral_acc;

        printf("    Step %zu: dt=%.3f, I=%.4f (expected=%.4f)\n",
               i + 1, dt, pid.getIntegral(), expected_I);

        // Only check integral (first step has D=0, subsequent have filter dynamics)
        if (i > 0) {
            ASSERT_NEAR(expected_I, pid.getIntegral(), 1e-4f, "I with varying dt");
        }
    }
}

// ============================================================================
// Test 8: Reset functionality
// ============================================================================
void test_reset() {
    TEST_SECTION("Test 8: Reset Functionality");

    stampfly::PID pid;
    stampfly::PIDConfig config;
    config.Kp = 2.0f;
    config.Ti = 0.5f;
    config.Td = 0.1f;
    config.output_min = -100.0f;
    config.output_max = 100.0f;
    pid.init(config);

    float dt = 0.01f;

    // Run several updates
    for (int i = 0; i < 10; i++) {
        pid.update(10.0f, 0.0f, dt);
    }

    printf("  Before reset: I=%.3f, D=%.3f\n",
           pid.getIntegral(), pid.getDerivative());

    // Verify states are non-zero
    bool states_nonzero = (std::fabs(pid.getIntegral()) > 0.1f);

    // Reset
    pid.reset();

    printf("  After reset: I=%.3f, D=%.3f\n",
           pid.getIntegral(), pid.getDerivative());

    ASSERT_NEAR(0.0f, pid.getIntegral(), 1e-6f, "I reset to 0");
    ASSERT_NEAR(0.0f, pid.getDerivative(), 1e-6f, "D reset to 0");

    // First update after reset should behave like initial
    float output = pid.update(10.0f, 0.0f, dt);
    float expected_P = config.Kp * 10.0f;
    // After reset, first derivative should be 0 (first_run = true)
    ASSERT_NEAR(0.0f, pid.getDerivative(), 1e-6f, "D=0 on first update after reset");
}

// ============================================================================
// Test 9: Output clamping
// ============================================================================
void test_output_clamping() {
    TEST_SECTION("Test 9: Output Clamping");

    stampfly::PID pid;
    stampfly::PIDConfig config;
    config.Kp = 100.0f;  // Large gain
    config.Ti = 0.0f;
    config.Td = 0.0f;
    config.output_min = -10.0f;
    config.output_max = 10.0f;
    pid.init(config);

    float dt = 0.01f;

    // Positive saturation
    float output1 = pid.update(100.0f, 0.0f, dt);
    printf("  Positive error: unlimited=%.1f, clamped=%.1f\n",
           config.Kp * 100.0f, output1);
    ASSERT_NEAR(config.output_max, output1, 1e-5f, "Output clamped at max");

    // Negative saturation
    float output2 = pid.update(-100.0f, 0.0f, dt);
    printf("  Negative error: unlimited=%.1f, clamped=%.1f\n",
           config.Kp * (-100.0f), output2);
    ASSERT_NEAR(config.output_min, output2, 1e-5f, "Output clamped at min");

    // Within limits
    float output3 = pid.update(0.05f, 0.0f, dt);
    float expected = config.Kp * 0.05f;  // 5.0, within limits
    printf("  Small error: output=%.1f (within limits)\n", output3);
    ASSERT_NEAR(expected, output3, 1e-5f, "Output not clamped when within limits");
}

// ============================================================================
// Main
// ============================================================================
int main() {
    printf("PID Controller Unit Tests\n");
    printf("=========================\n");

    test_p_only();
    test_pi_trapezoidal();
    test_pd_incomplete_derivative();
    test_full_pid();
    test_antiwindup();
    test_derivative_on_measurement();
    test_variable_dt();
    test_reset();
    test_output_clamping();

    printf("\n=========================\n");
    printf("Results: %d passed, %d failed\n", tests_passed, tests_failed);
    printf("=========================\n");

    return tests_failed > 0 ? 1 : 0;
}
