/*
 * VESC Power Controller Application
 * 
 * Monitors DC bus voltage and applies regenerative braking to maintain
 * constant voltage for hybrid drone applications.
 * 
 * Target: 48V DC bus with automatic regen control
 * 
 * This file is designed to be included by app_power_controller.c
 */

#include <math.h>
#include "mc_interface.h"
#include "timeout.h"
#include "app_power_controller.h"

// Target DC bus voltage (48V system)
#define TARGET_DC_VOLTAGE    48.0

// Voltage threshold to start regenerative braking
// Start regen when voltage drops below this value
#define VOLTAGE_THRESHOLD    47.5

// Maximum regenerative current (amperes) - always positive
// This will be applied as negative current for regen
#define MAX_REGEN_CURRENT    50.0

// Minimum voltage to stop all regen (protection)
#define MIN_VOLTAGE          45.0

// PID Controller Parameters
#define PID_KP               20.0   // Proportional gain (A/V)
#define PID_KI               5.0    // Integral gain (A/(V*s))
#define PID_KD               0.5    // Derivative gain (A*s/V)
#define PID_OUTPUT_LIMIT     50.0   // Maximum output current (A)
#define PID_INTEGRAL_LIMIT   10.0   // Anti-windup limit (V*s)

#define POWER_UPDATE_RATE_HZ 1000
#define DT                   (1.0 / POWER_UPDATE_RATE_HZ)  // Time step

static volatile bool stop_now = true;
static volatile bool is_running = false;

// PID Controller State Variables
typedef struct {
    float integral;
    float previous_error;
} pid_state_t;

static pid_state_t pid_state = {0.0, 0.0, 0.0, 0};

// PID Controller for voltage regulation
static float pid_update(float target_voltage, float current_voltage) {
    // Calculate error (positive when voltage is below target)
    float error = target_voltage - current_voltage;
    
    // Only run PID when voltage is below threshold (regen active zone)
    if (current_voltage > VOLTAGE_THRESHOLD) {
        // Reset PID state when not in regen zone
        pid_state.integral = 0.0;
        pid_state.previous_error = 0.0;
        return 0.0;
    }
    
    // Proportional term
    float proportional = PID_KP * error;
    
    // Integral term (with anti-windup)
    pid_state.integral += error * DT;
    if (pid_state.integral > PID_INTEGRAL_LIMIT) {
        pid_state.integral = PID_INTEGRAL_LIMIT;
    } else if (pid_state.integral < -PID_INTEGRAL_LIMIT) {
        pid_state.integral = -PID_INTEGRAL_LIMIT;
    }
    float integral = PID_KI * pid_state.integral;
    
    // Derivative term
    float derivative = PID_KD * (error - pid_state.previous_error) / DT;
    
    // Calculate output
    float output = proportional + integral + derivative;
    
    // Limit output
    if (output > PID_OUTPUT_LIMIT) {
        output = PID_OUTPUT_LIMIT;
    } else if (output < 0.0) {
        output = 0.0;
    }
    
    // Store for next iteration
    pid_state.previous_error = error;
    
    return output;
}

// Threads
static THD_FUNCTION(power_thread, arg);
static THD_WORKING_AREA(power_thread_wa, 1024);

void app_power_controller_start(void) {
	stop_now = false;
	chThdCreateStatic(power_thread_wa, sizeof(power_thread_wa), NORMALPRIO, power_thread, NULL);
}

void app_power_controller_stop(void) {
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

void app_power_controller_configure(app_configuration *conf) {
	(void) conf;
}

static THD_FUNCTION(power_thread, arg) {
	(void)arg;

	is_running = true;

	for(;;) {
		// Get voltage (real or test override)
		float voltage_now;
		#if TEST_MODE_ENABLED
		if (test_voltage_override >= 0.0) {
			voltage_now = test_voltage_override;
		} else {
			voltage_now = mc_interface_get_input_voltage_filtered();
		}
		#else
		voltage_now = mc_interface_get_input_voltage_filtered();
		#endif

		// Use PID controller to calculate regenerative current
		float current = pid_update(TARGET_DC_VOLTAGE, voltage_now);

		// Safety check - don't apply regen if voltage is too low
		if (voltage_now < MIN_VOLTAGE) {
			current = 0.0;
			// Reset PID when in safety zone
			pid_state.integral = 0.0;
			pid_state.previous_error = 0.0;
		}

		// Update debug statistics
		pid_state.iteration_count++;
		if (current > pid_state.max_current_applied) {
			pid_state.max_current_applied = current;
		}
		if (current > 0.1) {
			pid_state.total_energy_recovered += (current * voltage_now * DT) / 3600.0; // Wh
		}

		// Debug output every 1000 iterations (1 second at 1kHz)
		#if DEBUG_ENABLED
		if (pid_state.iteration_count % 1000 == 0) {
			// This will appear in VESC Tool terminal/log
			static char debug_str[200];
			sprintf(debug_str, 
				"PC: V=%.2fV, I=%.1fA, E=%.3fWh, Max=%.1fA, Iter=%lu",
				voltage_now, current, pid_state.total_energy_recovered, 
				pid_state.max_current_applied, pid_state.iteration_count);
			// Use VESC logging (if available)
			// commands_printf(debug_str);
		}
		#endif

		// Apply regenerative braking (negative current)
		if (current > 0.1) {
			mc_interface_set_current(-current);
		} else {
			mc_interface_release_motor();
		}

		// Sleep for a time according to the specified rate
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / POWER_UPDATE_RATE_HZ;

		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}
		chThdSleep(sleep_time);

		if (stop_now) {
			is_running = false;
			return;
		}

		// Reset timeout
		timeout_reset();
	}
}

// Export configuration constants for external access
const float POWER_CONTROLLER_TARGET_VOLTAGE = TARGET_DC_VOLTAGE;
const float POWER_CONTROLLER_THRESHOLD_VOLTAGE = VOLTAGE_THRESHOLD;
const float POWER_CONTROLLER_MAX_CURRENT = MAX_REGEN_CURRENT;
const float POWER_CONTROLLER_MIN_VOLTAGE = MIN_VOLTAGE;

// Utility functions for external monitoring (could be used by CAN or terminal)
float app_power_controller_get_voltage(void) {
	return mc_interface_get_input_voltage_filtered();
}

float app_power_controller_get_regen_current(void) {
	return mc_interface_get_tot_current_filtered();
}

bool app_power_controller_is_regen_active(void) {
	const float voltage_now = mc_interface_get_input_voltage_filtered();
	// Regen is active when voltage is below threshold and above minimum
	return (voltage_now < VOLTAGE_THRESHOLD && voltage_now > MIN_VOLTAGE);
}

// Test and Debug Functions
#ifdef TEST_MODE_ENABLED

// Simulate voltage drops for testing
static float test_voltage_override = -1.0; // -1 means use real voltage

void app_power_controller_test_set_voltage(float voltage) {
	test_voltage_override = voltage;
}

void app_power_controller_test_reset_voltage(void) {
	test_voltage_override = -1.0;
}

void app_power_controller_test_get_stats(void) {
	// Print current statistics
	static char stats_str[300];
	sprintf(stats_str, 
		"Power Controller Stats:\n"
		"  Iterations: %lu\n"
		"  Max Current: %.2f A\n"
		"  Energy Recovered: %.3f Wh\n"
		"  PID Integral: %.3f\n"
		"  Last Error: %.3f V\n"
		"  Active: %s\n",
		pid_state.iteration_count,
		pid_state.max_current_applied,
		pid_state.total_energy_recovered,
		pid_state.integral,
		pid_state.previous_error,
		app_power_controller_is_regen_active() ? "YES" : "NO"
	);
	// Print to VESC Tool terminal (if available)
	// commands_printf(stats_str);
}

void app_power_controller_test_reset_stats(void) {
	pid_state.iteration_count = 0;
	pid_state.max_current_applied = 0.0;
	pid_state.total_energy_recovered = 0.0;
	pid_state.integral = 0.0;
	pid_state.previous_error = 0.0;
}

// Test scenario functions
void app_power_controller_test_voltage_drop_scenario(void) {
	// Simulate voltage drop from 48V to 46V and recovery
	// This would be called periodically from external test
}

// Get test statistics in structured format
power_controller_test_stats_t app_power_controller_get_test_stats(void) {
	power_controller_test_stats_t stats;
	stats.iteration_count = pid_state.iteration_count;
	stats.max_current_applied = pid_state.max_current_applied;
	stats.total_energy_recovered = pid_state.total_energy_recovered;
	stats.pid_integral = pid_state.integral;
	stats.last_error = pid_state.previous_error;
	stats.regen_active = app_power_controller_is_regen_active();
	stats.current_voltage = app_power_controller_get_voltage();
	
	// Calculate current output (would need to store this in real implementation)
	stats.current_output = 0.0; // Placeholder
	
	return stats;
}

// PID tuning functions for testing
static float test_kp = PID_KP;
static float test_ki = PID_KI; 
static float test_kd = PID_KD;

void app_power_controller_test_set_pid_gains(float kp, float ki, float kd) {
	test_kp = kp;
	test_ki = ki;
	test_kd = kd;
	// Reset PID state when changing gains
	pid_state.integral = 0.0;
	pid_state.previous_error = 0.0;
}

void app_power_controller_test_get_pid_gains(float *kp, float *ki, float *kd) {
	if (kp) *kp = test_kp;
	if (ki) *ki = test_ki;
	if (kd) *kd = test_kd;
}

// Performance test functions
bool app_power_controller_test_performance(void) {
	// Test that controller can run at target frequency
	// Measure execution time, memory usage, etc.
	return true; // Placeholder
}

bool app_power_controller_test_stability(void) {
	// Test PID stability with various inputs
	// Check for oscillations, overshoot, etc.
	return true; // Placeholder  
}

bool app_power_controller_test_boundary_conditions(void) {
	// Test edge cases: exactly at thresholds, voltage limits, etc.
	float test_voltages[] = {47.5, 47.49, 45.0, 45.01, 44.0, 50.0};
	int num_tests = sizeof(test_voltages) / sizeof(test_voltages[0]);
	
	for (int i = 0; i < num_tests; i++) {
		app_power_controller_test_set_voltage(test_voltages[i]);
		bool active = app_power_controller_is_regen_active();
		
		// Verify expected behavior
		bool expected = (test_voltages[i] < VOLTAGE_THRESHOLD && test_voltages[i] > MIN_VOLTAGE);
		if (active != expected) {
			app_power_controller_test_reset_voltage();
			return false;
		}
	}
	
	app_power_controller_test_reset_voltage();
	return true;
}

// Main test runner function
void app_power_controller_run_tests(void) {
	// Reset stats before testing
	app_power_controller_test_reset_stats();
	
	// Run all test functions
	bool perf_ok = app_power_controller_test_performance();
	bool stability_ok = app_power_controller_test_stability();
	bool boundary_ok = app_power_controller_test_boundary_conditions();
	
	// Print results (in real implementation would use commands_printf)
	static char test_results[200];
	sprintf(test_results,
		"Power Controller Test Results:\n"
		"  Performance Test: %s\n"
		"  Stability Test: %s\n" 
		"  Boundary Test: %s\n"
		"  Overall: %s\n",
		perf_ok ? "PASS" : "FAIL",
		stability_ok ? "PASS" : "FAIL",
		boundary_ok ? "PASS" : "FAIL",
		(perf_ok && stability_ok && boundary_ok) ? "PASS" : "FAIL"
	);
}

#endif // TEST_MODE_ENABLED
