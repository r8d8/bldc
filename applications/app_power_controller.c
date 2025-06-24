/*
 * VESC Power Controller Application
 * 
 * Monitors DC bus voltage and applies regenerative braking to maintain
 * constant voltage for hybrid drone applications.
 * 
 * Target: 48V DC bus with automatic regen control
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
#define MIN_VOLTAGE          36.0

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

static pid_state_t pid_state = {0.0, 0.0};

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
		// Get current voltage
		float voltage_now = mc_interface_get_input_voltage_filtered();

		// Use PID controller to calculate regenerative current
		float current = pid_update(TARGET_DC_VOLTAGE, voltage_now);

		// Safety check - don't apply regen if voltage is too low
		if (voltage_now < MIN_VOLTAGE) {
			current = 0.0;
			// Reset PID when in safety zone
			pid_state.integral = 0.0;
			pid_state.previous_error = 0.0;
		}

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
