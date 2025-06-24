/*
 * VESC Power Controller Test Interface
 * 
 * Unit tests and integration tests for power controller validation.
 * Only compiled when PC_ENABLE_TESTS is defined.
 */

#ifndef APP_POWER_CONTROLLER_TEST_H_
#define APP_POWER_CONTROLLER_TEST_H_

#include <stdbool.h>
#include <stdint.h>

// Only compile test interface if tests are enabled
#if PC_ENABLE_TESTS

// Test Data Structure
typedef struct {
    uint32_t iteration_count;
    float max_current_applied;
    float total_energy_recovered;
    float pid_integral;
    float last_error;
    bool regen_active;
    float current_voltage;
    float current_output;
    float test_start_time;
    uint32_t test_failures;
    uint32_t test_passes;
} power_controller_test_stats_t;

// =============================================================================
// UNIT TESTS (Test individual functions in isolation)
// =============================================================================

// PID Controller Unit Tests
bool pc_test_pid_proportional_response(void);
bool pc_test_pid_integral_buildup(void);
bool pc_test_pid_derivative_damping(void);
bool pc_test_pid_anti_windup(void);
bool pc_test_pid_reset_behavior(void);

// Voltage Threshold Unit Tests  
bool pc_test_voltage_threshold_detection(void);
bool pc_test_voltage_boundary_conditions(void);
bool pc_test_safety_voltage_cutoff(void);

// Current Limiting Unit Tests
bool pc_test_current_output_limits(void);
bool pc_test_current_scaling(void);

// =============================================================================
// INTEGRATION TESTS (Test complete system behavior)
// =============================================================================

// System Response Integration Tests
bool pc_test_voltage_drop_scenario(void);
bool pc_test_voltage_recovery_scenario(void);
bool pc_test_oscillation_damping(void);
bool pc_test_steady_state_regulation(void);

// Performance Integration Tests
bool pc_test_response_time(void);
bool pc_test_stability_margins(void);
bool pc_test_energy_recovery_efficiency(void);

// Stress Integration Tests
bool pc_test_rapid_voltage_changes(void);
bool pc_test_long_duration_operation(void);
bool pc_test_extreme_voltage_conditions(void);

// =============================================================================
// TEST UTILITIES AND CONTROL
// =============================================================================

// Test Execution Control
void pc_test_run_all_unit_tests(void);
void pc_test_run_all_integration_tests(void);
void pc_test_run_complete_suite(void);

// Test Environment Control
void pc_test_set_voltage_override(float voltage);
void pc_test_reset_voltage_override(void);
void pc_test_reset_all_stats(void);
void pc_test_reset_pid_state(void);

// Test Data Access
power_controller_test_stats_t pc_test_get_stats(void);
void pc_test_print_stats(void);
void pc_test_print_results_summary(void);

// PID Tuning for Testing
void pc_test_set_pid_gains(float kp, float ki, float kd);
void pc_test_get_pid_gains(float *kp, float *ki, float *kd);
void pc_test_restore_default_pid_gains(void);

// =============================================================================
// MOCK INTERFACE (For isolated unit testing)
// =============================================================================

#if PC_ENABLE_MOCK_INTERFACE

// Mock VESC Interface Functions
void pc_mock_set_input_voltage(float voltage);
void pc_mock_set_current_response(float current);
float pc_mock_get_applied_current(void);
void pc_mock_reset_all(void);

// Mock State Queries
bool pc_mock_was_current_applied(void);
bool pc_mock_was_motor_released(void);
uint32_t pc_mock_get_set_current_call_count(void);

#endif // PC_ENABLE_MOCK_INTERFACE

// =============================================================================
// TEST CONFIGURATION AND CONSTANTS
// =============================================================================

// Test Configuration
#define PC_TEST_VOLTAGE_TOLERANCE       0.05f   // ±50mV tolerance
#define PC_TEST_CURRENT_TOLERANCE       0.1f    // ±100mA tolerance
#define PC_TEST_TIME_TOLERANCE          0.01f   // ±10ms tolerance
#define PC_TEST_MAX_ITERATIONS          10000   // Max test iterations
#define PC_TEST_DEFAULT_TIMEOUT         30.0f   // 30 second timeout

// Test Voltage Levels for Validation
#define PC_TEST_VOLTAGE_NORMAL          48.0f
#define PC_TEST_VOLTAGE_THRESHOLD       47.5f
#define PC_TEST_VOLTAGE_ACTIVE_HIGH     47.0f
#define PC_TEST_VOLTAGE_ACTIVE_MID      46.0f
#define PC_TEST_VOLTAGE_ACTIVE_LOW      45.5f
#define PC_TEST_VOLTAGE_MINIMUM         45.0f
#define PC_TEST_VOLTAGE_BELOW_MIN       44.0f

// Legacy Function Names (for backward compatibility)
#define app_power_controller_run_tests()                pc_test_run_complete_suite()
#define app_power_controller_test_get_stats()           pc_test_print_stats()
#define app_power_controller_test_reset_stats()         pc_test_reset_all_stats()
#define app_power_controller_test_set_voltage(v)        pc_test_set_voltage_override(v)
#define app_power_controller_test_reset_voltage()       pc_test_reset_voltage_override()

#endif // PC_ENABLE_TESTS

#endif /* APP_POWER_CONTROLLER_TEST_H_ */
