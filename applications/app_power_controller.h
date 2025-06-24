/*
 * VESC Power Controller Application Header
 * 
 * Core API for voltage monitoring and regenerative braking control
 * for hybrid drone applications targeting 48V DC bus regulation.
 */

#ifndef APP_POWER_CONTROLLER_H_
#define APP_POWER_CONTROLLER_H_

#include <stdbool.h>
#include "app_power_controller_config.h"
#include "datatypes.h"

// Core Application Interface (always available)
void app_power_controller_start(void);
void app_power_controller_stop(void);
void app_power_controller_configure(app_configuration *conf);

// Runtime Monitoring Functions
float app_power_controller_get_voltage(void);
float app_power_controller_get_regen_current(void);  
bool app_power_controller_is_regen_active(void);

// Configuration Constants (read-only access)
extern const float POWER_CONTROLLER_TARGET_VOLTAGE;
extern const float POWER_CONTROLLER_THRESHOLD_VOLTAGE;
extern const float POWER_CONTROLLER_MAX_CURRENT;
extern const float POWER_CONTROLLER_MIN_VOLTAGE;

// Conditional Test Interface (only in debug builds)
#if PC_ENABLE_TESTS
#include "app_power_controller_test.h"
#endif

#endif /* APP_POWER_CONTROLLER_H_ */
