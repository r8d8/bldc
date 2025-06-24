/*
 * VESC Power Controller Build Configuration
 * 
 * Controls compilation of test functionality and debug features
 */

#ifndef APP_POWER_CONTROLLER_CONFIG_H_
#define APP_POWER_CONTROLLER_CONFIG_H_

// Test compilation control
#ifdef DEBUG_BUILD
    #define PC_ENABLE_TESTS           1    // Enable test functions
    #define PC_ENABLE_DEBUG_OUTPUT    1    // Enable debug logging
    #define PC_ENABLE_MOCK_INTERFACE  1    // Enable mock VESC interface for unit tests
#else
    #define PC_ENABLE_TESTS           0    // Disable tests in production
    #define PC_ENABLE_DEBUG_OUTPUT    0    // Disable debug output
    #define PC_ENABLE_MOCK_INTERFACE  0    // Disable mock interface
#endif

// You can also manually enable tests by uncommenting this:
// #define DEBUG_BUILD
// This will automatically enable tests, debug output, and mock interface

// Test configuration
#if PC_ENABLE_TESTS
    #define PC_TEST_MAX_DATA_POINTS   2000
    #define PC_TEST_DEFAULT_DURATION  10.0   // seconds
    #define PC_TEST_TIMESTEP          0.01   // 10ms for smooth testing
#endif

#endif /* APP_POWER_CONTROLLER_CONFIG_H_ */
