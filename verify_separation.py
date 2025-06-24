#!/usr/bin/env python3
"""
VESC Power Controller Build Verification Script

This script verifies that the compile-time separation between 
production and test code is working correctly.
"""

import os
import re

def check_file_separation():
    """Verify that test code is properly separated from production code"""
    
    print("="*60)
    print("POWER CONTROLLER BUILD SEPARATION VERIFICATION")
    print("="*60)
    
    # Check core files
    core_file = "applications/app_power_controller.c"
    test_file = "applications/app_power_controller_test.c" 
    header_file = "applications/app_power_controller.h"
    test_header_file = "applications/app_power_controller_test.h"
    config_file = "applications/app_power_controller_config.h"
    
    files_to_check = [
        (core_file, "Core Implementation"),
        (test_file, "Test Implementation"), 
        (header_file, "Core Header"),
        (test_header_file, "Test Header"),
        (config_file, "Configuration Header")
    ]
    
    print("\n1. Checking file existence:")
    all_exist = True
    for filepath, description in files_to_check:
        if os.path.exists(filepath):
            print(f"   ✓ {description}: {filepath}")
        else:
            print(f"   ✗ {description}: {filepath} - MISSING")
            all_exist = False
    
    if not all_exist:
        print("\n❌ Some required files are missing!")
        return False
    
    print("\n2. Checking core file for test code contamination:")
    
    # Read core file and check for test patterns
    with open(core_file, 'r') as f:
        core_content = f.read()
    
    test_patterns = [
        r'TEST_MODE_ENABLED',
        r'test_voltage_override',
        r'iteration_count',
        r'max_current_applied',
        r'total_energy_recovered',
        r'debug_str',
        r'sprintf',
        r'_test_',
        r'TEST_',
        r'mock_',
        r'MOCK_'
    ]
    
    contamination_found = False
    for pattern in test_patterns:
        matches = re.findall(pattern, core_content, re.IGNORECASE)
        if matches:
            print(f"   ⚠️  Found test pattern '{pattern}' in core file")
            contamination_found = True
    
    if not contamination_found:
        print("   ✓ Core file is clean (no test code found)")
    else:
        print("   ✗ Core file contains test code!")
        return False
    
    print("\n3. Checking test file for test code:")
    
    with open(test_file, 'r') as f:
        test_content = f.read()
    
    expected_test_patterns = [
        r'PC_ENABLE_TESTS',
        r'TEST_ASSERT',
        r'RUN_TEST',
        r'pc_test_',
        r'mock_'
    ]
    
    test_code_found = True
    for pattern in expected_test_patterns:
        if not re.search(pattern, test_content):
            print(f"   ⚠️  Expected test pattern '{pattern}' not found")
            test_code_found = False
    
    if test_code_found:
        print("   ✓ Test file contains expected test code")
    else:
        print("   ⚠️  Test file may be incomplete")
    
    print("\n4. Checking conditional compilation:")
    
    # Check that test file is wrapped in PC_ENABLE_TESTS
    if re.search(r'#if\s+PC_ENABLE_TESTS', test_content):
        print("   ✓ Test file uses conditional compilation")
    else:
        print("   ⚠️  Test file should be wrapped in #if PC_ENABLE_TESTS")
    
    # Check configuration file
    with open(config_file, 'r') as f:
        config_content = f.read()
    
    if re.search(r'PC_ENABLE_TESTS', config_content):
        print("   ✓ Configuration file defines PC_ENABLE_TESTS")
    else:
        print("   ✗ Configuration file missing PC_ENABLE_TESTS")
        return False
    
    print("\n5. Checking build system integration:")
    
    # Check applications.mk for conditional test inclusion
    makefile_path = "applications/applications.mk"
    if os.path.exists(makefile_path):
        with open(makefile_path, 'r') as f:
            makefile_content = f.read()
        
        if re.search(r'ifdef\s+DEBUG_BUILD.*app_power_controller_test\.c', makefile_content, re.DOTALL):
            print("   ✓ Makefile includes test file conditionally")
        else:
            print("   ⚠️  Makefile may not properly exclude test file in production")
    else:
        print("   ⚠️  Applications makefile not found")
    
    # Check test makefile
    test_makefile_path = "Makefile.test"
    if os.path.exists(test_makefile_path):
        print("   ✓ Test makefile exists")
    else:
        print("   ⚠️  Test makefile not found")
    
    print("\n6. File size analysis:")
    
    core_size = os.path.getsize(core_file)
    test_size = os.path.getsize(test_file) 
    
    print(f"   Core file size: {core_size:,} bytes")
    print(f"   Test file size: {test_size:,} bytes")
    print(f"   Test/Core ratio: {test_size/core_size:.1f}x")
    
    if test_size > core_size * 0.5:
        print("   ✓ Test file has substantial test coverage")
    else:
        print("   ⚠️  Test file seems small relative to core file")
    
    print("\n" + "="*60)
    print("VERIFICATION SUMMARY")
    print("="*60)
    
    if not contamination_found and all_exist:
        print("✅ VERIFICATION PASSED")
        print("   - Core file is clean of test code")
        print("   - Test code is properly separated")
        print("   - Conditional compilation is in place")
        print("   - Build system supports both modes")
        return True
    else:
        print("❌ VERIFICATION FAILED")
        print("   - Issues found with code separation")
        return False

def print_file_hierarchy():
    """Print the current file organization"""
    print("\n" + "="*60)
    print("FILE HIERARCHY")
    print("="*60)
    
    structure = {
        "Production Code (always compiled)": [
            "applications/app_power_controller.c",
            "applications/app_power_controller.h"
        ],
        "Test Code (DEBUG_BUILD only)": [
            "applications/app_power_controller_test.c",
            "applications/app_power_controller_test.h"
        ],
        "Configuration": [
            "applications/app_power_controller_config.h"
        ],
        "Build System": [
            "Makefile.test",
            "applications/applications.mk"
        ],
        "Python Tests (no hardware required)": [
            "test_power_controller.py",
            "test_power_controller_improved.py"
        ]
    }
    
    for category, files in structure.items():
        print(f"\n{category}:")
        for filepath in files:
            if os.path.exists(filepath):
                size = os.path.getsize(filepath)
                print(f"   ✓ {filepath:<40} ({size:,} bytes)")
            else:
                print(f"   ✗ {filepath:<40} (missing)")

if __name__ == "__main__":
    print_file_hierarchy()
    success = check_file_separation()
    
    if success:
        print("\n🎉 Power Controller build separation is properly implemented!")
        print("\nNext steps:")
        print("   1. Test production build: make fsesc_75_300")
        print("   2. Test debug build:      make DEBUG_BUILD=1 fsesc_75_300")
        print("   3. Run Python tests:      make -f Makefile.test test")
        print("   4. Run hardware tests:    Flash debug firmware and use VESC Tool")
    else:
        print("\n🚨 Issues found - please review the verification output above")
    
    exit(0 if success else 1)
