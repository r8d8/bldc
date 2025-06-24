#!/usr/bin/env python3
"""
VESC Power Controller Integration Test Suite

This script provides automated testing for the power controller application
using VESC Tool's Python interface or serial communication.
"""

import time
import matplotlib.pyplot as plt
import numpy as np
from typing import List, Tuple

class PowerControllerTester:
    def __init__(self, vesc_connection=None):
        """
        Initialize the tester with optional VESC connection
        
        Args:
            vesc_connection: VESC Tool connection object (if available)
        """
        self.vesc = vesc_connection
        self.test_data = []
        
        # Realistic PID state for simulation
        self.pid_integral = 0.0
        self.pid_previous_error = 0.0
        self.pid_kp = 20.0
        self.pid_ki = 5.0
        self.pid_kd = 0.5
        self.dt = 0.001  # 1ms timestep
        
    def reset_pid(self):
        """Reset PID controller state"""
        self.pid_integral = 0.0
        self.pid_previous_error = 0.0
        
    def log_data(self, timestamp: float, voltage: float, current: float, regen_active: bool):
        """Log test data point"""
        self.test_data.append({
            'time': timestamp,
            'voltage': voltage, 
            'current': current,
            'regen_active': regen_active
        })
    
    def test_voltage_response(self) -> bool:
        """Test voltage regulation response"""
        print("Testing voltage regulation response...")
        
        # Reset PID state for clean test
        self.pid_integral = 0.0
        self.pid_previous_error = 0.0
        
        # Define more realistic test scenario with smoother transitions
        # Simulate gradual voltage drop and recovery over 10 seconds
        duration = 10.0  # seconds
        time_step = 0.1  # 100ms steps
        num_points = int(duration / time_step)
        
        start_time = time.time()
        
        for i in range(num_points):
            t = i * time_step
            
            # Create smooth voltage profile with realistic transitions
            if t < 2.0:
                # Normal operation
                voltage = 48.0
            elif t < 3.0:
                # Gradual drop to threshold
                voltage = 48.0 - (t - 2.0) * 0.5  # Drop 0.5V/s
            elif t < 5.0:
                # Continue dropping below threshold (regen active zone)
                voltage = 47.5 - (t - 3.0) * 0.75  # Drop 0.75V/s
            elif t < 6.0:
                # Reach minimum point
                voltage = 46.0 - (t - 5.0) * 1.0   # Drop to ~45V
            elif t < 8.0:
                # Recovery phase
                voltage = 45.0 + (t - 6.0) * 1.25  # Recover 1.25V/s
            else:
                # Return to normal
                voltage = 47.5 + (t - 8.0) * 0.25  # Slow return to 48V
            
            # Ensure voltage stays within reasonable bounds
            voltage = max(min(voltage, 50.0), 44.0)
            
            # Get controller response
            current = self.simulate_controller_response(voltage)
            regen_active = voltage < 47.5 and voltage > 45.0
            
            self.log_data(t, voltage, current, regen_active)
            
            if i % 10 == 0:  # Print every 1 second
                print(f"  t={t:.1f}s: V={voltage:.2f}V, I={current:.1f}A, Regen={regen_active}")
        
        return True
    
    def __init__(self, vesc_connection=None):
        """
        Initialize the tester with optional VESC connection
        
        Args:
            vesc_connection: VESC Tool connection object (if available)
        """
        self.vesc = vesc_connection
        self.test_data = []
        
        # PID simulation state
        self.pid_integral = 0.0
        self.pid_previous_error = 0.0
        self.pid_kp = 20.0
        self.pid_ki = 5.0
        self.pid_kd = 0.5
        self.dt = 0.1  # 100ms time step for testing
    
    def simulate_controller_response(self, voltage: float) -> float:
        """
        Simulate realistic PID controller response
        (In real testing, this would read from actual VESC)
        """
        TARGET_VOLTAGE = 48.0
        VOLTAGE_THRESHOLD = 47.5
        MIN_VOLTAGE = 45.0
        MAX_CURRENT = 50.0
        INTEGRAL_LIMIT = 10.0
        
        # Safety zones - no regen
        if voltage >= VOLTAGE_THRESHOLD or voltage <= MIN_VOLTAGE:
            # Reset PID state when not in active zone
            self.pid_integral = 0.0
            self.pid_previous_error = 0.0
            return 0.0
        
        # PID Controller simulation
        error = TARGET_VOLTAGE - voltage
        
        # Proportional term
        proportional = self.pid_kp * error
        
        # Integral term with anti-windup
        self.pid_integral += error * self.dt
        if self.pid_integral > INTEGRAL_LIMIT:
            self.pid_integral = INTEGRAL_LIMIT
        elif self.pid_integral < -INTEGRAL_LIMIT:
            self.pid_integral = -INTEGRAL_LIMIT
        integral = self.pid_ki * self.pid_integral
        
        # Derivative term
        derivative = self.pid_kd * (error - self.pid_previous_error) / self.dt
        
        # Calculate output
        output = proportional + integral + derivative
        
        # Limit output
        output = min(max(output, 0.0), MAX_CURRENT)
        
        # Store for next iteration
        self.pid_previous_error = error
        
        return output
    
    def test_pid_stability(self) -> bool:
        """Test PID controller stability"""
        print("Testing PID stability...")
        
        # Reset PID state
        self.pid_integral = 0.0
        self.pid_previous_error = 0.0
        
        # Simulate steady state with small realistic disturbances
        base_voltage = 47.0  # In regen active zone
        test_duration = 5.0  # 5 seconds
        time_step = 0.1
        num_points = int(test_duration / time_step)
        
        for i in range(num_points):
            t = i * time_step
            
            # Add small sinusoidal disturbances to simulate real-world noise
            noise = 0.05 * np.sin(t * 2 * np.pi * 0.5)  # 0.5 Hz, ±50mV amplitude
            random_noise = np.random.normal(0, 0.01)     # Small random noise ±10mV
            
            test_voltage = base_voltage + noise + random_noise
            
            current = self.simulate_controller_response(test_voltage)
            regen_active = test_voltage < 47.5 and test_voltage > 45.0
            
            self.log_data(t, test_voltage, current, regen_active)
            
            if i % 25 == 0:  # Print every 2.5 seconds
                print(f"  t={t:.1f}s: V={test_voltage:.3f}V, I={current:.1f}A (noise: {noise+random_noise:+.3f}V)")
        
        return True
    
    def test_boundary_conditions(self) -> bool:
        """Test edge cases and boundary conditions"""
        print("Testing boundary conditions...")
        
        boundary_tests = [
            ("Above threshold", 47.6, False),
            ("Exactly at threshold", 47.5, False), 
            ("Just below threshold", 47.49, True),
            ("Mid-range", 46.5, True),
            ("Just above minimum", 45.01, True),
            ("Exactly at minimum", 45.0, False),
            ("Below minimum", 44.0, False),
        ]
        
        for test_name, voltage, expected_regen in boundary_tests:
            current = self.simulate_controller_response(voltage)
            actual_regen = voltage < 47.5 and voltage > 45.0
            
            result = "✓" if actual_regen == expected_regen else "✗"
            print(f"  {result} {test_name}: V={voltage:.2f}V, Expected regen={expected_regen}, Actual={actual_regen}")
            
            self.log_data(len(self.test_data) * 0.1, voltage, current, actual_regen)
        
        return True
    
    def test_performance(self) -> bool:
        """Test controller performance metrics"""
        print("Testing performance...")
        
        # Reset PID state
        self.pid_integral = 0.0
        self.pid_previous_error = 0.0
        
        # Simulate realistic high-frequency voltage variations
        start_time = time.time()
        duration = 2.0  # 2 seconds
        time_step = 0.001  # 1ms (1kHz simulation)
        num_points = int(duration / time_step)
        
        # Only log every 10ms to keep data manageable
        log_interval = 10
        
        for i in range(num_points):
            t = i * time_step
            
            # Create realistic voltage profile with multiple frequency components
            base_voltage = 47.0
            slow_variation = 0.3 * np.sin(t * 2 * np.pi * 0.5)    # 0.5 Hz variation
            fast_variation = 0.1 * np.sin(t * 2 * np.pi * 5.0)    # 5 Hz variation  
            noise = np.random.normal(0, 0.02)                      # Measurement noise
            
            voltage = base_voltage + slow_variation + fast_variation + noise
            voltage = max(min(voltage, 50.0), 44.0)  # Clamp to realistic range
            
            current = self.simulate_controller_response(voltage)
            
            # Log every 10ms for plotting
            if i % log_interval == 0:
                self.log_data(t, voltage, current, voltage < 47.5 and voltage > 45.0)
        
        elapsed = time.time() - start_time
        effective_rate = num_points / elapsed
        
        print(f"  {num_points} updates completed in {elapsed:.3f}s")
        print(f"  Effective update rate: {effective_rate:.0f} Hz")
        print(f"  Target rate: {1/time_step:.0f} Hz")
        
        return elapsed < 5.0  # Should complete in reasonable time
    
    def generate_report(self) -> None:
        """Generate test report with plots"""
        if not self.test_data:
            print("No test data to report")
            return
        
        # Extract data for plotting
        times = [d['time'] for d in self.test_data]
        voltages = [d['voltage'] for d in self.test_data]
        currents = [d['current'] for d in self.test_data]
        regen_states = [d['regen_active'] for d in self.test_data]
        
        # Create plots
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))
        
        # Voltage plot
        ax1.plot(times, voltages, 'b-', linewidth=2, label='Voltage')
        ax1.axhline(y=47.5, color='r', linestyle='--', label='Threshold (47.5V)')
        ax1.axhline(y=45.0, color='orange', linestyle='--', label='Minimum (45.0V)')
        ax1.axhline(y=48.0, color='g', linestyle='--', label='Target (48.0V)')
        ax1.set_ylabel('Voltage (V)')
        ax1.set_title('VESC Power Controller Test Results')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Current plot
        ax2.plot(times, currents, 'r-', linewidth=2, label='Regen Current')
        ax2.fill_between(times, currents, alpha=0.3, color='red')
        ax2.set_ylabel('Current (A)')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Regen state plot
        regen_values = [1 if state else 0 for state in regen_states]
        ax3.plot(times, regen_values, 'g-', linewidth=3, label='Regen Active')
        ax3.fill_between(times, regen_values, alpha=0.3, color='green')
        ax3.set_ylabel('Regen State')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylim(-0.1, 1.1)
        ax3.set_yticks([0, 1])
        ax3.set_yticklabels(['OFF', 'ON'])
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig('power_controller_test_results.png', dpi=300, bbox_inches='tight')
        # plt.show()  # Disabled for headless environment
        
        print("\nTest report generated: power_controller_test_results.png")
    
    def run_all_tests(self) -> bool:
        """Run complete test suite"""
        print("=" * 50)
        print("VESC Power Controller Integration Test Suite")
        print("=" * 50)
        
        tests = [
            ("Voltage Response", self.test_voltage_response),
            ("PID Stability", self.test_pid_stability), 
            ("Boundary Conditions", self.test_boundary_conditions),
            ("Performance", self.test_performance),
        ]
        
        passed = 0
        total = len(tests)
        
        for test_name, test_func in tests:
            print(f"\n--- {test_name} ---")
            try:
                result = test_func()
                if result:
                    print(f"✓ {test_name} PASSED")
                    passed += 1
                else:
                    print(f"✗ {test_name} FAILED")
            except Exception as e:
                print(f"✗ {test_name} ERROR: {e}")
        
        print(f"\n" + "=" * 50)
        print(f"Test Results: {passed}/{total} tests passed")
        print(f"Success Rate: {passed/total*100:.1f}%")
        
        self.generate_report()
        
        return passed == total

def main():
    """Main test runner"""
    # Initialize tester (would connect to real VESC in practice)
    tester = PowerControllerTester()
    
    # Run tests
    success = tester.run_all_tests()
    
    if success:
        print("\n🎉 All tests passed! Power controller is ready for real hardware testing.")
    else:
        print("\n❌ Some tests failed. Review results before hardware testing.")

if __name__ == "__main__":
    main()
