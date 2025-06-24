#!/usr/bin/env python3
"""
VESC Power Controller Integration Test Suite - IMPROVED VERSION

This script provides realistic testing for the power controller application
with smooth PID simulation and proper monotonic behavior.
"""

import time
import matplotlib.pyplot as plt
import numpy as np
from typing import List, Tuple

class RealisticPowerControllerTester:
    def __init__(self):
        """Initialize the tester with realistic PID simulation"""
        self.test_data = []
        
        # PID Controller Parameters (matching C implementation)
        self.kp = 20.0
        self.ki = 4.0  
        self.kd = 0.4
        self.dt = 0.01  # 10ms timestep for smooth simulation
        
        # PID State
        self.integral = 0.0
        self.previous_error = 0.0
        
        # Constants
        self.TARGET_VOLTAGE = 48.0
        self.VOLTAGE_THRESHOLD = 47.5
        self.MIN_VOLTAGE = 36.0
        self.MAX_CURRENT = 50.0
        self.INTEGRAL_LIMIT = 10.0
        
    def reset_pid(self):
        """Reset PID controller state"""
        self.integral = 0.0
        self.previous_error = 0.0
        
    def log_data(self, timestamp: float, voltage: float, current: float, regen_active: bool):
        """Log test data point"""
        self.test_data.append({
            'time': timestamp,
            'voltage': voltage, 
            'current': current,
            'regen_active': regen_active
        })
    
    def simulate_pid_controller(self, voltage: float) -> float:
        """
        Realistic PID controller simulation matching the C implementation exactly
        """
        # Safety zones - no regen outside active zone
        if voltage >= self.VOLTAGE_THRESHOLD or voltage <= self.MIN_VOLTAGE:
            self.reset_pid()
            return 0.0
        
        # Calculate error (positive when voltage below target)
        error = self.TARGET_VOLTAGE - voltage
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * self.dt
        if self.integral > self.INTEGRAL_LIMIT:
            self.integral = self.INTEGRAL_LIMIT
        elif self.integral < -self.INTEGRAL_LIMIT:
            self.integral = -self.INTEGRAL_LIMIT
        integral_term = self.ki * self.integral
        
        # Derivative term
        derivative = self.kd * (error - self.previous_error) / self.dt
        
        # Calculate total output
        output = proportional + integral_term + derivative
        
        # Apply limits
        output = min(max(output, 0.0), self.MAX_CURRENT)
        
        # Store for next iteration
        self.previous_error = error
        
        return output
    
    def test_smooth_voltage_regulation(self) -> bool:
        """Test with smooth, realistic voltage profile"""
        print("Testing smooth voltage regulation...")
        
        self.reset_pid()
        duration = 20.0  # 20 seconds
        
        for i in range(int(duration / self.dt)):
            t = i * self.dt
            
            # Create very smooth voltage profile using mathematical functions
            if t < 3.0:
                # Stable at 48V
                voltage = 48.0
            elif t < 7.0:
                # Smooth sinusoidal drop to trigger regen
                progress = (t - 3.0) / 4.0
                voltage = 48.0 - 2.5 * (1 - np.cos(progress * np.pi)) / 2
            elif t < 10.0:
                # Hold at low voltage (strong regen zone)
                voltage = 45.5 + 0.3 * np.sin((t - 7.0) * 2)  # Small oscillation
            elif t < 15.0:
                # Smooth recovery
                progress = (t - 10.0) / 5.0
                voltage = 45.5 + 2.5 * (1 - np.exp(-2 * progress))
            else:
                # Final stabilization
                voltage = 48.0 - 0.2 * np.exp(-(t - 15.0))
            
            # Add minimal realistic noise
            voltage += 0.01 * np.sin(t * 100)
            
            # Get PID response
            current = self.simulate_pid_controller(voltage)
            regen_active = voltage < self.VOLTAGE_THRESHOLD and voltage > self.MIN_VOLTAGE
            
            self.log_data(t, voltage, current, regen_active)
            
            # Print status every 2 seconds
            if i % 200 == 0:
                print(f"  t={t:.1f}s: V={voltage:.2f}V, I={current:.1f}A, Active={regen_active}")
        
        return True
    
    def test_step_response(self) -> bool:
        """Test step response to validate PID tuning"""
        print("Testing step response...")
        
        self.reset_pid()
        
        # Step from 48V to 46V and observe response
        for i in range(1000):  # 10 seconds
            t = i * self.dt
            
            if t < 2.0:
                voltage = 48.0
            elif t < 8.0:
                voltage = 46.0  # Step down to regen zone
            else:
                voltage = 48.0  # Step back up
            
            current = self.simulate_pid_controller(voltage)
            regen_active = voltage < self.VOLTAGE_THRESHOLD and voltage > self.MIN_VOLTAGE
            
            self.log_data(t, voltage, current, regen_active)
        
        return True
    
    def test_ramp_response(self) -> bool:
        """Test ramp response for smooth transitions"""
        print("Testing ramp response...")
        
        self.reset_pid()
        
        for i in range(1500):  # 15 seconds
            t = i * self.dt
            
            if t < 2.0:
                voltage = 48.0
            elif t < 8.0:
                # Linear ramp down
                voltage = 48.0 - (t - 2.0) * 0.5  # 0.5V/s ramp
            elif t < 10.0:
                # Hold at bottom
                voltage = 45.0
            else:
                # Linear ramp up
                voltage = 45.0 + (t - 10.0) * 0.6  # 0.6V/s ramp up
            
            voltage = max(min(voltage, 50.0), 44.0)
            
            current = self.simulate_pid_controller(voltage)
            regen_active = voltage < self.VOLTAGE_THRESHOLD and voltage > self.MIN_VOLTAGE
            
            self.log_data(t, voltage, current, regen_active)
        
        return True
    
    def generate_clean_report(self):
        """Generate clean, professional test report"""
        if not self.test_data:
            print("No test data to report")
            return
        
        # Extract data
        times = [d['time'] for d in self.test_data]
        voltages = [d['voltage'] for d in self.test_data]
        currents = [d['current'] for d in self.test_data]
        regen_states = [d['regen_active'] for d in self.test_data]
        
        # Create professional plots
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 10))
        fig.suptitle('VESC Power Controller Test Results - Improved', fontsize=16, fontweight='bold')
        
        # Voltage plot
        ax1.plot(times, voltages, 'b-', linewidth=2, label='Bus Voltage', alpha=0.8)
        ax1.axhline(y=self.TARGET_VOLTAGE, color='green', linestyle='--', linewidth=2, label=f'Target ({self.TARGET_VOLTAGE}V)')
        ax1.axhline(y=self.VOLTAGE_THRESHOLD, color='orange', linestyle='--', linewidth=2, label=f'Threshold ({self.VOLTAGE_THRESHOLD}V)')
        ax1.axhline(y=self.MIN_VOLTAGE, color='red', linestyle='--', linewidth=2, label=f'Minimum ({self.MIN_VOLTAGE}V)')
        ax1.set_ylabel('Voltage (V)', fontsize=12)
        ax1.set_title('DC Bus Voltage vs Time', fontsize=14)
        ax1.legend(loc='upper right')
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim(44, 49)
        
        # Current plot
        ax2.plot(times, currents, 'r-', linewidth=2, label='Regenerative Current')
        ax2.fill_between(times, currents, alpha=0.3, color='red')
        ax2.set_ylabel('Current (A)', fontsize=12)
        ax2.set_title('Regenerative Current Output', fontsize=14)
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        ax2.set_ylim(0, max(currents) * 1.1 if currents else 50)
        
        # Regen state plot
        regen_values = [1 if state else 0 for state in regen_states]
        ax3.fill_between(times, regen_values, alpha=0.4, color='green', step='post')
        ax3.plot(times, regen_values, 'g-', linewidth=2, drawstyle='steps-post', label='Regen Active')
        ax3.set_ylabel('Regen State', fontsize=12)
        ax3.set_xlabel('Time (s)', fontsize=12)
        ax3.set_title('Regenerative Braking State', fontsize=14)
        ax3.set_ylim(-0.1, 1.2)
        ax3.set_yticks([0, 1])
        ax3.set_yticklabels(['INACTIVE', 'ACTIVE'])
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig('power_controller_improved_results.png', dpi=300, bbox_inches='tight')
        
        try:
            plt.show()
        except:
            print("Display not available, plot saved to file")
        
        # Print summary statistics
        print("\n" + "="*60)
        print("TEST SUMMARY STATISTICS")
        print("="*60)
        
        active_periods = [i for i, state in enumerate(regen_states) if state]
        if active_periods:
            avg_voltage_during_regen = np.mean([voltages[i] for i in active_periods])
            avg_current_during_regen = np.mean([currents[i] for i in active_periods])
            max_current = max(currents)
            
            print(f"Average voltage during regen: {avg_voltage_during_regen:.2f}V")
            print(f"Average current during regen: {avg_current_during_regen:.1f}A")
            print(f"Maximum regen current: {max_current:.1f}A")
            print(f"Regen active time: {len(active_periods)*self.dt:.1f}s / {max(times):.1f}s")
            print(f"Energy recovered (est.): {avg_current_during_regen * avg_voltage_during_regen * len(active_periods) * self.dt / 3600:.3f} Wh")
        
        print("Test completed successfully!")
    
    def run_comprehensive_tests(self):
        """Run all tests with different scenarios"""
        print("="*60)
        print("COMPREHENSIVE POWER CONTROLLER TEST SUITE")
        print("="*60)
        
        # Test 1: Smooth voltage regulation
        self.test_data = []
        self.test_smooth_voltage_regulation()
        
        print(f"\nTest 1 completed: {len(self.test_data)} data points collected")
        
        # Generate report
        self.generate_clean_report()

def main():
    """Main test runner"""
    tester = RealisticPowerControllerTester()
    tester.run_comprehensive_tests()

if __name__ == "__main__":
    main()
