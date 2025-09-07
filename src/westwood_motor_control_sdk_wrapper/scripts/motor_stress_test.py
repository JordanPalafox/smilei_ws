#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from westwood_motor_interfaces.srv import SetMotorIdAndTarget, GetMotorPositions, GetAvailableMotors
import time
import threading
import signal
import sys

class MotorStressTest(Node):
    def __init__(self):
        super().__init__('motor_stress_test')
        self.get_logger().info('🔥 Motor Stress Test Node - Testing for saturation/freezing')
        
        # Test parameters
        self.test_motor_ids = [1, 2]  # Test motors 1 and 2
        self.target_positions = [0.0, 1.0]  # Alternate between 0 and 1
        self.current_target_index = 0
        self.test_frequency = 10.0  # Hz - How fast to switch positions
        self.running = True
        self.test_count = 0
        self.success_count = 0
        self.failure_count = 0
        
        # Service clients
        self.set_motor_client = self.create_client(
            SetMotorIdAndTarget, 
            'westwood_motor/set_motor_id_and_target'
        )
        self.get_positions_client = self.create_client(
            GetMotorPositions,
            'westwood_motor/get_motor_positions'
        )
        self.get_available_client = self.create_client(
            GetAvailableMotors,
            'westwood_motor/get_available_motors'
        )
        
        # Wait for services
        self.get_logger().info('⏳ Waiting for motor services...')
        self.set_motor_client.wait_for_service(timeout_sec=10.0)
        self.get_positions_client.wait_for_service(timeout_sec=10.0)
        self.get_available_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('✅ All services ready')
        
        # Check available motors first
        self.check_available_motors()
        
        # Setup signal handler for clean exit
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # Start the stress test
        self.start_stress_test()
    
    def signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully"""
        self.get_logger().info('\n🛑 Stopping stress test...')
        self.running = False
        self.print_final_stats()
        sys.exit(0)
    
    def check_available_motors(self):
        """Check which motors are available before starting test"""
        request = GetAvailableMotors.Request()
        future = self.get_available_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result():
            response = future.result()
            if response.success:
                available = response.motor_ids
                self.get_logger().info(f'🔍 Available motors: {available}')
                
                # Update test_motor_ids to only include available motors
                self.test_motor_ids = [m for m in self.test_motor_ids if m in available]
                
                if not self.test_motor_ids:
                    self.get_logger().error('❌ No test motors available!')
                    sys.exit(1)
                else:
                    self.get_logger().info(f'🎯 Testing with motors: {self.test_motor_ids}')
            else:
                self.get_logger().error(f'❌ Failed to get available motors: {response.message}')
                sys.exit(1)
        else:
            self.get_logger().error('❌ Service call failed - is the motor server running?')
            sys.exit(1)
    
    def start_stress_test(self):
        """Start the rapid position switching test"""
        self.get_logger().info(f'🚀 Starting stress test at {self.test_frequency} Hz')
        self.get_logger().info(f'📊 Motors: {self.test_motor_ids}')
        self.get_logger().info(f'📊 Positions: {self.target_positions}')
        self.get_logger().info('📊 Press Ctrl+C to stop and see results')
        
        # Create timer for rapid position changes
        timer_period = 1.0 / self.test_frequency  # seconds
        self.stress_timer = self.create_timer(timer_period, self.stress_test_callback)
        
        # Create status reporter timer (every 2 seconds)
        self.status_timer = self.create_timer(2.0, self.report_status)
    
    def stress_test_callback(self):
        """Rapidly switch motor positions between 0 and 1"""
        if not self.running:
            return
            
        # Alternate target position
        target_pos = self.target_positions[self.current_target_index]
        self.current_target_index = (self.current_target_index + 1) % len(self.target_positions)
        
        # Create target positions array (same position for all test motors)
        target_positions = [target_pos] * len(self.test_motor_ids)
        
        # Send command
        request = SetMotorIdAndTarget.Request()
        request.motor_ids = self.test_motor_ids
        request.target_positions = target_positions
        
        start_time = time.time()
        future = self.set_motor_client.call_async(request)
        
        def handle_response(future):
            end_time = time.time()
            response_time = (end_time - start_time) * 1000  # ms
            
            self.test_count += 1
            
            try:
                result = future.result()
                if result and result.success:
                    self.success_count += 1
                    if self.test_count % 50 == 0:  # Log every 50th success
                        self.get_logger().info(f'✅ Test #{self.test_count}: Motors → {target_pos} ({response_time:.1f}ms)')
                else:
                    self.failure_count += 1
                    self.get_logger().warning(f'❌ Test #{self.test_count} FAILED: {result.message if result else "No response"} ({response_time:.1f}ms)')
                    
            except Exception as e:
                self.failure_count += 1
                self.get_logger().error(f'❌ Test #{self.test_count} ERROR: {str(e)} ({response_time:.1f}ms)')
        
        future.add_done_callback(handle_response)
    
    def report_status(self):
        """Report current test statistics"""
        if self.test_count == 0:
            return
            
        success_rate = (self.success_count / self.test_count) * 100
        failure_rate = (self.failure_count / self.test_count) * 100
        
        self.get_logger().info(f'📊 Status: {self.test_count} tests | ✅ {self.success_count} ({success_rate:.1f}%) | ❌ {self.failure_count} ({failure_rate:.1f}%)')
        
        # Check for potential saturation/freezing
        if failure_rate > 10:  # More than 10% failures
            self.get_logger().warning(f'⚠️ HIGH FAILURE RATE: {failure_rate:.1f}% - Possible saturation!')
        
        if self.test_count > 100 and success_rate == 0:  # No successes after 100 tests
            self.get_logger().error('🚨 COMPLETE FAILURE - Server may be frozen!')
    
    def print_final_stats(self):
        """Print final test statistics"""
        if self.test_count == 0:
            self.get_logger().info('📊 No tests completed')
            return
            
        success_rate = (self.success_count / self.test_count) * 100
        failure_rate = (self.failure_count / self.test_count) * 100
        
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('📊 FINAL STRESS TEST RESULTS')
        self.get_logger().info('='*50)
        self.get_logger().info(f'🎯 Test Motors: {self.test_motor_ids}')
        self.get_logger().info(f'🔄 Test Frequency: {self.test_frequency} Hz')
        self.get_logger().info(f'📈 Total Tests: {self.test_count}')
        self.get_logger().info(f'✅ Successes: {self.success_count} ({success_rate:.1f}%)')
        self.get_logger().info(f'❌ Failures: {self.failure_count} ({failure_rate:.1f}%)')
        
        # Performance evaluation
        if failure_rate == 0:
            self.get_logger().info('🎉 EXCELLENT: No failures detected - Server is robust!')
        elif failure_rate < 5:
            self.get_logger().info('✅ GOOD: Very low failure rate - Server performance is solid')
        elif failure_rate < 15:
            self.get_logger().warning('⚠️ MODERATE: Some failures detected - Check for optimization opportunities')
        else:
            self.get_logger().error('🚨 POOR: High failure rate - Server may have saturation/freezing issues!')
        
        self.get_logger().info('='*50)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MotorStressTest()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()