#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import math
import numpy as np
from pybear import Manager


class MotorPositionClient(Node):
    def __init__(self):
        super().__init__('motor_position_client')
        
        # Initialize Pybear manager
        self.bear = Manager.BEAR(port="/dev/ttyUSB0", baudrate=8000000)
        self.m_id_1 = 1
        self.m_id_2 = 2
        
        # Variables para almacenar datos
        self.pos1, self.pos2 = 0.0, 0.0
        self.vel1, self.vel2 = 0.0, 0.0
        
        # PD Control parameters (matching standalone script)
        self.kp = 1.0        # Proportional gain
        self.kd = 0.1        # Damping gain
        self.Kt = 0.35       # N.m/A
        self.r1 = 0.4
        self.r2 = 0.3
        self.p1 = (2*self.r2 - self.r1) / self.r1
        self.p2 = (2*self.r2 - self.r1) / self.r2
        self.Fc = 35         # Frequency cutoff
        self.Tl = 0.002      # Loop frequency
        
        # Velocity estimator variables
        self.theta_1 = 0.0
        self.vel_stim1 = 0.0
        self.theta_2 = 0.0
        self.vel_stim2 = 0.0
        
        # Control start time for trajectory generation
        self.start_time = time.time()
        
        # Variables para calcular frecuencia
        self.last_time = time.time()
        self.response_count = 0
        self.freq_timer = self.create_timer(1.0, self.print_frequency)
        
        # Check BEAR connection
        self.get_logger().info("Connecting to BEAR...")
        BEAR_connected = self.bear.ping(self.m_id_1, self.m_id_2)[0]
        if not BEAR_connected:
            self.get_logger().error("BEAR is offline. Check power and communication.")
            return
        self.get_logger().info("BEAR connected successfully")
        
        # Configure motors
        self.configure_motors()
        
        # Timer para leer a frecuencia controlada
        self.timer = self.create_timer(0.001, self.control_loop)  # 1ms ≈ 1000Hz
        
    def control_loop(self):
        """Main control loop using Pybear directly"""
        try:
            # Get motor positions and velocities directly from Pybear
            self.pos1 = self.bear.get_present_position(self.m_id_1)[0][0][0]
            self.pos2 = self.bear.get_present_position(self.m_id_2)[0][0][0]
            self.vel1 = self.bear.get_present_velocity(self.m_id_1)[0][0][0]
            self.vel2 = self.bear.get_present_velocity(self.m_id_2)[0][0][0]
            
            # Enable torque
            self.bear.set_torque_enable((self.m_id_1, 1), (self.m_id_2, 1))
            
            # Compute and apply PD control
            self.response_count += 1
            self.compute_pd_control()
            
            self.get_logger().info(f"Motor 1: pos={self.pos1:.4f}, vel={self.vel1:.4f} | Motor 2: pos={self.pos2:.4f}, vel={self.vel2:.4f}")
            
        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}")
    
    def print_frequency(self):
        current_time = time.time()
        elapsed_time = current_time - self.last_time
        frequency = self.response_count / elapsed_time if elapsed_time > 0 else 0
        self.get_logger().info(f"==================== Frecuencia: {frequency:.1f} Hz ({self.response_count} respuestas en {elapsed_time:.1f}s) ====================")
        self.response_count = 0
        self.last_time = current_time
    
    def configure_motors(self):
        """Configure motors using Pybear methods"""
        try:
            self.get_logger().info("Configuring motors...")
            
            # Set PID gains for iq/id control
            self.bear.set_p_gain_iq((self.m_id_1, 0.277), (self.m_id_2, 0.277))
            self.bear.set_i_gain_iq((self.m_id_1, 0.061), (self.m_id_2, 0.061))
            self.bear.set_d_gain_iq((self.m_id_1, 0), (self.m_id_2, 0))
            self.bear.set_p_gain_id((self.m_id_1, 0.277), (self.m_id_2, 0.277))
            self.bear.set_i_gain_id((self.m_id_1, 0.061), (self.m_id_2, 0.061))
            self.bear.set_d_gain_id((self.m_id_1, 0), (self.m_id_2, 0))
            
            # Clear PID position mode
            self.bear.set_p_gain_position((self.m_id_1, 0), (self.m_id_2, 0))
            self.bear.set_i_gain_position((self.m_id_1, 0), (self.m_id_2, 0))
            self.bear.set_d_gain_position((self.m_id_1, 0), (self.m_id_2, 0))
            
            # Clear PID force mode
            self.bear.set_p_gain_force((self.m_id_1, 0), (self.m_id_2, 0))
            self.bear.set_i_gain_force((self.m_id_1, 0), (self.m_id_2, 0))
            self.bear.set_d_gain_force((self.m_id_1, 0), (self.m_id_2, 0))
            
            # Set motors to current mode (mode 0)
            self.bear.set_mode((self.m_id_1, 0), (self.m_id_2, 0))
            
            self.get_logger().info("Motors configured successfully")
                
        except Exception as e:
            self.get_logger().error(f"Error configuring motors: {e}")
    
    def compute_pd_control(self):
        """Compute PD control and send current commands"""
        try:
            # Generate desired trajectory (sinusoidal reference like in original code)
            current_time = time.time()
            elapsed_time = current_time - self.start_time
            qd = 0.5 * math.pi * math.sin(elapsed_time)
            
            # Compute position errors (desired position = 0.0 like in original)
            e_1 = self.pos1 - 0.0
            e_2 = self.pos2 - 0.0
            
            # Velocity estimator (low-pass filter implementation from original)
            self.vel_stim1 = self.Fc * (self.theta_1 + self.pos1)
            self.theta_1 = self.theta_1 - self.Tl * self.vel_stim1
            
            self.vel_stim2 = self.Fc * (self.theta_2 + self.pos2)
            self.theta_2 = self.theta_2 - self.Tl * self.vel_stim2
            
            # Compute PD control torques (non-linear proportional term)
            tau1 = -self.kp * ((abs(e_1)**self.p1) * np.sign(e_1)) - self.kd * self.vel_stim1
            tau2 = -self.kp * ((abs(e_2)**self.p1) * np.sign(e_2)) - self.kd * self.vel_stim2
            
            # Convert torque to current
            i_g_1 = tau1 / self.Kt
            i_g_2 = tau2 / self.Kt
            
            # Send current commands
            self.send_current_commands([i_g_1, i_g_2])
            
        except Exception as e:
            self.get_logger().error(f"Error en cálculo PD: {e}")
    
    def send_current_commands(self, currents):
        """Send current commands to motors using Pybear"""
        try:
            # Send current commands to both motors
            self.bear.set_goal_iq((self.m_id_1, currents[0]))
            self.bear.set_goal_iq((self.m_id_2, currents[1]))
            
        except Exception as e:
            self.get_logger().error(f"Error sending current commands: {e}")
    
    def destroy_node(self):
        """Clean shutdown - disable torque before destroying node"""
        try:
            self.get_logger().info("Shutting down - disabling torque...")
            self.bear.set_torque_enable((self.m_id_1, 0), (self.m_id_2, 0))
            self.get_logger().info("Thanks for using BEAR!")
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {e}")
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = MotorPositionClient()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Cliente terminado por usuario")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()