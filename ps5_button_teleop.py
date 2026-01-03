#!/usr/bin/env python3
"""
PS5 Button-Based Teleop Node
Converts button presses to cmd_vel commands
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class PS5ButtonTeleop(Node):
    def __init__(self):
        super().__init__('ps5_button_teleop')
        
        # Button mappings (PS5 DualSense)
        # 0=X, 1=Circle, 2=Square, 3=Triangle
        # 4=L1, 5=R1, 6=L2, 7=R2
        self.declare_parameter('button_forward', 3)      # Triangle = forward
        self.declare_parameter('button_backward', 0)     # X = backward
        self.declare_parameter('button_left', 4)        # L1 = rotate left
        self.declare_parameter('button_right', 5)        # R1 = rotate right
        
        # Speed settings
        self.declare_parameter('linear_speed', 0.3)     # m/s
        self.declare_parameter('angular_speed', 0.5)     # rad/s
        
        # Get parameters
        self.button_forward = self.get_parameter('button_forward').value
        self.button_backward = self.get_parameter('button_backward').value
        self.button_left = self.get_parameter('button_left').value
        self.button_right = self.get_parameter('button_right').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        
        # Subscribe to joy topic
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Publish cmd_vel
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Current button states
        self.buttons_pressed = [False] * 20
        
        self.get_logger().info('PS5 Button Teleop started')
        self.get_logger().info(f'Forward: Button {self.button_forward} (Triangle)')
        self.get_logger().info(f'Backward: Button {self.button_backward} (X)')
        self.get_logger().info(f'Left: Button {self.button_left} (L1)')
        self.get_logger().info(f'Right: Button {self.button_right} (R1)')
    
    def joy_callback(self, msg):
        # Update button states
        if len(msg.buttons) > max(self.button_forward, self.button_backward, 
                                  self.button_left, self.button_right):
            self.buttons_pressed = msg.buttons
        
        # Create twist message
        twist = Twist()
        
        # Forward/Backward
        if self.buttons_pressed[self.button_forward]:
            twist.linear.x = self.linear_speed
        elif self.buttons_pressed[self.button_backward]:
            twist.linear.x = -self.linear_speed
        
        # Left/Right rotation
        if self.buttons_pressed[self.button_left]:
            twist.angular.z = self.angular_speed
        elif self.buttons_pressed[self.button_right]:
            twist.angular.z = -self.angular_speed
        
        # Publish command
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PS5ButtonTeleop()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot before exiting
        stop_twist = Twist()
        node.cmd_vel_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

