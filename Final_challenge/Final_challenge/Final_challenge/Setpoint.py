"""
Signal Generator Node

Author: Ricardo Navarro
GitHub: RikuNav
File: Setpoint.py

This script defines a ROS 2 node that generates various types of signals (sinusoidal, square, sawtooth)
based on the provided parameters (type, amplitude, frequency, offset) and publishes them.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from scipy.signal import sawtooth, square
import numpy as np

class MySignalGenerator(Node):
    
    # ROS 2 Node for generating and publishing various types of signals

    def __init__(self):
    
        # Initializes the Signal Generator Node
        super().__init__('Setpoint')
        
        # Publisher for the generated signal
        self.pub_signal = self.create_publisher(Float32, 'setpoint', 10)
        
        # Declaring parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('type', rclpy.Parameter.Type.INTEGER),  # Type of signal (0: constant, 1: sinusoidal, 2: square, 3: sawtooth)
                ('default.amplitude', rclpy.Parameter.Type.DOUBLE),  # Default amplitude of the signal
                ('default.frequency', rclpy.Parameter.Type.DOUBLE),  # Default frequency of the signal
                ('default.offset', rclpy.Parameter.Type.DOUBLE),  # Default offset of the signal
                ('sinusoidal.amplitude', rclpy.Parameter.Type.DOUBLE),  # Default amplitude of the signal
                ('sinusoidal.frequency', rclpy.Parameter.Type.DOUBLE),  # Default frequency of the signal
                ('sinusoidal.offset', rclpy.Parameter.Type.DOUBLE),  # Default offset of the signal
                ('square.amplitude', rclpy.Parameter.Type.DOUBLE),  # Default amplitude of the signal
                ('square.frequency', rclpy.Parameter.Type.DOUBLE),  # Default frequency of the signal
                ('square.offset', rclpy.Parameter.Type.DOUBLE),  # Default offset of the signal
                ('sawtooth.amplitude', rclpy.Parameter.Type.DOUBLE),  # Default amplitude of the signal
                ('sawtooth.frequency', rclpy.Parameter.Type.DOUBLE),  # Default frequency of the signal
                ('sawtooth.offset', rclpy.Parameter.Type.DOUBLE),  # Default offset of the signal
            ]
        )
        
        # Timer period for generating signals
        timer_period = 0.02  # Frequency of 10 Hz for the publish

        
        # Timer for signal generation
        self.timer = self.create_timer(timer_period, self.signal_callback)
        
        # Logging initialization message
        self.get_logger().info('Setpoint Initialized!!!')
        
        # Initialization of signal parameters
        self.signal_type = 0
        self.signal = Float32()
        self.amplitude = Float32()
        self.frequency = Float32()
        self.time = Float32()
        self.offset = Float32()
        self.time.data = 0.0

    # Callback function to generate and publish signals
    def signal_callback(self):
        
        # Retrieving parameters
        self.signal_type = self.get_parameter('type').get_parameter_value().integer_value

        # Generating and publishing signals based on signal type
        if self.signal_type == 0: # Constant signal
            self.amplitude.data = float(self.get_parameter('default.amplitude').get_parameter_value().double_value)
            self.frequency.data = float(self.get_parameter('default.frequency').get_parameter_value().double_value)
            self.offset.data = float(self.get_parameter('default.offset').get_parameter_value().double_value)
            self.signal.data = self.amplitude.data + self.offset.data
        if self.signal_type == 1:  # Sinusoidal signal
            self.amplitude.data = float(self.get_parameter('sinusoidal.amplitude').get_parameter_value().double_value)
            self.frequency.data = float(self.get_parameter('sinusoidal.frequency').get_parameter_value().double_value)
            self.offset.data = float(self.get_parameter('sinusoidal.offset').get_parameter_value().double_value)
            self.signal.data = self.amplitude.data * np.sin(2 * np.pi * self.frequency.data * self.time.data) + self.offset.data
        elif self.signal_type == 2:  # Square signal
            self.amplitude.data = float(self.get_parameter('square.amplitude').get_parameter_value().double_value)
            self.frequency.data = float(self.get_parameter('square.frequency').get_parameter_value().double_value)
            self.offset.data = float(self.get_parameter('square.offset').get_parameter_value().double_value)
            self.signal.data = self.amplitude.data * square(2 * np.pi * self.frequency.data * self.time.data) + self.offset.data
        elif self.signal_type == 3:  # Sawtooth signal
            self.amplitude.data = float(self.get_parameter('sawtooth.amplitude').get_parameter_value().double_value)
            self.frequency.data = float(self.get_parameter('sawtooth.frequency').get_parameter_value().double_value)
            self.offset.data = float(self.get_parameter('sawtooth.offset').get_parameter_value().double_value)
            self.signal.data = self.amplitude.data * sawtooth(2 * np.pi * self.frequency.data * self.time.data) + self.offset.data
        
        if(self.signal.data > 1.0):
            self.signal.data = 1
        elif(self.signal.data < -1.0):
            self.signal.data = -1

        self.pub_signal.publish(self.signal)

        self.time.data += 0.02

    
# Main function to initialize the node and spin it
def main(args = None):
    rclpy.init(args=args)
    signal_node = MySignalGenerator()
    rclpy.spin(signal_node)
    signal_node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()
