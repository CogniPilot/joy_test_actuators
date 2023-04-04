#!/usr/bin/env python3
import rclpy
import numpy as np
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import Parameter, ParameterType, ParameterDescriptor
from actuator_msgs.msg import Actuators
from sensor_msgs.msg import Joy

class JoyTestActuators(Node):
    def __init__(self):
        super().__init__('joy_test_actuators_node')


        joy_input_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='joy input topic name.')

        actuators_output_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='actuators output topic name.')

        joy_scale_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Scale value for joy input.')

        self.declare_parameter("joy_input_topic", "/joy", 
            joy_input_topic_descriptor)

        self.declare_parameter("actuators_output_topic", "/actuators",
            actuators_output_topic_descriptor)

        self.declare_parameter("joy_scale", 100, 
            joy_scale_descriptor)

        self.JoyScale = self.get_parameter("joy_scale").value
        self.ActuatorsOutputTopic = self.get_parameter("actuators_output_topic").value
        self.JoyInputTopic = self.get_parameter("joy_input_topic").value

        self.JoySub = self.create_subscription(Joy, '{:s}'.format(self.JoyInputTopic), self.JoyCallback, 1)
        self.ActuatorsPub = self.create_publisher(Actuators, '{:s}'.format(self.ActuatorsOutputTopic), 0)

    def JoyCallback(self, msgJoy):       
        msg = Actuators()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "can0"
        vel = [float(self.JoyScale*msgJoy.axes[0]),
               float(self.JoyScale*msgJoy.axes[1]),
               float(self.JoyScale*msgJoy.axes[3]),
               float(self.JoyScale*msgJoy.axes[4])]
        msg.velocity = vel#np.frombuffer(vel, dtype=np.float64, count=4)
        self.ActuatorsPub.publish(msg)
        return

def main(args=None):
    rclpy.init()
    JTA = JoyTestActuators()
    rclpy.spin(JTA)
    JTA.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()