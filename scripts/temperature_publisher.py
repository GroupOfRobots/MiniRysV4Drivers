#!/usr/bin/env python3
from tmp102 import TMP102
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from std_msgs.msg import Header
from sensor_msgs.msg import Temperature

class TmpPublisher(Node):

	def __init__(self):
		super().__init__('temperature_publisher')

		period = self.declare_parameter("period", 5)
		self.timer_period = period.value
		units = self.declare_parameter("units", 'C')
		self.tmp_units = units.value

		self.publisher_ = self.create_publisher(Temperature, 'tmp', 10)
		self.tmp = TMP102('C', 0x48, 1)
		self.tmp.setUnits(self.tmp_units)

		self.timer = self.create_timer(self.timer_period, self.timer_callback)

	def timer_callback(self):
		msg = Temperature()
		h = Header()

		# uncomment for SYSTEM_TIME
		h.stamp = Clock().now().to_msg()

		# uncomment for ROS_TIME
		# h.stamp = self.get_clock().now().to_msg()

		msg.header = h
		msg.temperature = self.tmp.readTemperature()
		msg.variance = 0.0
		self.publisher_.publish(msg)

if __name__ == '__main__':
	rclpy.init()
	tmp_publisher = TmpPublisher()
	rclpy.spin(tmp_publisher)
	tmp_publisher.destroy_node()
	rclpy.shutdown()
