#!/usr/bin/env python3
from mcp3221 import MCP3221
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from std_msgs.msg import Header
from sensor_msgs.msg import BatteryState

class VolPublisher(Node):

	def __init__(self):
		super().__init__('voltage_publisher')

		period = self.declare_parameter("period", 5)
		self.timer_period = period.value

		self.publisher_ = self.create_publisher(BatteryState, 'voltage', 10)
		self.vol = MCP3221(0x48, 0, 3.3, [10, 37])

		self.timer = self.create_timer(self.timer_period, self.timer_callback)

	def timer_callback(self):
		msg = BatteryState()
		h = Header()

		# uncomment for SYSTEM_TIME
		h.stamp = Clock().now().to_msg()

		# uncomment for ROS_TIME
		# h.stamp = self.get_clock().now().to_msg()

		msg.header = h
		msg.voltage = self.vol.readVoltage()
		msg.percentage = min(0.0, max(1.0, (msg.voltage-10.0)/2.210))
		msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL if msg.voltage >= 12.0 else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
		msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD if msg.voltage < 10.0 else BatteryState.POWER_SUPPLY_HEALTH_GOOD
		msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
		msg.present = True if msg.voltage > 0 else False
		self.publisher_.publish(msg)

if __name__ == '__main__':
	rclpy.init()
	vol_publisher = VolPublisher()
	rclpy.spin(vol_publisher)
	vol_publisher.destroy_node()
	rclpy.shutdown()
