#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class RobotLidarController:
	def __init__(self):
		self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

		self.front_distance = None
		self.obstacle_distance = rospy.get_param("~obstacle_distance", 0.4)
		self.forward_speed = 0.2

		self.turn_degrees = 90.0
		self.turn_duration = 1.5
		self.turning = False
		self.turn_end_time = rospy.Time.now()
		self.state = "FORWARD"

	def degrees_to_angular_z(self, degrees, duration_sec):
		"""Convert degrees over duration to angular velocity in rad/s."""
		if duration_sec <= 0:
			raise ValueError("duration_sec must be > 0")
		return math.radians(degrees) / duration_sec

	def scan_callback(self, msg):
		"""Store the minimum valid distance in a small front LiDAR sector."""
		total = len(msg.ranges)
		if total == 0:
			self.front_distance = None
			return

		window = 15
		indices = list(range(window)) + list(range(total - window, total))

		valid = []
		for i in indices:
			distance = msg.ranges[i]
			if math.isfinite(distance) and distance >= msg.range_min and distance <= msg.range_max:
				valid.append(distance)

		self.front_distance = min(valid) if valid else None

	def publish_cmd(self, linear_x, angular_z):
		cmd = Twist()
		cmd.linear.x = linear_x
		cmd.angular.z = angular_z
		self.cmd_pub.publish(cmd)

	def start_turn(self):
		self.turning = True
		self.state = "TURNING"
		self.turn_end_time = rospy.Time.now() + rospy.Duration(self.turn_duration)

	def run(self):
		rate = rospy.Rate(10)

		while not rospy.is_shutdown():
			if self.state == "TURNING":
				if rospy.Time.now() < self.turn_end_time:
					self.publish_cmd(
						linear_x=0.0,
						angular_z=self.degrees_to_angular_z(self.turn_degrees, self.turn_duration),
					)
				else:
					self.turning = False
					self.state = "FORWARD"
					self.publish_cmd(linear_x=0.0, angular_z=0.0)
			elif self.front_distance is not None and self.front_distance <= self.obstacle_distance:
				rospy.loginfo(
					"Wall detected: distance=%.2f m (threshold=%.2f m)",
					self.front_distance,
					self.obstacle_distance,
				)
				self.publish_cmd(linear_x=0.0, angular_z=0.0)
				self.start_turn()
			else:
				self.publish_cmd(linear_x=self.forward_speed, angular_z=0.0)

			rate.sleep()


def main():
	rospy.init_node("robot_lidar")
	controller = RobotLidarController()
	rospy.sleep(0.5)
	controller.run()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
