#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist


def publish_for_duration(pub, linear_x, angular_z, duration_sec, rate_hz=10):
	"""Publish a constant velocity command for a fixed duration."""
	cmd = Twist()
	cmd.linear.x = linear_x
	cmd.angular.z = angular_z

	rate = rospy.Rate(rate_hz)
	end_time = rospy.Time.now() + rospy.Duration(duration_sec)

	while not rospy.is_shutdown() and rospy.Time.now() < end_time:
		pub.publish(cmd)
		rate.sleep()


def stop_for_duration(pub, duration_sec=1.0, rate_hz=10):
	"""Publish zero velocity for a fixed duration."""
	publish_for_duration(pub, linear_x=0.0, angular_z=0.0, duration_sec=duration_sec, rate_hz=rate_hz)


def degrees_to_angular_z(degrees, duration_sec):
	"""Convert a target turn in degrees over time into angular_z (rad/s)."""
	if duration_sec <= 0:
		raise ValueError("duration_sec must be > 0")
	return math.radians(degrees) / duration_sec


def main():
	rospy.init_node("move_robot")
	cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

	# Give publisher time to connect.
	rospy.sleep(0.5)

	# Move forward for 3 seconds.
	publish_for_duration(cmd_pub, linear_x=0.2, angular_z=0.0, duration_sec=3.0)
	stop_for_duration(cmd_pub, duration_sec=1.0)

	# Turn left in place.
	turn_duration = 1.5
	left_turn_degrees = 90.0
	publish_for_duration(
		cmd_pub,
		linear_x=0.0,
		angular_z=degrees_to_angular_z(left_turn_degrees, turn_duration),
		duration_sec=turn_duration,
	)
	stop_for_duration(cmd_pub, duration_sec=1.0)

	# Move forward again for 3 seconds.
	publish_for_duration(cmd_pub, linear_x=0.2, angular_z=0.0, duration_sec=3.0)

	# Stop at the end.
	cmd_pub.publish(Twist())


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
