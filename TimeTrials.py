#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

rospy.init_node('timer_move')

velocity_publisher = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=10)
license_publisher = rospy.Publisher('/license_plate', String, queue_size=10)
time.sleep(2)

start_time = None
distance_moved = 0

def move_robot(linear_speed, angular_speed):
    global distance_moved

    velocity_msg = Twist()
    velocity_msg.linear.x = linear_speed  # Move at a linear speed
    velocity_msg.angular.z = angular_speed  # Set the angular speed for turning

    velocity_publisher.publish(velocity_msg)

    if start_time is not None:
        elapsed_time = time.time() - start_time
        distance_moved = elapsed_time * linear_speed  # Distance = speed * time
        rospy.loginfo("Elapsed time: %.2f seconds. Distance moved: %.2f meters.", elapsed_time, distance_moved)

def stop_robot():
    velocity_msg = Twist()
    velocity_msg.linear.x = 0  # Stop moving
    velocity_msg.angular.z = 0  # Stop turning

    velocity_publisher.publish(velocity_msg)

def start_timer():
    global start_time

    start_time = time.time()
    rospy.loginfo("Timer started.")
    license_publisher.publish(str('TeamRed,multi21,0,XR58'))


def stop_timer():
    global start_time

    start_time = None
    rospy.loginfo("Timer stopped.")
    license_publisher.publish(str('TeamRed,multi21,-1,XR58'))


def main():
    rate = rospy.Rate(10)  # Publish messages at 10 Hz

    while not rospy.is_shutdown():
        if start_time is None:
            # Start the timer and make the robot turn left by 135 degrees
            start_timer()
            move_robot(0, 1.2)

        elif distance_moved < 0.8:
            # Move the robot straight ahead while turning left
            move_robot(0.4, 1.1)

        elif distance_moved < 1.3:
            # Move the robot straight ahead
            move_robot(0.4, 0)

        else:
            # Stop the timer and the robot
            stop_timer()
            stop_robot()
            rospy.signal_shutdown()

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
