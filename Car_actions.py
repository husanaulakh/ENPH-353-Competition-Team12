# packages
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
from geometry_msgs.msg import Twist
from MovementLogger import MovementLogger


class CarActions:
    def __init__(self, linear_speed=0.3, angular_speed=0.7, log=False):
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.movement = [0, 0, 0] # Forward, Left, Right
        self.move = Twist()
        self.publish_twist = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
        self.log = log
        self.logger = MovementLogger()

    
    def move_forward(self):
        self.move.linear.x = self.linear_speed
        self.move.angular.z = 0
        self.movement = [0, 1, 0]
        self.publish_twist.publish(self.move)

    def turn_left(self):
        self.move.angular.z = self.angular_speed
        self.move.linear.x = self.linear_speed * 0.6
        self.movement = [1, 0, 0]
        self.publish_twist.publish(self.move)

    def turn_right(self):
        self.move.linear.x = self.linear_speed * 0.6
        self.move.angular.z = -1 * self.angular_speed
        self.movement = [0, 0, 1]
        self.publish_twist.publish(self.move)

    def move_backward(self):
        self.move.linear.x = -1 * self.linear_speed
        self.move.angular.z = 0
        self.movement = [0, 0, 0]
        self.publish_twist.publish(self.move)

    def stop(self):
        self.move.linear.x = 0
        self.move.angular.z = 0
        self.movement = [0, 0, 0]
        self.publish_twist.publish(self.move)
        
    
    def perform_action(self, key):
        if key == 82: # move forward when up arrow key is pressed
            self.move.linear.x = self.linear_speed
            self.move.angular.z = 0
            self.movement = [0, 1, 0]
        
        elif key == 81: # turn left when left arrow key is pressed
            self.move.angular.z = self.angular_speed
            self.move.linear.x = self.linear_speed*0.6
            self.movement = [1, 0, 0]

        elif key == 83: # turn right when right arrow key is pressed
            self.move.linear.x = self.linear_speed*0.6
            self.move.angular.z = -1*self.angular_speed
            self.movement = [0, 0, 1]
        
        elif key == 84: # move backward when down arrow key is pressed
            self.move.linear.x = -1*self.linear_speed
            self.move.angular.z = 0
            self.movement = [0, 0, 0]

        elif key == ord(" "): # stop when space bar is pressed
            self.move.linear.x = 0
            self.move.angular.z = 0
            self.movement = [0, 0, 0]
        
        if self.log:
            print(f"Movement: {self.movement}, Twist: {self.move}")
        
        self.publish_twist.publish(self.move)
