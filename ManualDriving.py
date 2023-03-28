#! /usr/bin/env python3

# packages
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
from geometry_msgs.msg import Twist
from MovementLogger import MovementLogger

class ManualDrive:
    """
    Class to manually drive a robot using arrow keys

    @param self The object pointer
    """
  
    def __init__(self):
        """
        Initializes an instance of the ManualDrive class

        @param self The object pointer
        """
        self.bridge = CvBridge() # Create a CvBridge object to convert sensor_msgs/Image type to cv2 image

        self.linear_speed = 0.3
        self.angular_speed = 0.7
        self.publish_twist = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
        self.move = Twist()
        self.logger = MovementLogger()
        self.movement = [0, 0, 0]
        self.subscriber = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)
        

    def callback(self,data):
        """
        Callback function to process key events and control robot's movement

        @param self The object pointer
        @param data The Image message received from the camera
        """
        key = cv2.waitKey(1) & 0xFF
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        cv2.imshow("input_frame___", frame)
        cv2.waitKey(3)

        if key == 82: # move forward when up arrow key is pressed
            self.move.linear.x = self.linear_speed
            self.move.angular.z = 0
            self.movement = [0, 1, 0]
        
        elif key == 81: # turn left when left arrow key is pressed
            self.move.angular.z = self.angular_speed
            self.move.linear.x = self.linear_speed*0.6
            self.movement = [1, 0, 0]

        elif key == 83: # turn right when right arrow key is pressed
            self.move.linear.x = self.linear_speed/2
            self.move.angular.z = -1*self.angular_speed*0.6
            self.movement = [0, 0, 1]

        # elif key == ord(" "):
        #     self.move.linear.x = 0
        #     self.move.angular.z = 0
        #     self.movement = [0, 0, 0]
        #     self.publish_twist.publish(self.move)
        
        self.publish_twist.publish(self.move)
        if self.movement != [0, 0, 0]:
            self.logger.add_entry(frame, self.movement)


    def shutdown_hook(self):
        """
        Shutdown hook function to save the log before shutting down
        """
        self.logger.save("training_data")


if __name__ == '__main__':
    rospy.init_node('topic_publisher')
    robot = ManualDrive()
    rospy.on_shutdown(robot.shutdown_hook)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting off")
    cv2.destroyAllWindows()