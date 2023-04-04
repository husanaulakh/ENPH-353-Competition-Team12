#! /usr/bin/env python3

# packages
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
from geometry_msgs.msg import Twist

# class Video_recorder:
    
#     def __init__(self):
#         self.bridge = CvBridge()
#         # Define the video codec and create VideoWriter object
#         self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
#         self.out = cv2.VideoWriter('output.mp4',self.fourcc, 20.0, (640,480))
#         self.subscriber = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)

#     def callback(self):
#         frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        


class PID_drive:
    """
    Class to move a robot based on image data received from a camera

    @param self The object pointer
    """
  
    def __init__(self):
        """
        Initializes an instance of the move_robot class

        @param self The object pointer
        """
        self.bridge = CvBridge() # Create a CvBridge object to convert sensor_msgs/Image type to cv2 image
        self.publish_twist = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
        self.position = (0, 0)
        self.subscriber = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)

        # Define the video codec and create VideoWriter object
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter('output.mp4',self.fourcc, 20.0, (640,480))

    def callback(self,data):
        """
        Callback function to process image data and control robot's movement

        @param self The object pointer
        @param data The Image message received from the camera
        """
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        height, width, _ = frame.shape
        radius = 15
        (x, y) = self.getCoordinates(frame, radius, height)

        # fps = video_reader.get(cv2.CAP_PROP_FPS)
        # out = cv2.VideoWriter('output.mp4', fourcc, fps, (width, height))

        if x != 0 and y != 0: 
            frame = cv2.circle(frame, (x, y), radius=radius, color=(0,0,255), thickness=-1)
            move = Twist()
            move.linear.x = 0.01
            move.angular.z = -(x- (width/2)) / 500
            self.publish_twist.publish(move)
        else: 
            move = Twist()
            move.linear.x = 0.0
            move.angular.z = -0.4
            self.publish_twist.publish(move)

        cv2.imshow("input_frame___", frame)
        cv2.waitKey(3)


    def giveCommands(self, x, width):
        move = Twist()
        move.linear.x = 0.1
        move.angular.z = -(x- (width/2)) / 500
        self.publish_twist.publish(move)


    def getCoordinates(self,frame, radius, frameHeight):
        """
        Get the x,y coordinates of a ball in the image

        @param self The object pointer
        @param frame The image
        @param radius The radius of the ball
        @param frameHeight The height of the image frame

        @return The x,y coordinates of the ball
        """

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY_INV)
        cv2.imshow("input_frame____", mask)
        cv2.waitKey(3)
        road_upperbound = 0
        for i in range(radius):
            road_upperbound = -radius+i
            strip = mask[road_upperbound,:]
            road = np.where(strip>=255)[0]
            if len(road) != 0: break
        if len(road) != 0: 
            center = (max(road)+min(road))//2
            (x, y) = (center, frameHeight+road_upperbound)
        else: (x, y) = (0, 0)
        return (x, y)

if __name__ == '__main__':
    robot = PID_drive()
    rospy.init_node('topic_publisher')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting off")
    cv2.destroyAllWindows()