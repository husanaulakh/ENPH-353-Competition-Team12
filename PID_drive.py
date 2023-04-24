#! /usr/bin/env python3

## \file
# \brief A file containing the PID_drive class that moves a robot based on image data received from a camera
# \author [Author Name]

# Import packages
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
from geometry_msgs.msg import Twist

## \class PID_drive
# \brief A class that moves a robot based on image data received from a camera
class PID_drive:
    
    ## \brief Initializes an instance of the PID_drive class
    # \param self The object pointer
    def __init__(self):
        ## \brief Create a CvBridge object to convert sensor_msgs/Image type to cv2 image
        self.bridge = CvBridge()
        
        ## \brief A publisher to publish Twist messages to control the robot's movement
        self.publish_twist = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
        
        ## \brief A tuple representing the current position of the robot
        self.position = (0, 0)
        
        ## \brief A subscriber to subscribe to the /R1/pi_camera/image_raw topic and receive image data
        self.subscriber = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)

        ## \brief Define the video codec and create a VideoWriter object to write video output
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter('output.mp4',self.fourcc, 20.0, (640,480))


    ## \brief A callback function to process image data and control robot's movement
    # \param self The object pointer
    # \param data The Image message received from the camera
    def callback(self, data):
        """
        Callback function to process image data and control robot's movement

        @param self The object pointer
        @param data The Image message received from the camera
        """
        
        ## \brief Convert the ROS Image message to cv2 image
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        ## \brief Get the height, width, and radius of the frame
        height, width, _ = frame.shape
        radius = 15
        
        ## \brief Get the x, y coordinates of the ball in the image
        x, _ = self.getCoordinates(frame, radius, height)

        ## \brief Calculate the PID controller output
        error = x - width/2
        self.accum_error += error
        output = self.Kp*error + self.Ki*self.accum_error + self.Kd*(error - self.prev_error)
        self.prev_error = error

        ## \brief Set the linear and angular speed of the car using the PID controller output
        self.move.linear.x = 0.01
        self.move.angular.z = -output/500
        
        ## \brief Publish the Twist object to the '/R1/cmd_vel' topic
        self.publish_twist.publish(self.move)
        
        ## \brief Display the image frame
        cv2.imshow("input_frame___", frame)
        cv2.waitKey(3)

    def giveCommands(self, x, width):
        ## Publishes move commands to control the robot's motion.
        ##
        ## @param self The object pointer.
        ## @param x The x-coordinate of the ball.
        ## @param width The width of the image frame.
        ##
        ## @return None
        move = Twist()
        move.linear.x = 0.1
        move.angular.z = -(x- (width/2)) / 500
        self.publish_twist.publish(move)


    def getCoordinates(self,frame, radius, frameHeight):
        ## Get the x,y coordinates of a ball in the image.
        ##
        ## @param self The object pointer.
        ## @param frame The image.
        ## @param radius The radius of the ball.
        ## @param frameHeight The height of the image frame.
        ##
        ## @return The x,y coordinates of the ball.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY_INV)
        cv2.imshow("input_frame____", mask)
        cv2.waitKey(3)
        road_upperbound = 0
        for i in range(radius):
            road_upperbound = -radius+i
            strip = mask[road_upperbound,:]
            road = np.where(strip>=255)[0]
            if len(road) != 0: 
                break
        if len(road) != 0: 
            center = (max(road)+min(road))//2
            (x, y) = (center, frameHeight+road_upperbound)
        else: 
            (x, y) = (0, 0)
        return (x, y)


if __name__ == '__main__':
    ## Main function to run the PID controller for the robot's motion.
    robot = PID_drive()
    rospy.init_node('topic_publisher')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting off")
    cv2.destroyAllWindows()