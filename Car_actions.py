## \file
# \brief A file containing the CarActions class that controls a robotic car's movements

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
from geometry_msgs.msg import Twist
from MovementLogger import MovementLogger

## \class CarActions
# \brief A class that controls a robotic car's movements
class CarActions:
    
    ## \brief Initializes the CarActions object
    # \param linear_speed A float representing the linear speed of the car (default 0.3)
    # \param angular_speed A float representing the angular speed of the car (default 1.05)
    # \param log A boolean representing whether to log the movements of the car (default False)
    def __init__(self, linear_speed=0.3, angular_speed=1.05, log=False):
        
        ## \brief A CvBridge object to convert sensor_msgs/Image type to cv2 image
        self.bridge = CvBridge()
        
        ## \brief A float representing the linear speed of the car
        self.linear_speed = linear_speed
        
        ## \brief A float representing the angular speed of the car
        self.angular_speed = angular_speed
        
        ## \brief A list representing the movement of the car (Forward, Left, Right)
        self.movement = [0, 0, 0]
        
        ## \brief A Twist object representing the movement of the car
        self.move = Twist()
        
        ## \brief A publisher object to publish the Twist object to the '/R1/cmd_vel' topic
        self.publish_twist = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
        
        ## \brief A boolean representing whether to log the movements of the car
        self.log = log
        
        ## \brief A MovementLogger object to log the movements of the car
        self.logger = MovementLogger()
        
        ## \brief An integer representing the counter used for recording
        self.counter = 0
        
        ## \brief A float representing the adjuster used for recording
        self.adjuster = 1
        
        ## \brief A subscriber object to subscribe to the '/R1/pi_camera/image_raw' topic and call the callback function
        self.subscriber = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback)
    
    
    def callback(self, data):
        if (self.log or self.recording) and self.movement != [0, 0, 0]: 
            frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self.logger.add_entry(frame, self.movement)
            self.counter += 1
        
        self.publish_twist.publish(self.move)
    
    ## \brief Move the car forward
    def move_forward(self):
        ## \brief Set the linear speed of the car to the predefined linear speed
        self.move.linear.x = self.linear_speed
        
        ## \brief Set the angular speed of the car to 0
        self.move.angular.z = 0
        
        ## \brief Update the movement list to indicate that the car is moving forward
        self.movement = [0, 1, 0]        

    ## \brief Turn the car left
    def turn_left(self):
        ## \brief Set the angular speed of the car to the predefined angular speed
        self.move.angular.z = self.angular_speed
        
        ## \brief Set the linear speed of the car to the predefined linear speed multiplied by the adjuster
        self.move.linear.x = self.linear_speed * self.adjuster
        
        ## \brief Update the movement list to indicate that the car is turning left
        self.movement = [1, 0, 0]

    ## \brief Turn the car right
    def turn_right(self):
        ## \brief Set the linear speed of the car to the predefined linear speed multiplied by the adjuster
        self.move.linear.x = self.linear_speed * self.adjuster
        
        ## \brief Set the angular speed of the car to the negative of the predefined angular speed
        self.move.angular.z = -1 * self.angular_speed
        
        ## \brief Update the movement list to indicate that the car is turning right
        self.movement = [0, 0, 1]

    ## \brief Move the car backward
    def move_backward(self):
        ## \brief Set the linear speed of the car to the negative of the predefined linear speed
        self.move.linear.x = -1 * self.linear_speed
        
        ## \brief Set the angular speed of the car to 0
        self.move.angular.z = 0
        
        ## \brief Update the movement list to indicate that the car is moving backward
        self.movement = [0, 0, 0]

    ## \brief Stop the car
    def stop(self):
        ## \brief Set the linear speed and angular speed of the car to 0
        self.move.linear.x = 0
        self.move.angular.z = 0
        
        ## \brief Update the movement list to indicate that the car has stopped
        self.movement = [0, 0, 0]