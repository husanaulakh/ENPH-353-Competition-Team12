# packages
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
from geometry_msgs.msg import Twist
from MovementLogger import MovementLogger

# best 0.3 1.095

class CarActions:
    def __init__(self, linear_speed=0.3, angular_speed=1, log=False):
        self.bridge = CvBridge() # Create a CvBridge object to convert sensor_msgs/Image type to cv2 image
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.movement = [0, 0, 0] # Forward, Left, Right
        self.move = Twist()
        self.publish_twist = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
        self.log = log
        self.logger = MovementLogger()
        self.recording = False
        self.adjuster = 1
        self.subscriber = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)

    def callback(self, data):
        if self.log and self.movement != [0, 0, 0]: 
            frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            # if self.recording:
        # if self.counter % 2 == 0:
            self.logger.add_entry(frame, self.movement)
        
        self.publish_twist.publish(self.move)

    def record(self):
        self.recording = True
    
    def stopRecording(self):
        self.recording = False
    
    def move_forward(self):
        self.move.linear.x = self.linear_speed
        self.move.angular.z = 0
        self.movement = [0, 1, 0]        

    def turn_left(self):
        self.move.angular.z = self.angular_speed
        self.move.linear.x = self.linear_speed * self.adjuster
        self.movement = [1, 0, 0]

    def turn_right(self):
        self.move.linear.x = self.linear_speed * self.adjuster
        self.move.angular.z = -1 * self.angular_speed
        self.movement = [0, 0, 1]

    def move_backward(self):
        self.move.linear.x = -1 * self.linear_speed
        self.move.angular.z = 0
        self.movement = [0, 0, 0]

    def stop(self):
        self.move.linear.x = 0
        self.move.angular.z = 0
        self.movement = [0, 0, 0]