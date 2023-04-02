#! /usr/bin/env python3

# packages
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
from geometry_msgs.msg import Twist
from MovementLogger import MovementLogger
import tensorflow as tf
from tensorflow import keras
from keras import layers, models, optimizers, backend
from keras.utils import plot_model

class Imitator:

  
    def __init__(self):
        self.bridge = CvBridge() # Create a CvBridge object to convert sensor_msgs/Image type to cv2 image
        self.model = tf.keras.models.load_model('TrainedImitatorV2.h5')
        self.linear_speed = 0.3
        self.angular_speed = 0.7
        self.publish_twist = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
        self.move = Twist()
        self.subscriber = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)

    def callback(self,data):
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        cv2.imshow("Imitator", frame)
        cv2.waitKey(3)

        img_aug = cv2.resize(frame, (frame.shape[1] // 10, frame.shape[0] // 10)) # resize by 10x
        img_aug = np.expand_dims(img_aug, axis=0)

        prediction = self.model.predict(img_aug)[0]

        max_index = np.argmax(prediction)
        action = [0, 0, 0]
        action[max_index] = 1

        if np.array_equal(action, [0, 1, 0]): # move forward when up arrow key is pressed
            self.move.linear.x = self.linear_speed
            self.move.angular.z = 0

        elif np.array_equal(action, [1, 0, 0]): # turn left when left arrow key is pressed
            self.move.angular.z = self.angular_speed
            self.move.linear.x = self.linear_speed*0.6

        elif np.array_equal(action, [0, 0, 1]): # turn right when right arrow key is pressed
            self.move.linear.x = self.linear_speed/2
            self.move.angular.z = -1*self.angular_speed*0.6
        else:
            self.move.linear.x = 0
            self.move.angular.z = 0



        # if action == [0, 1, 0]: # move forward when up arrow key is pressed
        #     self.move.linear.x = self.linear_speed
        #     self.move.angular.z = 0
        
        # elif action == [1, 0, 0]: # turn left when left arrow key is pressed
        #     self.move.angular.z = self.angular_speed
        #     self.move.linear.x = self.linear_speed*0.6

        # elif action == [0, 0, 1]: # turn right when right arrow key is pressed
        #     self.move.linear.x = self.linear_speed/2
        #     self.move.angular.z = -1*self.angular_speed*0.6
        # else:
        #     self.move.linear.x = 0
        #     self.move.angular.z = 0
        print(action)
        
        self.publish_twist.publish(self.move)
    
    def shutdown_hook(self):
        """
        Shutdown hook function to save the log before shutting down
        """
        self.move.linear.x = 0
        self.move.angular.z = 0
        self.publish_twist.publish(self.move)


if __name__ == '__main__':
    rospy.init_node('topic_publisher')
    robot = Imitator()
    rospy.on_shutdown(robot.shutdown_hook)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting off")
    cv2.destroyAllWindows()