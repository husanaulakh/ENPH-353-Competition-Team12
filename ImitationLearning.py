#!/usr/bin/env python3

import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
import tensorflow as tf
from Car_actions import CarActions
import time
from MovementLogger import MovementLogger


class Imitator:
    """
    Class for imitating the movement of a car based on image input.
    """

    def __init__(self):
        """
        Initializes an instance of the Imitator class.
        """
        self.bridge = CvBridge()
        self.model = tf.keras.models.load_model('TrainedImitator_Best.h5')
        self.CarActions = CarActions()
        self.detected_redline = False
        self.binary_mask = np.zeros(10)
        self.pedestrianCheck = np.zeros(10)
        self.checkRed = True
        self.checkRedCounter = 0
        self.logger = MovementLogger()
        self.subscriber = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback)

    def callback(self, data):
        """
        Callback function to process the image and control the robot's movement.

        @param data: The Image message received from the camera.
        """
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        # self.checkLine(frame)

        # if self.detected_redline and self.checkRed:
        #     self.CarActions.stop()
        #     self.pedestrianCrossing(frame)
        #     # timer_thread = threading.Thread(target=self.oneSecondTimer)
        #     # timer_thread.start()
        #     self.checkRed = False  # Disable checkRed for 2 seconds
        #     timer_thread = threading.Timer(2.0, self.disable_checkRed)
        #     timer_thread.start()

        # # if self.detected_redline:
        # #     if self.checkRed:
        # #         self.CarActions.stop()
        # #         self.pedestrianCrossing(frame)
        # #     else:
        # #         timer_thread = threading.Thread(target=self.oneSecondTimer)
        # #         timer_thread.start()
            
        # else:
        action = self.getPrediction(frame)
        if np.array_equal(action, [0, 1, 0]):  # move forward when up arrow key is pressed
            self.CarActions.move_forward()

        elif np.array_equal(action, [1, 0, 0]):  # turn left when left arrow key is pressed
            self.CarActions.turn_left()

        elif np.array_equal(action, [0, 0, 1]):  # turn right when right arrow key is pressed
            self.CarActions.turn_right()

        else:
            self.CarActions.stop()

        cv2.imshow("Imitator", frame)
        cv2.waitKey(3)
    

    def disable_checkRed(self):
        self.checkRed = True

    def oneSecondTimer(self):
        self.checkRed = False
        print("check red off")
        time.sleep(2)
        print("Check red on")
        self.checkRed = True

    def pedestrianCrossing(self, frame):
        indices = np.where(self.binary_mask != 0)

        if len(indices[1]) > 0:
            middle = (np.min(indices[1]) + np.max(indices[1]))//2
            prev = self.pedestrianCheck
            self.pedestrianCheck = frame[:,middle-10:middle+10]

            if not np.array_equal(prev, self.pedestrianCheck):
                self.detected_redline = False
                print("wait for pedestrian to cross")
                time.sleep(0.5)
                # Start the timer thread
                # timer_thread = threading.Thread(target=self.oneSecondTimer)
                # timer_thread.start()


    def checkLine(self, frame):
        frame = frame[550:600, 200:1000]
        
        # Apply a color threshold to extract the red pixels
        lower_red = (0, 0, 100)
        upper_red = (100, 100, 255)
        mask_red = cv2.inRange(frame, lower_red, upper_red)

        # Apply a binary threshold to create a binary mask
        _, self.binary_mask = cv2.threshold(mask_red, 127, 255, cv2.THRESH_BINARY)

        # Find the contours of the binary mask
        contours, _ = cv2.findContours(self.binary_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Loop over the contours and check for a red line contour  
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 100 and area < 200000:
                approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
                if len(approx) == 4:
                    self.detected_redline = True
                    print("LINE DETECTED")
                    # self.logger.add_entry(frame,[0, 0, 0])
                    break

    def getPrediction(self, frame):
        """
        Helper function to make a prediction based on an input frame.

        @param frame: The input frame to make a prediction on.
        @return: An array of 3 elements representing the predicted action.
        """
        img_aug = cv2.resize(frame, (frame.shape[1] // 10, frame.shape[0] // 10))  # resize by 10x
        img_aug = np.expand_dims(img_aug, axis=0)

        prediction = self.model.predict(img_aug)[0]

        max_index = np.argmax(prediction)
        action = [0, 0, 0]
        action[max_index] = 1

        return action

    def shutdown_hook(self):
        """
        Shutdown hook function to stop the robot movement before shutting down.
        """
        self.CarActions.stop()


if __name__ == '__main__':
    rospy.init_node('topic_publisher')
    robot = Imitator()
    rospy.on_shutdown(robot.shutdown_hook)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
