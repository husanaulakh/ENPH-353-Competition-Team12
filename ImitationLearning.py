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
        self.pedestrianCrossed = False
        self.binary_mask = np.zeros(10)
        self.pedestrianCheck = None
        self.checkRed = True
        self.checkRedCounter = 0
        self.specialState = False
        self.start_time = time.time()
        self.logger = MovementLogger()
        self.subscriber = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback)

    def callback(self, data):
        """
        Callback function to process the image and control the robot's movement.

        @param data: The Image message received from the camera.
        """
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        # self.checkLineHSV(frame)
        
        if self.specialState:
            self.continueDriving(frame)
            return
        else:
            self.checkLineHSV(frame)
        
        if self.detected_redline:
            if self.pedestrianCrossed:
                self.specialState = True  
                self.detected_redline = False
            else:
                self.CarActions.stop()
                # time.sleep(1)
                self.pedestrian(frame)
                # self.pedestrianCrossing(frame)
        else:
            self.predictionDrive(frame)
        
        cv2.imshow("Imitator", frame)
        cv2.waitKey(3)


    def continueDriving(self, frame):
        
        if time.time() - self.start_time < 2:
            self.specialState = False
        
        self.predictionDrive(frame)
    
    def pedestrian(self, frame):
        
        # Apply a color threshold to extract the red pixels
        lower_red = (71, 56, 38)
        upper_red = (75, 60, 44)
        mask_red = cv2.inRange(frame, lower_red, upper_red)

        # Apply a binary threshold to create a binary mask
        _, binary_mask = cv2.threshold(mask_red, 120, 255, cv2.THRESH_BINARY)

        # Find the contours in the binary mask
        contours, _ = cv2.findContours(binary_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Sort the contours by their area (largest to smallest)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        if len(contours) != 0:
            # Get the second largest contour
            largest_contour = contours[0]

            # Draw a bounding rectangle around the largest contour
            x, y, w, h = cv2.boundingRect(largest_contour)

            # Define the size of the contour we want to draw
            contour_size = (300, 100)

            # Calculate the center of the bounding rectangle
            center_x = x + w // 2
            center_y = y + h // 2

            # Calculate the top-left corner of the contour
            contour_x = center_x - contour_size[1] // 2
            contour_y = center_y - contour_size[0] // 2

            cv2.rectangle(frame, (contour_x, contour_y), (contour_x + contour_size[1], contour_y + contour_size[0]), (0, 255, 0), 3)
            if x > 550 and x < 650:
                self.pedestrianCrossed = True
                print("Move forward")
                self.start_time = time.time()
                self.pedestrianCheck = None


    def pedestrianCrossing(self, frame):
        height, width, _ = frame.shape

        middle = width // 2
        if self.pedestrianCheck is None:
            time.sleep(2)
            prev = frame[:, middle-50:middle+50]
            self.pedestrianCheck = frame[:, middle-50:middle+50]
        else:
            prev = self.pedestrianCheck
            self.pedestrianCheck = frame[:, middle-50:middle+50]
        
        if not np.array_equal(prev, self.pedestrianCheck):
            cv2.imshow("previous frame",prev)
            cv2.imshow("current frame", self.pedestrianCheck)
            self.pedestrianCrossed = True
            print("Move forward")
            self.start_time = time.time()
            self.pedestrianCheck = None


        # indices = np.where(self.binary_mask != 0)

        # if len(indices) > 1 and len(indices[1]) > 0:
        #     middle = (np.min(indices[1]) + np.max(indices[1]))//2
        #     prev = self.pedestrianCheck
        #     self.pedestrianCheck = frame[:,middle-70:middle+70]

        #     if not np.array_equal(prev, self.pedestrianCheck):
        #         self.pedestrianCrossed = True
        #         self.detected_redline = False
        #         print("wait for pedestrian to cross")
        #         self.CarActions.move_forward()
        #         time.sleep(0.5)

    def predictionDrive(self, frame):
        action = self.getPrediction(frame)
        if np.array_equal(action, [0, 1, 0]):  # move forward when up arrow key is pressed
            self.CarActions.move_forward()

        elif np.array_equal(action, [1, 0, 0]):  # turn left when left arrow key is pressed
            self.CarActions.turn_left()

        elif np.array_equal(action, [0, 0, 1]):  # turn right when right arrow key is pressed
            self.CarActions.turn_right()

        else:
            self.CarActions.stop()

    def disable_checkRed(self):
        self.checkRed = True

    def oneSecondTimer(self):
        self.checkRed = False
        print("check red off")
        time.sleep(2)
        print("Check red on")
        self.checkRed = True

    def checkLineWhitePixels(self, frame):
        
        count = 0
        frame = frame[300:, 200:1000]

        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the lower and upper range of red color in HSV
        lower_red = np.array([0, 95, 95])
        upper_red = np.array([15, 255, 255])

        # Create a mask of the red color using the defined range
        mask_red = cv2.inRange(hsv, lower_red, upper_red)

        # Apply a binary threshold to create a binary mask
        _, self.binary_mask = cv2.threshold(mask_red, 12, 255, cv2.THRESH_BINARY)
        

        # Count the number of non-zero pixels in the binary mask
        count = cv2.countNonZero(self.binary_mask)

        if count > 12000:
            self.detected_redline = True


    def checkLineHSV(self, frame):
        frame = frame[:, 200:1000]
        count = 0

        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the lower and upper range of red color in HSV
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        # Create a mask of the red color using the defined range
        mask_red = cv2.inRange(hsv, lower_red, upper_red)

        # Apply a binary threshold to create a binary mask
        _, binary_mask = cv2.threshold(mask_red, 12, 255, cv2.THRESH_BINARY)

        # Find the contours of the binary mask
        contours, _ = cv2.findContours(binary_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Loop over the contours and check for a red line contour  
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 200 and area < 22000:
                # print(area)
                approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
                # if len(approx) == 4:
                count +=1
                if count >= 2:
                    self.detected_redline = True
                    print("Line detected")

                cv2.drawContours(frame, [approx], 0, (0, 255, 0), 3)

    def checkLineRGB(self, frame):
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
