#! /usr/bin/env python3

# packages
import cv2
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy
from Car_actions import CarActions
import numpy as np
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from ManualDriving import ManualDrive

class LicensePlateDetection:
    """
    Class to manually drive a robot using arrow keys

    @param self The object pointer
    """
  
    def __init__(self):
        """
        Initializes an instance of the ManualDrive class.
        """

        self.bridge = CvBridge() # Create a CvBridge object to convert sensor_msgs/Image type to cv2 image
        self.CarActions = CarActions(log=False)
        self.subscriber = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)
        

    def callback(self, data):
        """
        Callback function to process key events and control robot's movement.

        @param data The Image message received from the camera
        """
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        # Display camera feed as you drive
        cv2.imshow("*camera_view*", frame)

        # Wait for a key event
        key = cv2.waitKey(10)

        # move forward when up arrow key is pressed
        if key == 82:
            self.CarActions.move_forward()

        # turn left when left arrow key is pressed
        elif key == 81:
            self.CarActions.turn_left()

        # turn right when right arrow key is pressed
        elif key == 83:
            self.CarActions.turn_right()

        # move backward when down arrow key is pressed
        elif key == 84:
            self.CarActions.move_backward()

        # stop when spacebar is pressed
        elif key == ord(" "):
            self.CarActions.stop()

        elif key == ord("r"):
            self.CarActions.record()

        elif key == ord("s"):
            self.CarActions.stopRecording()

        # Copy the frame
        img = np.copy(frame)

        # Convert to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define HSV range to mask
        lower_range = np.array([0, 0, 100])
        upper_range = np.array([0, 0, 105])

        # Create binary mask
        mask = cv2.inRange(hsv, lower_range, upper_range)

        # Find contours in the binary mask
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Combine the bounding rectangles of all contours with an area greater than 200
        combined_rect = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 200:
                rect = cv2.boundingRect(contour)
                if combined_rect is None:
                    combined_rect = rect
                else:
                    x, y, w, h = rect
                    x_min = min(combined_rect[0], x)
                    y_min = min(combined_rect[1], y)
                    x_max = max(combined_rect[0] + combined_rect[2], x + w)
                    y_max = max(combined_rect[1] + combined_rect[3], y + h)
                    combined_rect = (x_min, y_min, x_max - x_min, y_max - y_min)

        # Crop the rectangle part from the original image
        if combined_rect is not None:
            x, y, w, h = combined_rect
            cropped_img = img[y:y+h, x:x+w]
            # Display the cropped image using Matplotlib
            plt.imshow(cv2.cvtColor(cropped_img, cv2.COLOR_BGR2RGB))
            plt.show()

    def shutdown_hook(self):
        """
        Shutdown hook function to stop the robot movement and save the log before shutting down.
        """
        self.CarActions.stop()


if __name__ == '__main__':
    rospy.init_node('topic_publisher')
    robot = ManualDrive()
    rospy.on_shutdown(robot.shutdown_hook)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting off")
    cv2.destroyAllWindows()