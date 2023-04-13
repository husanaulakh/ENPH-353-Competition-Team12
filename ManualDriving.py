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

class ManualDrive:
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

        img = np.copy(frame)
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define original HSV range to mask
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
            
            # Define new HSV range to mask
            new_lower_range = np.array([40, 0, 80])
            new_upper_range = np.array([129, 80, 100])
            
            # Create binary mask with new HSV range
            new_mask = cv2.inRange(cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV), new_lower_range, new_upper_range)
            
            # Find the contours in the masked image
            contours, hierarchy = cv2.findContours(new_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Extract the contour with the largest area
            max_area = 0
            max_contour = None
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > max_area:
                    max_area = area
                    max_contour = contour

            # Create a binary mask for the largest contour
            mask = np.zeros_like(new_mask)
            cv2.drawContours(new_mask, [max_contour], 0, 255, -1)

            # Extract the masked region from the original image
            masked_img = cv2.bitwise_and(cropped_img, cropped_img, mask=mask)

            # Draw the extracted region on the original image using Matplotlib
            fig, ax = plt.subplots()
            ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
            rect = cv2.boundingRect(max_contour)
            x, y, w, h = rect
            x += combined_rect[0]
            y += combined_rect[1]
            ax.add_patch(plt.Rectangle((x, y), w, h, fill=False, edgecolor='red', linewidth=2))
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