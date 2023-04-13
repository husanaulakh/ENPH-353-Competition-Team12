#! /usr/bin/env python3

# packages
import cv2
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy
from Car_actions import CarActions
import numpy as np
<<<<<<< HEAD
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
=======
>>>>>>> husan

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
<<<<<<< HEAD
        self.CarActions = CarActions(log=False)
=======
        self.CarActions = CarActions(linear_speed=0.27,angular_speed=1,log=True)
>>>>>>> husan
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

        # # move forward when up arrow key is pressed
        # if key == ord("i"): self.CarActions.move_forward()

        # # turn left when left arrow key is pressed
        # elif key == ord("j"): self.CarActions.turn_left()

        # # turn right when right arrow key is pressed
        # elif key == ord("l"): self.CarActions.turn_right()
        
        # # move backward when down arrow key is pressed
        # elif key == ord("k"): self.CarActions.move_backward()

        # stop when spacebar is pressed
        elif key == ord(" "):
            self.CarActions.stop()

        elif key == ord("r"):
            self.CarActions.record()

<<<<<<< HEAD
        elif key == ord("s"):
            self.CarActions.stopRecording()

        img = np.copy(frame)
        
=======
        # self.license_plate(frame)
        self.findlicenseplate(frame)

        # elif key == ord("r"): self.CarActions.record()
        
        # elif key == ord("s"): self.CarActions.stopRecording()


    def license_plate(self,frame):

        img = np.copy(frame)

>>>>>>> husan
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
<<<<<<< HEAD
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
=======
            if max_contour is not None:
                cv2.drawContours(mask, [max_contour], 0, 255, -1)

            # cv2.drawContours(new_mask, [max_contour], 0, 255, -1)

            # Extract the masked region from the cropped image
            masked_img = cv2.bitwise_and(cropped_img, cropped_img, mask=mask)
            # if masked_img.shape[0] >= 20 and masked_img.shape[1] <= 20 and masked_img.shape[0] <= 100 and masked_img.shape[1] <= 120:

            if True:
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

                # Crop the rectangle part from the cropped image
                if combined_rect is not None:
                    x, y, w, h = combined_rect
                    masked_img = cropped_img[y:y+h, x:x+w]
                    masked_height, masked_width, channels = masked_img.shape
                    masked_area = int(masked_height*masked_width)
                    if masked_height < 45 and masked_height > 20 and masked_width > 80 and masked_width < 190:
                        cv2.imshow("license plate", masked_img)
                        cv2.waitKey(3)

    def licenseplate(self, frame, lower_range, upper_range, new_lower_range, new_upper_range, area_threshold):
        img = np.copy(frame)
        boolean = False # checks if an image comes out (starts on No for default)

        # Convert to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Defined original HSV range to mask

        # Create binary mask
        mask = cv2.inRange(hsv, lower_range, upper_range)

        # Find contours in the binary mask
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Combine the bounding rectangles of all contours with an area greater than 200
        combined_rect = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > area_threshold:
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
            
            # Defined new HSV range to mask
            
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
            if max_contour is not None:
                cv2.drawContours(mask, [max_contour], 0, 255, -1)

            # Extract the masked region from the cropped image
            masked_img = cv2.bitwise_and(cropped_img, cropped_img, mask=mask)

            # Combine the bounding rectangles of all contours with an area greater than 200
            combined_rect = None
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > area_threshold:
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

            # Crop the rectangle part from the cropped image
            if combined_rect is not None:
                x, y, w, h = combined_rect
                masked_img = cropped_img[y:y+h, x:x+w]
                masked_height, masked_width, channels = masked_img.shape
                masked_area = int(masked_height*masked_width)
                if masked_height < 45 and masked_height > 20 and masked_width > 80 and masked_width < 190:
                    cv2.imshow("license plate", masked_img)
                    cv2.waitKey(3)
                    boolean = True

        return boolean

    def findlicenseplate(self, frame):
        # Define original HSV range to mask
        lower_range1 = np.array([0, 0, 100]) # P1 and P2
        upper_range1 = np.array([0, 0, 105]) # P1 and P2
        lower_range2 = np.array([0, 0, 190]) # P4 and P5
        upper_range2 = np.array([0, 0, 230]) # P4 and P5
        lower_range3 = np.array([0, 0, 105]) # P3
        upper_range3 = np.array([0, 0, 125]) # P3

        # Define new HSV range to mask
        new_lower_range1 = np.array([40, 0, 80])    # P1 and P2
        new_upper_range1 = np.array([129, 80, 100]) # P1 and P2
        new_lower_range2 = np.array([10, 0, 100])   # P4 and P5
        new_upper_range2 = np.array([120, 20, 180]) # P4 and P5
        new_lower_range3 = np.array([40, 0, 80])    # P3
        new_upper_range3 = np.array([129, 80, 110]) # P3

        # Define threshold for area
        area_threshold1 = 200 # P1, P2, P4, and P5
        area_threshold2 = 2000 # P3

        if not (self.licenseplate(frame, lower_range3, upper_range3, new_lower_range3, new_upper_range3, area_threshold2)):
            self.licenseplate(frame, lower_range1, upper_range1, new_lower_range1, new_upper_range1, area_threshold1)
            self.licenseplate(frame, lower_range2, upper_range2, new_lower_range2, new_upper_range2, area_threshold1)
>>>>>>> husan

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