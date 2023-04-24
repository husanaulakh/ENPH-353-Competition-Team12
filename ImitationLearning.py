#!/usr/bin/env python3
##
## \file Imitator.py
## \brief A brief description of the file.
##
## A more detailed description of the file.
##
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
import tensorflow as tf
from Car_actions import CarActions
import time
from LicenseLogger import LicenseLogger
from std_msgs.msg import String

## \class Imitator
#  \brief A class for imitating the movement of a car based on image input.
#
#  A more detailed description of the class.
class Imitator:

    ## \brief The constructor for Imitator.
    #
    #  \details Initializes an instance of the Imitator class.
    def __init__(self):
        self.bridge = CvBridge()
        self.drivingModel = tf.keras.models.load_model('TrainedImitator_wholeLoop.h5')
        self.characterModel = tf.keras.models.load_model('characterRecogModel.h5')
        self.CarActions = CarActions(linear_speed=0.27, angular_speed=0.9, log=False)
        self.licenseLogger = LicenseLogger()
        self.detected_redline = False
        self.pedestrianCrossed = False
        self.binary_mask = np.zeros(10)
        self.pedestrianCheck = None
        self.specialState = False
        self.start_time = time.time()
        self.countPedestrianCrossings = 0
        self.count = 0
        self.licensePlateCounter = 1
        self.beginCourse = time.time()
        self.licensePublisher = rospy.Publisher('/license_plate', String, queue_size=1)    
        rospy.loginfo("Timer started.")
        
        #self.clock_sub = rospy.Subscriber('/clock', Clock, self.clock_callback)
        time.sleep(0.2)
        self.timerstopped = False
        
        self.licensePublisher.publish(String('HUSH,BarbieBot,0,HAJLD12')) #Start Time
        self.subscriber = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback)

    ## \brief A method for processing the image and controlling the robot's movement.
    #
    #  \details This method is called as a callback for the image received from the camera.
    #  @param data The Image message received from the camera.
    def callback(self, data):

        if time.time() - self.beginCourse > 39:
            
            if not self.timerstopped:
                rospy.loginfo("Timer stopped.")
                self.licensePublisher.publish(str('TeamRed,multi21,-1,XR58'))
                self.timerstopped = True

        
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        self.predictCharacters(frame)
        
        if self.specialState:
            self.continueDriving(frame)
            # cv2.imshow("Imitator", frame)
            # cv2.waitKey(7)
            return
        elif self.countPedestrianCrossings < 2:
            self.checkLineHSV(frame)
        
        if self.detected_redline:
            if self.pedestrianCrossed:
                self.specialState = True  
                self.detected_redline = False
                self.pedestrianCrossed = False
            else:
                self.CarActions.stop()
                self.pedestrian(frame)
        else:
            self.predictionDrive(frame)
            if self.licensePlateCounter == 1:
                license_img = self.findlicenseplate(frame)
                if license_img is not None and self.licensePlateCounter == 1:
                    self.predictCharacters(license_img)
            
            # cv2.imshow("Imitator", frame)
            # cv2.waitKey(7)
        


    ## \brief A method for predicting characters from an image.
    #
    #  \details This method predicts characters from an image and publishes them to a topic.
    #  @param frame The image to predict characters from.
    def predictCharacters(self, frame):
        # Define a function to convert a one-hot vector to a string

        # if self.licensePlateCounter < 1:
        chars = self.getCharacters(frame)
        if len(chars) == 4:
            plate = ""
            for character in chars:
                character = cv2.resize(character, (30,36))
                img_aug = np.expand_dims(character, axis=0)
                one_hot = self.characterModel.predict(img_aug)
                plate += str(self.onehot_to_string(one_hot))

            # self.licensePublisher.publish(str('HUSH,Ken,{},{}', str(self.licensePlateCounter), plate))
            # self.licensePublisher.publish('HUSH,Ken,{},{}}'.format(str(self.licensePlateCounter), plate))
            if self.licensePlateCounter == 1:
                self.licensePublisher.publish('HUSH,BarbieBot,{},{}'.format(str(self.licensePlateCounter), plate))
                self.licensePlateCounter +=1
                print(plate)

    ## \brief A method for converting a one-hot vector to a string.
    #
    #  \details This method converts a one-hot vector to a string using a dictionary of indices to characters.
    #  @param onehot The one-hot vector to convert.
    #  @return The corresponding character from the dictionary.
    def onehot_to_string(self, onehot):
        # Define a dictionary mapping indices to characters
        char_dict = {
            0: "A", 1: "B", 2: "C", 3: "D", 4: "E", 5: "F", 6: "G", 7: "H", 8: "I",
            9: "J", 10: "K", 11: "L", 12: "M", 13: "N", 14: "O", 15: "P", 16: "Q", 17: "R",
            18: "S", 19: "T", 20: "U", 21: "V", 22: "W", 23: "X", 24: "Y", 25: "Z",
            26: "0", 27: "1", 28: "2", 29: "3", 30: "4", 31: "5", 32: "6", 33: "7", 34: "8", 35: "9"
        }

        # Find the index of the maximum value in the one-hot vector
        index = np.argmax(onehot)

        # Return the corresponding character from the dictionary
        return char_dict[index]


    ## \brief A method for extracting characters from an image.
    #
    #  \details This method extracts characters from an image by first converting it to the HSV color space,
    #  thresholding the image, and then finding the connected components. The four largest components are then
    #  cropped out and returned as a list of images.
    #  @param frame The image to extract characters from.
    #  @return A list of images containing the extracted characters.
    def getCharacters(self, frame):
        
        # Convert image to HSV color space
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        _, mask = cv2.threshold(grey, 55, 255, cv2.THRESH_BINARY)
        mask = cv2.bitwise_not(mask)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)

        # Sort the connected components by their areas in descending order
        sorted_stats = sorted(stats, key=lambda x: x[4], reverse=True)

        # Pick out the four largest components
        largest_components = sorted_stats[1:5]
        x_arr = [] 
        chars = []
    
        # Crop out each of the four largest components using their bounding boxes
        for i, (x, y, w, h, area) in enumerate(largest_components):
            component = grey[y-3:y+h+3, x-2:x+w+2]
            if component.size != 0:
                chars.append(component)
                x_arr.append(x)
            
        picture_index_pairs = zip(chars, x_arr)

        # sort the tuples based on the index value
        sorted_pairs = sorted(picture_index_pairs, key=lambda x: x[1])

        # extract only the pictures from the sorted tuples
        sorted_pictures = [pair[0] for pair in sorted_pairs]

        return sorted_pictures

    ## \brief A method for continuing driving.
    #
    #  \details This method continues driving the car based on image input by calling the predictionDrive and 
    #  findlicenseplate methods.
    #  @param frame The image to continue driving with.
    def continueDriving(self, frame):
        if time.time() - self.start_time > 3:
            self.specialState = False
        self.predictionDrive(frame)
        self.findlicenseplate(frame)

        

    ## \brief A method for detecting pedestrians in an image and controlling the robot's movement.
    #
    #  \details This method applies a color threshold to extract the red pixels in an image and then detects the largest
    #  contour in the resulting binary mask. It draws a bounding rectangle around the contour and checks if the pedestrian
    #  has crossed the red line.
    #  @param frame The image to detect pedestrians in.
    def pedestrian(self, frame):
        # Apply a color threshold to extract the red pixels
        # lower_red = (71, 56, 38)
        # upper_red = (75, 60, 44)
        lower_red = (65, 50, 32)
        upper_red = (75, 60, 44)
        # mask_red = cv2.inRange(frame[100:600,200:1000], lower_red, upper_red)
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
            contour_size = (200, 100)

            # Calculate the center of the bounding rectangle
            center_x = x + w // 2
            center_y = y + h // 2

            # Calculate the top-left corner of the contour
            contour_x = center_x - contour_size[1] // 2
            contour_y = center_y - contour_size[0] // 2

            cv2.rectangle(frame, (contour_x, contour_y), (contour_x + contour_size[1], contour_y + contour_size[0]), (0, 255, 0), 3)
            if x > 525 and x < 675:
                self.pedestrianCrossed = True
                print("Move forward")
                self.countPedestrianCrossings += 1
                self.start_time = time.time()
                self.pedestrianCheck = None

    ## \brief A method for predicting the movement of a car based on an image input.
    #
    #  \details This method predicts the movement of a car based on an image input and controls the robot's movement
    #  accordingly. It uses a deep learning model to predict the next action based on the image input.
    #  @param frame The image to predict the movement of the car from.
    def predictionDrive(self, frame):
        self.count += 1
        # if self.count % 2 == 1:
        action = self.getPrediction(frame)
        if np.array_equal(action, [0, 1, 0]):  # move forward when up arrow key is pressed
            self.CarActions.move_forward()

        elif np.array_equal(action, [1, 0, 0]):  # turn left when left arrow key is pressed
            self.CarActions.turn_left()

        elif np.array_equal(action, [0, 0, 1]):  # turn right when right arrow key is pressed
            self.CarActions.turn_right()

        else:
            self.CarActions.stop()


    ## \brief A method for checking the presence of a red line in an image.
    #
    #  \details This method checks whether a red line is present in a given image using HSV color space, 
    #  and updates the value of a flag accordingly.
    #  @param frame The image to check for a red line.
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
            if area > 10000 and area < 22000:
                # print(area)
                approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
                # if len(approx) == 4:
                count +=1
                # if count >= 2:
                self.detected_redline = True
                print("Line detected")

                cv2.drawContours(frame, [approx], 0, (0, 255, 0), 3)

    ## \brief A method for making a prediction based on an input frame.
    #
    #  \details This method makes a prediction based on an input frame, and returns an array of 3 elements representing the predicted action.
    #  @param frame The input frame to make a prediction on.
    #  @return An array of 3 elements representing the predicted action.
    def getPrediction(self, frame):
        img_aug = cv2.resize(frame, (frame.shape[1] // 10, frame.shape[0] // 10))  # resize by 10x
        img_aug = np.expand_dims(img_aug, axis=0)

        prediction = self.drivingModel.predict(img_aug)[0]

        max_index = np.argmax(prediction)
        action = [0, 0, 0]
        action[max_index] = 1

        return action
    
    ## \brief A method for generating a binary mask of a license plate in an image.
    #
    #  \details This method generates a binary mask of a license plate in an image using a specified color range.
    #  @param frame The image to generate a binary mask from.
    #  @param lower_range The original lower range of the color to mask.
    #  @param upper_range The original upper range of the color to mask.
    #  @param new_lower_range The new lower range of the color to mask.
    #  @param new_upper_range The new upper range of the color to mask.
    #  @param area_threshold The threshold area for contours to include in the mask.
    #  @return A binary mask of the license plate in the image.
    def getLicenseMask(self, frame, lower_range, upper_range, new_lower_range, new_upper_range, area_threshold):
        return_img = None
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
                    # cv2.imshow("license plate", masked_img)
                    # cv2.waitKey(7)
                    # self.licenseLogger.add_entry(masked_img)
                    boolean = True
                    return_img = masked_img


        return return_img

    ## \brief A method for finding a license plate in an image.
    #
    #  \details This method applies HSV masks to an image in order to isolate the region where the license plate is likely to be found.
    #  @param frame The image to search for the license plate in.
    #  @return The masked image containing the license plate, or None if the license plate was not found.
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
        img = self.getLicenseMask(frame, lower_range3, upper_range3, new_lower_range3, new_upper_range3, area_threshold2)
        if img is None:
            img = self.getLicenseMask(frame, lower_range1, upper_range1, new_lower_range1, new_upper_range1, area_threshold1)
            if img is None:
                img = self.getLicenseMask(frame, lower_range2, upper_range2, new_lower_range2, new_upper_range2, area_threshold1)
                    
        return img


        # if not (self.licenseplate(frame, lower_range3, upper_range3, new_lower_range3, new_upper_range3, area_threshold2)):
        #     if not (self.licenseplate(frame, lower_range1, upper_range1, new_lower_range1, new_upper_range1, area_threshold1)):
        #         self.licenseplate(frame, lower_range2, upper_range2, new_lower_range2, new_upper_range2, area_threshold1)


    ## \brief Shutdown hook function to stop the robot movement and save the log before shutting down.
    #
    #  \details This function stops the robot movement and saves the log before shutting down the program.
    #  @param self The object pointer.
    def shutdown_hook(self):
        self.CarActions.stop()


if __name__ == '__main__':
    ## \brief Initializes the ROS node for the topic publisher.
    rospy.init_node('topic_publisher')

    ## \brief Initializes an instance of the Imitator class.
    robot = Imitator()

    ## \brief Registers a shutdown hook to stop the robot movement and save the log before shutting down.
    rospy.on_shutdown(robot.shutdown_hook)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting off")

    ## \brief Destroys all open windows when the program terminates.
    cv2.destroyAllWindows()