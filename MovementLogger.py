## @file
#  @brief A file containing the MovementLogger class that logs frames and movement commands
#  @details This file contains the definition of the MovementLogger class, which is used to log frames and movement commands.
#  @author [Author Name]

import os
import cv2
import numpy as np
from datetime import datetime

## @class MovementLogger
#  @brief A class that logs frames and movement commands
#  @details The MovementLogger class is used to log frames and movement commands for a robot.
class MovementLogger:
    
    ## @brief Initializes the MovementLogger object
    #  @details This method initializes a new instance of the MovementLogger class with default values.
    def __init__(self):
        ## @brief A list representing the log of frames and movement commands
        self.log = []
        
        ## @brief A string representing the folder name where the images will be saved
        self.image_folder = 'whole_loop_L0.27_A0.9'
        
        ## @brief Create the folder to save the images if it doesn't exist
        if not os.path.exists(self.image_folder):
            os.makedirs(self.image_folder)
    
    ## @brief Adds a new entry to the logger
    #  @details This method adds a new entry to the logger with the given frame and movement command.
    #  @param frame The current frame to be logged
    #  @param movement A 3-element numpy array indicating the movement command [leftTurn, forward, rightTurn]
    def add_entry(self, frame, movement):
        ## @brief Get the current date and time as a string
        date_string = datetime.now().strftime('%Y-%m-%d_%H-%M-%S-%f')
        
        ## @brief Generate a filename for the image using the current date and time and the movement command
        filename = os.path.join(self.image_folder, f'{np.argmax(movement)}-{date_string}.jpeg')
        
        ## @brief Write the image to a file with the generated filename
        cv2.imwrite(filename, frame)
        
        ## @brief Add the filename and movement command to the log list
        self.log.append((filename, *np.array(movement)))
