## @package pyexample
#  Documentation for this module.
#
#  More details.

import os
import cv2
import numpy as np
from datetime import datetime

## Documentation for the LicenseLogger class.
#
#  More details.
class LicenseLogger:
    
    ## The constructor.
    def __init__(self):
        ## @var log
        #  A list representing the log of license plate images.
        self.log = []
        
        ## @var image_folder
        #  A string representing the folder name where the license plate images will be saved.
        self.image_folder = 'LicensePlates'
        
        ## Create the folder to save the license plate images if it doesn't exist.
        if not os.path.exists(self.image_folder):
            os.makedirs(self.image_folder)
    
    ## Adds a new entry to the logger.
    #
    #  @param frame The current frame to be logged.
    def add_entry(self, frame):
        ## Get the current date and time as a string.
        date_string = datetime.now().strftime('%Y-%m-%d_%H-%M-%S-%f')
        
        ## Generate a filename for the image using the current date and time.
        filename = os.path.join(self.image_folder, f'{date_string}.jpeg')
        
        ## Write the image to a file with the generated filename.
        cv2.imwrite(filename, frame)
        
        ## Add the filename to the log list.
        self.log.append((filename,))