import os
import cv2
import numpy as np
from datetime import datetime

class LicenseLogger:
    def __init__(self):
        self.log = []
        self.image_folder = 'LicensePlates'
        if not os.path.exists(self.image_folder):
            os.makedirs(self.image_folder)
    
    def add_entry(self, frame):
        """
        Adds a new entry to the logger
        
        @param frame: the current frame to be logged
        """

        date_string = datetime.now().strftime('%Y-%m-%d_%H-%M-%S-%f')
        filename = os.path.join(self.image_folder, f'{date_string}.jpeg')
        cv2.imwrite(filename, frame)
        self.log.append((filename,))
