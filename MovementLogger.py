import os
import cv2
import numpy as np
from datetime import datetime

class MovementLogger:
    def __init__(self):
        self.log = []
        self.image_folder = 'best_training_outerloopV2'
        if not os.path.exists(self.image_folder):
            os.makedirs(self.image_folder)
    
    def add_entry(self, frame, movement):
        """
        Adds a new entry to the logger
        
        @param frame: the current frame to be logged
        @param movement: a 3-element numpy array indicating the movement command
                         [leftTurn, forward, rightTurn]
        """

        date_string = datetime.now().strftime('%Y-%m-%d_%H-%M-%S-%f')
        filename = os.path.join(self.image_folder, f'{np.argmax(movement)}-{date_string}.jpeg')
        cv2.imwrite(filename, frame)
        self.log.append((filename, *np.array(movement)))
