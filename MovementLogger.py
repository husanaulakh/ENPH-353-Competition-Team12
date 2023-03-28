import csv
import os
import cv2
import numpy as np
from datetime import datetime

class MovementLogger:
    def __init__(self):
        self.log = []
        self.image_folder = 'training_images'
        if not os.path.exists(self.image_folder):
            os.makedirs(self.image_folder)
    
    def add_entry(self, frame, movement):
        """
        Adds a new entry to the logger
        
        @param frame: the current frame to be logged
        @param movement: a 4-element numpy array indicating the movement command
                         [leftTurn, forward, rightTurn, stop]
        """

        date_string = datetime.now().strftime('%Y-%m-%d_%H-%M-%S-%f')
        filename = os.path.join(self.image_folder, f'{np.argmax(movement)}-{date_string}.jpeg')
        cv2.imwrite(filename, frame)
        # movement = movement.astype(int) # convert one-hot vector to integers
        self.log.append((filename, *np.array(movement)))

    # def save(self, filename):
    #     """
    #     Saves the log to a CSV file
        
    #     @param filename: the name of the file to be saved
    #     """
    #     with open(filename, 'w', newline='') as csvfile:
    #         writer = csv.writer(csvfile)
    #         # writer.writerow(['filename', 'leftTurn', 'forward', 'rightTurn', 'stop'])
    #         for entry in self.log:
    #             writer.writerow(entry)


# import csv
# import os
# import cv2
# import numpy as np

# class MovementLogger:
#     def __init__(self):
#         self.log = []
#         self.image_folder = 'training_images'
#         if not os.path.exists(self.image_folder):
#             os.makedirs(self.image_folder)
    
#         # if os.path.exists(self.image_folder):
#         #     with open(self.image_folder, 'r') as f:
#         #         reader = csv.reader(f)
#         #         for row in reader:
#         #             self.log.append(row)

#     def add_entry(self, frame, movement):
#         """
#         Adds a new entry to the logger
        
#         @param frame: the current frame to be logged
#         @param movement: a 4-element numpy array indicating the movement command
#                          [leftTurn, forward, rightTurn, stop]
#         """

#         filename = os.path.join(self.image_folder, f'{len(self.log):06d}.jpeg')
#         cv2.imwrite(filename, frame)
#         # movement = movement.astype(int) # convert one-hot vector to integers
#         self.log.append((filename, *np.array(movement).tolist()))

#     def save(self, filename):
#         """
#         Saves the log to a CSV file
        
#         @param filename: the name of the file to be saved
#         """
#         with open(filename, 'w', newline='') as csvfile:
#             writer = csv.writer(csvfile)
#             # writer.writerow(['filename', 'leftTurn', 'forward', 'rightTurn', 'stop'])
#             for entry in self.log:
#                 writer.writerow(entry)