#!/usr/bin/env python3
##
## \file video_recorder.py
## \brief A brief description of the file.
##
## A more detailed description of the file.
##

## \package video_recorder
#  \brief A brief description of the package.
#
#  A more detailed description of the package.
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image

## \class Video_recorder
#  \brief A brief description of the class.
#
#  A more detailed description of the class.
class Video_recorder:

    ## \brief The constructor for Video_recorder.
    #
    #  \details More detailed information about the constructor.
    def __init__(self):
        self.bridge = CvBridge()
        self.subscriber = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback)
        self.out = None

    ## \brief A brief description of the callback method.
    #
    #  \details A more detailed description of the callback method.
    #  @param data The data received from the subscriber.
    def callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        if self.out is None:
            # Define the video codec and create VideoWriter object
            height, width, _ = frame.shape
            self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.out = cv2.VideoWriter('output.mp4', self.fourcc, 10.0, (width, height))

        # Write the frame to the video
        self.out.write(frame)

        # Show the frame
        cv2.imshow("camera view", frame)
        cv2.waitKey(1)

    ## \brief A brief description of the stop method.
    #
    #  \details A more detailed description of the stop method.
    def stop(self):
        # Release the VideoWriter object
        self.out.release()

if __name__ == '__main__':
    video = Video_recorder()
    rospy.init_node('topic_publisher')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting off")
    cv2.destroyAllWindows()
