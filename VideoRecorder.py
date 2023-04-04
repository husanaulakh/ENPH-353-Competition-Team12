#! /usr/bin/env python3

# packages
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image

class Video_recorder:
    
    def __init__(self):
        self.bridge = CvBridge()
        self.subscriber = rospy.Subscriber("/R1/pi_camera/image_raw", Image, self.callback)
        self.out = None

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