#! /usr/bin/env python3

# packages
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
from Car_actions import CarActions

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
        self.CarActions = CarActions(log=True)
        self.subscriber = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)
        

    def callback(self,data):
        """
        Callback function to process key events and control robot's movement.

        @param data The Image message received from the camera
        """
        key = cv2.waitKey(1) & 0xFF
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        cv2.imshow("*camera_view*", frame)
        cv2.waitKey(3)

        # move forward when up arrow key is pressed
        if key == 82: self.CarActions.move_forward()

        # turn left when left arrow key is pressed
        elif key == 81: self.CarActions.turn_left()

        # turn right when right arrow key is pressed
        elif key == 83: self.CarActions.turn_right()
        
        # move backward when down arrow key is pressed
        elif key == 84: self.CarActions.move_backward()

        # stop when spacebar is pressed
        elif key == ord(" "): self.CarActions.stop()

        elif key == ord("r"): self.CarActions.record()
        
        elif key == ord("s"): self.CarActions.stopRecording()

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