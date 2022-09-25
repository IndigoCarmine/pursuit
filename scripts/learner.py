#!/usr/bin/python python
from itertools import count
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
class Learner:

    def __init__(self):
        self.image_subscriber = rospy.Subscriber('/usb_cam/image_raw', Image, queue_size=1,callback=self.callback)
        self.cv_bridge = CvBridge()
        self.counter = 0
        rospy.loginfo("Learner is initialized.")
        rospy.loginfo(cv2.__version__)
        os.mkdir('data')
        
    def callback(self,data):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)
        
        
        if cv2.imwrite('data/'+str(self.counter) + '.png',cv_image):
            rospy.loginfo("can save a image.")
            self.counter+=1

        else:
            rospy.loginfo("cannot save a image.")
            
        rospy.sleep(1)

def main():
    rospy.init_node('learning_node', anonymous=True)
    ln = Learner()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutdown now")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
