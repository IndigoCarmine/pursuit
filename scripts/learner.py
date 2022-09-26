#!/usr/bin/python python
from operator import ne
import re
import rospy
from sensor_msgs.msg import Image, Joy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
import numpy as np
from math import dist

class Learner:
    def __init__(self):
        self.image_subscriber = rospy.Subscriber('/usb_cam/image_raw', Image, queue_size=1,callback=self.callback)
        self.joy_subscriber = rospy.Subscriber('joy', Joy,queue_size=1,callback=self.joy_callback)
        self.test_publisher = rospy.Publisher('image',Image,queue_size=1)
        self.cv_bridge = CvBridge()
        self.counter = 0
        self.before_img_descriptor = None
        self.before_img_keypoints = None
        self.bf = cv2.BFMatcher()
        self.akaze = cv2.AKAZE_create()
        self.target_point = [200,200]
        self.islockon = False
        rospy.loginfo("Learner is initialized.")
        rospy.loginfo(cv2.__version__)
        os.makedirs('data', exist_ok=True)    

    def joy_callback(self,data):
        self.target_point += np.floor([-data.axes[0]*3,-data.axes[1]*3]).astype(np.int)


    def callback(self,data):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)
        

        
        # if cv2.imwrite('data/'+str(self.counter) + '.png',cv_image):
        #     rospy.loginfo("can save a image.")
        #     self.counter+=1
        # else:
        #     rospy.loginfo("cannot save a image.")     

        self.bfMatch(cv_image)
        
    def bfMatch(self,next_img):
        # キーポイントなどを見やすくするためにグレースケールで画像読み込み
        gray = cv2.cvtColor(next_img, cv2.COLOR_BGR2GRAY)
        # キーポイントの検出と特徴の記述
        kp, descriptor = self.akaze.detectAndCompute(gray, None)
        if self.before_img_descriptor is not None:
            matches = self.bf.knnMatch(descriptor, self.before_img_descriptor, k=2)
            min_distance = 100000
            near_point = (0,0)
            near_vector = [0,0]
            for m, n in matches:
                if m.distance < 0.75 * n.distance:
                    # m is a good match.
                    # next_img = cv2.circle(img=next_img,center=np.floor(near_point).astype(np.uint),radius=10,color=(255,0,0),thickness=5)

                    # 特徴点のうちターゲットの点に最も近いものを選ぶ。
                    ds = dist(self.before_img_keypoints[m.trainIdx].pt,self.target_point)
                    if min_distance > ds:
                        min_distance = ds
                        near_point = kp[m.queryIdx].pt
                        near_vector = np.floor(kp[m.queryIdx].pt).astype(np.int) - np.floor(self.before_img_keypoints[m.trainIdx].pt).astype(np.int)
                        rospy.loginfo(near_vector)
                        
            # next_img = cv2.circle(img=next_img,center=np.floor(near_point).astype(np.uint),radius=10,color=(255,0,0),thickness=5)

            #十分に近い点が見つけられたら、ターゲットの点を移動させる。
            # if min_distance <100:
            self.target_point +=  near_vector
            rospy.loginfo(self.target_point)
            # 点を描画して、Publishする。   
            next_img = cv2.circle(img=next_img,center=np.floor(self.target_point).astype(np.uint),radius=10,color=(0,255,0),thickness=5)
            self.test_publisher.publish(self.cv_bridge.cv2_to_imgmsg(next_img,"bgr8"))
       
                  
        self.before_img_descriptor = descriptor
        self.before_img_keypoints = kp


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
