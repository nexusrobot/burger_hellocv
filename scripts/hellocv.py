#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray

class ColorFinder:
    def __init__(self,bgr_image,lower,upper):
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        mask_image = cv2.inRange(hsv_image, lower, upper)
        processed_image = cv2.bitwise_and(bgr_image, bgr_image, mask=mask_image)

        mom = cv2.moments(mask_image)
        cx = 0
        cy = 0
        
        if mom["m00"] > 1000000:
            if "m00" in mom and "m10" in mom and "m01" in mom and mom["m00"] <> 0:
                cx = int(mom["m10"]/mom["m00"])
                cy = int(mom["m01"]/mom["m00"])
        self.center = [cx,cy]
        self.area = mom["m00"]

        #marker point
        color = (255, 0, 255)
        processed_image = cv2.circle(processed_image, (cx,cy), 3, color, -1)

        #publishing result image
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('processed_image', Image, queue_size=10)
        image_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
        self.image_pub.publish(image_msg)

    def getCenter(self):
        return self.center

    def getArea(self):
        return self.area


class HelloCv:
    def __init__(self):
        self.yellow_center_pub = rospy.Publisher('yellow_center', Int32MultiArray, queue_size=10)

        cols = 640
        rows = 480
        self.img = np.full((rows, cols, 3), 0, dtype=np.uint8)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)
    
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            rospy.loginfo("image received");
        except CvBridgeError as e:
            rospy.logerr(e)
    
    def execute(self):
        bgr_image = self.img

        lower = np.array([20,127,210])#yellow
        upper = np.array([35,255,255])#yellow
        #lower = np.array([40,100,50])#green
        #upper = np.array([60,255,255])#green

        yellow_finder = ColorFinder(
                bgr_image,
                np.array([20,127,210])
                ,np.array([35,255,255])
                )

        ycen = yellow_finder.getCenter()
        rospy.loginfo("Area %f, xy(%d,%d)", yellow_finder.getArea(), ycen[0],ycen[1])

        yellow_center = Int32MultiArray(data=ycen)
        self.yellow_center_pub.publish(yellow_center)



if __name__ == '__main__':
    try:
        rospy.init_node('hellocv', anonymous=False)
        hello = HelloCv()

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            hello.execute()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
