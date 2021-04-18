#!/usr/bin/python2.7

'''

__Author_ = Lowyi
__Email__ = MR.LowBattery@gmail.com
__Team__  = MRL_UAV

'''


import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys


class ColorFilter:

    def __init__(self, camera=1):
        self.camera = camera
        self.bridge = CvBridge()


    def nothing(self,x):
        pass

    def color_filtering(self, camera_image):

        cv2.namedWindow('Trackbar window')
        height, width = camera_image.shape[:2]
        img = camera_image[0:height, 0:width]

        # create trackbars for color change
        cv2.createTrackbar('H_high', 'Trackbar window', 0, 255, self.nothing)
        cv2.createTrackbar('S_high', 'Trackbar window', 0, 255, self.nothing)
        cv2.createTrackbar('V_high', 'Trackbar window', 0, 255, self.nothing)
        cv2.createTrackbar('H_low', 'Trackbar window', 0, 255, self.nothing)
        cv2.createTrackbar('S_low', 'Trackbar window', 0, 255, self.nothing)
        cv2.createTrackbar('V_low', 'Trackbar window', 0, 255, self.nothing)


        cv2.imshow('Trackbar window', np.zeros((1, 512, 3), np.uint8))

        _f = cv2.GaussianBlur(img, (15, 15), 2)
        _f = cv2.cvtColor(_f, cv2.COLOR_BGR2HSV)  # To HSV

        h_low = cv2.getTrackbarPos('H_low', 'Trackbar window')
        s_low = cv2.getTrackbarPos('S_low', 'Trackbar window')
        v_low = cv2.getTrackbarPos('V_low', 'Trackbar window')
        h_high = cv2.getTrackbarPos('H_high', 'Trackbar window')
        s_high = cv2.getTrackbarPos('S_high', 'Trackbar window')
        v_high = cv2.getTrackbarPos('V_high', 'Trackbar window')

        # define range of color in HSV
        lower_bound = np.array([h_low, s_low, v_low])
        upper_bound = np.array([h_high, s_high, v_high])

        mask = cv2.inRange(_f, lower_bound, upper_bound)
        img = cv2.bitwise_and(img, img, mask=mask)  # Comment this line if you won't show the frame later

        # Comment the one you won't need
        cv2.imshow('frame', img)
        # cv2.imshow('mask',mask)
        cv2.waitKey(15)

    def callback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.camera == 1:
            self.color_filtering(cv_image)

    def __del__(self):
        pass


cf = None
cf = ColorFilter()


def callback(data):
    cf.callback(data)


def main(args):
    rospy.init_node('ColorFilter', anonymous=True)
    rospy.Subscriber("/kinect2/qhd/image_color_rect", Image, callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
