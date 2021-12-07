#! /usr/bin/env python
import rospy
import cv2
from image.msg import ping
import numpy as np
Low_H = 6
High_H = 89
Low_S = 65
High_S = 213
Low_V = 193
High_V = 255
IMAGE_WINDOW_NAME = 'YelloBarTracker'
CONTROL_WINDOW_NAME = 'Control'
MASK_WINDOW_NAME = 'Mask'
kernel = np.ones((5, 5), np.uint8)
def nothing():
    pass
def make_hsv_adjustment():
    cv2.namedWindow(CONTROL_WINDOW_NAME)
    cv2.createTrackbar('Low_H', CONTROL_WINDOW_NAME, Low_H, 255, nothing) #Hue (0 - 179)
    cv2.createTrackbar('High_H', CONTROL_WINDOW_NAME, High_H, 255, nothing)

    cv2.createTrackbar('Low_S', CONTROL_WINDOW_NAME, Low_S, 255, nothing) #Saturation (0 - 255)
    cv2.createTrackbar('High_S', CONTROL_WINDOW_NAME, High_S, 255, nothing)

    cv2.createTrackbar('Low_V', CONTROL_WINDOW_NAME, Low_V, 255, nothing) #Value (0 - 255)
    cv2.createTrackbar('High_V', CONTROL_WINDOW_NAME, High_V, 255, nothing)
def main():
    rospy.init_node("image_process", anonymous=True)
    ping_info_pub = rospy.Publisher('/ping_info', ping, queue_size=1)
    rate = rospy.Rate(30)
    msg = ping()
    msg.x = 0
    msg.y = 0
    msg.r = 0
    lower_yellow = np.array([Low_H,Low_S,Low_V])
    upper_yellow = np.array([High_H,High_S,High_V])
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    #make_hsv_adjustment()
    if cap.isOpened():
        rospy.loginfo("cap.isopened")
    while not rospy.is_shutdown():
        ret, image0 = cap.read()
        if(ret):
            # blur = cv2.GaussianBlur(image0, (5,5),0)
            hsv = cv2.cvtColor(image0, cv2.COLOR_BGR2HSV)
            # iLowH = cv2.getTrackbarPos('LowH', CONTROL_WINDOW_NAME)
            # iHighH = cv2.getTrackbarPos('HighH', CONTROL_WINDOW_NAME)
            # iLowS = cv2.getTrackbarPos('LowS', CONTROL_WINDOW_NAME)
            # iHighS = cv2.getTrackbarPos('HighS', CONTROL_WINDOW_NAME)
            # iLowV = cv2.getTrackbarPos('LowV', CONTROL_WINDOW_NAME)
            # iHighV = cv2.getTrackbarPos('HighV', CONTROL_WINDOW_NAME)
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            e_mask = cv2.erode(mask, kernel, iterations=1)
            p_mask = cv2.dilate(e_mask, kernel, iterations=1)
            binary,contours, hierarchy = cv2.findContours(p_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            areas = [cv2.contourArea(contours[i]) for i in range(len(contours))]
            contour = list()
            if len(areas) != 0:
                contour.append(contours[areas.index(max(areas))])
                x,y = (int(np.mean(contour[0][:,0,0])),int(np.mean(contour[0][:,0,1])))
            else:
                x,y = 0,0
            cv2.drawContours(image0, contour, -1, (0, 0, 255), 3)
            rospy.loginfo("publisher:   X:%d,Y:%d,R:%d",x,y,0)
            msg.x = x
            msg.y = y
            ping_info_pub.publish(msg)
            cv2.imshow(IMAGE_WINDOW_NAME, image0)
            cv2.waitKey(1)
        rate.sleep()

if __name__ == '__main__':
    main()
