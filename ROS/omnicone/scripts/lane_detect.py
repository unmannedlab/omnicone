#!/usr/bin/env python2.7
import numpy as np
import cv2
import time
import math

cap = cv2.VideoCapture('/home/jacob/Desktop/Videos/test_video_w1.h264')

while(cap.isOpened()):
    ret, frame = cap.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    low_white = np.array([150, 150, 150])
    up_white = np.array([255, 255, 255])
    mask1 = cv2.blur(cv2.inRange(frame, low_white, up_white),(11,11))

    mask2 = mask1[1:360,:]
    edges = cv2.Canny(mask2, 75, 150)

    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, maxLineGap=50)
    if lines is not None:
        slope_sum = 0
        x1_sum = 0.0
        x2_sum = 0.0
        y1_sum = 0.0
        y2_sum = 0.0
        count = 0.0
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if (x2-x1) > 300:
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)
                x1_sum = x1_sum + x1
                x2_sum = x2_sum + x2
                y1_sum = y1_sum + y1
                y2_sum = y2_sum + y2
                slope_sum = slope_sum + (y1-y2)/float(x1-x2)
                count = count + 1
        if count > 0:
            print math.tan(slope_sum/count)
            x1_avg = x1_sum/count
            x2_avg = x2_sum/count
            y1_avg = y1_sum/count
            y2_avg = y2_sum/count

            x1_est = 0
            x2_est = 640
            y1_est = int((y1_avg -y2_avg)*(x1_est-x1_avg)/(x1_avg - x2_avg) + y1_avg)
            y2_est = int((y1_avg -y2_avg)*(x2_est-x1_avg)/(x1_avg - x2_avg) + y1_avg)
            cv2.line(frame, (x1_est, y1_est), (x2_est, y2_est), (255, 0, 0), 5)



    # cv2.imshow("mask", mask)
    # cv2.imshow("edges", edges)
    cv2.imshow("frame", frame)

    numpy_horizontal = np.hstack((mask2, edges, gray[1:360,:]))
    numpy_horizontal_concat = np.concatenate((mask2, edges, gray[1:360,:]), axis=1)
    cv2.imshow('Mask Edges Gray', numpy_horizontal_concat)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    time.sleep(0.05)
cap.release()
cv2.destroyAllWindows()
