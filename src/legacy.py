import cv2
import numpy as np

## Top left - 84,4
## Top right - 594,9
## Bottom left - 73,479
## Bottom right - 626,462

video = cv2.VideoCapture(0)
points_1 = np.float32([(84,4),(594,9),(73,479),(626,462)]) 
points_2 = np.float32([(0,0),(640,0),(0,480),(640,480)])

while True:
    _,frame = video.read()
    frame = cv2.medianBlur(frame,3)
    circles = []
    lower_white = np.array([(230,230,230)])
    higher_white = np.array([(255,255,255)])
    lower_yellow = np.array([(120,230,220)])
    higher_yellow = np.array([(155,255,255)])
    perspective_transform = cv2.getPerspectiveTransform(points_1,points_2)
    dst = cv2.warpPerspective(frame,perspective_transform,(640,480))
    mask_white = cv2.inRange(dst,lower_white,higher_white)
    mask_yellow = cv2.inRange(dst,lower_yellow,higher_yellow)
    circles = cv2.HoughCircles(mask_white, cv2.HOUGH_GRADIENT,2,20,param1=20,param2=5,minRadius=3,maxRadius=20)
    print (np.shape(circles))
    for i in circles[0,:]:
        cv2.circle(mask_yellow,(i[0],i[1]),i[2],(0,255,0),2)
    cv2.imshow('Live',dst)
    cv2.imshow('Masked',mask_white)
    cv2.imshow('Ball',mask_yellow)
    print (circles)
    k = cv2.waitKey(5) & 0xFF   #Waiting for the 'Esc' button to be pressed
    if k == 27:
        break

cv2.destroyAllWindows()    #close all the windows
video.release()
