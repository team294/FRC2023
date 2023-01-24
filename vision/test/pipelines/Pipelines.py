import cv2
import numpy as np
import math
import time

def test(frame):
    return detect_pole(frame)

# basic setting variables (constants)
width = 680 
height = 480
port = 8083
minimumArea = 25
maximumArea = 300
yTolerance = 3000

name = "reflectivetape"

lower_threshold = np.array([56, 100, 65])
upper_threshold = np.array([74, 255, 255])


def detect_pole(input_img):
    # convert image to hsv

    # TODO What kind of blur?
    blurred = cv2.blur(input_img, (10,10))

    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # get threshold
    threshold = cv2.inRange(hsv, lower_threshold, upper_threshold)

    # erode and then dilate by 3 x 3 kernel of 1s
    kernel = np.ones((3, 3), np.uint8)
    threshold = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, kernel)    
    calibration = cv2.cvtColor(threshold, cv2.COLOR_GRAY2RGB)

    # get contours, TODO use first one for actual deployment on wpilibpi
    # _, contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # init filtered array
    filtered = []
    
    rv = 0
    d1 = 0
    d2 = 0
    # fill filtered array with values from all contours with an area greater than 15
    # respective values are the contour's center x-value, center y-value, 
    # bounding-rectangle lower-left x-val bounding-rect lower-left y-val, 
    # bounding rect upper-right x, bounding-rect upper-right y-val, and area
    for c in contours:
        area = cv2.contourArea(c)
        # if (area < minimumArea or area > maximumArea):
        #     continue
        rect = cv2.boundingRect(c)
        x,y,w,h = rect
        cv2.rectangle(input_img,(x,y),(x+w,y+h),(0,255,0),2)
        M = cv2.moments(c)
        # gets center x and y, x1, x2, y1, y2, and area
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            filtered.append((cX, cY, x, x+w, y, y+h, area, h))
            rv+=1

    # dist1 = 359-
    if rv == 0:
        return input_img

    for tape in filtered:
        if tape[1] < height * 0.5:
            d1 = 359 - 81.3 * math.log(tape[7])
        else:
            d2 =  387 - 78.6 * math.log(tape[7])

    filtered.sort(key=lambda c: c[6])
    
    try:
        a = filtered[0]
        b = filtered[1]
        print(f"A - height: {a[5]-a[4]}, area: {a[6]}, cY: {a[1]}, distance {d1}")
        print(f"B - height: {b[5]-b[4]}, area: {b[6]}, cY: {b[1]}, distance {d2}")
    except:
        time.sleep(0.01)

    # cv2.rectangle(input_img,(fx, fy),(bx, by),(0,255,0),2)
    return input_img