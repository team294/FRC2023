from cscore import CameraServer, MjpegServer
from networktables import NetworkTablesInstance
import cv2
import numpy as np
import json
import time

# basic setting variables (constants)
width = 680 
height = 480
port = 8083
minimumArea = 25
maximumArea = 300

name = "reflectivetape"

lower_threshold = np.array([56, 100, 65])
upper_threshold = np.array([74, 255, 255])

# initialize network tables
NetworkTablesInstance.getDefault().initialize(server='10.2.94.2')
sd = NetworkTablesInstance.getDefault().getTable(name)
sd.putNumber("snapshot", 0)

# initialize camera server
cs = CameraServer.getInstance()
cs.enableLogging()

# start capture
camera = cs.startAutomaticCapture()
# camera.setConfigJson(json.dumps(config))
camera.setExposureManual(7)
camera.setWhiteBalanceManual(4500)
camera.setResolution(width, height)

# initialize input and output instances
sink = cs.getVideo()
output = cs.putVideo(name, width, height)
mjpeg = MjpegServer("cvhttpserver", "", port)
mjpeg.setSource(output)

# frame variable
input_img = np.array([[]])

# vision loop
while True:
    # get frame
    time, input_img = sink.grabFrame(input_img)

    # There is an error
    if time == 0:
        output.notifyError(sink.getError())
        continue

    # take a snapshot
    if (sd.getNumber("snapshot", 0) == 1):
        timestr = time.strftime("%Y-%m-%d_%H-%M-%S")
        cv2.imwrite(f"/home/pi/snapshot_{timestr}.jpg", input_img)
        sd.putNumber("snapshot", 0)

    # convert image to hsv
    hsv = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
    
    # get threshold
    threshold = cv2.inRange(hsv, lower_threshold, upper_threshold)

    # erode and then dilate by 3 x 3 kernel of 1s
    kernel = np.ones((3, 3), np.uint8)
    threshold = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, kernel)    
    calibration = cv2.cvtColor(threshold, cv2.COLOR_GRAY2RGB)

    # get contours
    _, contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # init filtered array
    filtered = []
    
    # fill filtered array with values from all contours with an area greater than 15
    # respective values are the contour's center x-value, center y-value, 
    # bounding-rectangle lower-left x-val bounding-rect lower-left y-val, 
    # bounding rect upper-right x, bounding-rect upper-right y-val, and area
    for c in contours:
        area = cv2.contourArea(c)
        if (area < minimumArea or area > maximumArea):
            continue
        rect = cv2.boundingRect(c)
        x,y,w,h = rect
        M = cv2.moments(c)
        # gets center x and y, x1, x2, y1, y2, and area
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            fitlered.append((cX, cY, x, x+w, y, y+h, area))

    rv = 0 # amount of contours found
    rw = 0 # width of contour
    rx = 0 # center x of contour
    # if there are any values in filtered
    if len(filtered) > 0:
        # finds median value and filters for all values within the y tolerance on smart dashboard
        filtered.sort(key=lambda c: c[1]) # sorts array by y-value
        median = filtered[int(len(filtered)/2)][1] # gets median y-value
        # filtered = list(filter(lambda f: abs(median-f[1]) < yT, filtered)) # filters
        filtered = list(filter(lambda f: abs(median-f[1]) < yTolerance, filtered)) # filters

        
        # if there are any values left in filtered
        if len(filtered) > 1:
            # sorts filtered array by contour area and caps it to at-most 4 elements
            filtered = sorted(filtered, key=lambda f: f[6])[-4:]
            
            rv = len(filtered) # gets the amount of contours found

            if rv > 1:
                longWidth = filtered[len(filtered)-1][3] - filtered[len(filtered)-1][2]
                filtered = sorted(filtered, key=lambda c: c[0])
                for i in range(len(filtered)-1, 0, -1):
                    if (filtered[i][0]-filtered[i-1][0] > longWidth*3):
                        rv+=1



            # gets lower-left-most x- and y-value and upper-right-most x- and y-value for final bounding box
            fx, fy, bx, by = filtered[0][2], filtered[0][4], filtered[0][3], filtered[0][5]
            for f in filtered:
                if f[2] < fx: fx = f[2]
                if f[4] < fy: fy = f[4]
                if f[3] > bx: bx = f[3]
                if f[5] > by: by = f[5]
            rw = bx - fx
            rx = -0.0937486*(0.5*(bx+fx)-0.5*width) - 4.99446 # x in pixels converted to angle in degrees!
            # draws bounding rectangle
            cv2.rectangle(input_img,(fx, fy),(bx, by),(0,255,0),2)

    final = cv2.resize(input_img, (160, 120)) # TODO TEST

    # posts number of elements found, width of bounding box, and x-value in angle degrees
    sd.putNumber("rv", rv)
    sd.putNumber("rw", rw)
    sd.putNumber("rx", rx)

    # posts final image to stream
    output.putFrame(final)