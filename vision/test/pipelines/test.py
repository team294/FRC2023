from Pipelines import test

import cv2
import numpy as np


# filename or index
i = "vision/test/videos/filename6brightness0saturation89contrast100exposure_abs0.avi"

cap = cv2.VideoCapture(i)


while True:
    ret, frame = cap.read()

    frame = test(frame)

    cv2.imshow('frame',frame)         # show image
    if cv2.waitKey(10) == ord('q'):  # wait a bit, and see keyboard press
        break                        # if q pressed, quit

# release things before quiting
cap.release()
cv2.destroyAllWindows()