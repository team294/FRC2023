from Pipelines import test

import cv2
import numpy as np
import time

# filename or index
# i = "vision/test/videos/filename6brightness0saturation89contrast100exposure_abs0.avi"
i = "vision/test/videos/filename6brightness0saturation100contrast100exposure_abs0(test_2_42_inches_from_pole).avi"

cap = cv2.VideoCapture(i)


while True:
    ret, frame = cap.read()

    # if cv2.waitKey(10) == ord('f'):
    #     print("TESTING")
    #     cv2.imshow('frame',frame)
    #     continue

    frame = test(frame)

    cv2.imshow('frame',frame)         # show image
    if cv2.waitKey(10) == ord('q'):  # wait a bit, and see keyboard press
        break                        # if q pressed, quit
    elif cv2.waitKey(10) == ord('p'):
        while cv2.waitKey(10) != ord('p'):
            time.sleep(.02)

# release things before quiting
cap.release()
cv2.destroyAllWindows()