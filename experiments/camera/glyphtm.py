from __future__ import print_function
import cv2
import numpy as np
from matplotlib import pyplot as plt
from glyphrun import *
import traceback

template = cv2.imread('glyph1.png',0)
w, h = template.shape[::-1]

# All the 6 methods for comparison in a list
methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR',
            'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']

if __name__ == "__main__":
    try:
        while True:            
            #Capture and convert to grey
            image = webcam.get_current_frame()
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            method = cv2.TM_CCOEFF
            res = cv2.matchTemplate(gray, template, method)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
            # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
            if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
                top_left = min_loc
            else:
                top_left = max_loc
            bottom_right = (top_left[0] + w, top_left[1] + h)

            cv2.rectangle(image,top_left, bottom_right, 255, 2)
            cv2.imshow('2D Augmented Reality using Glyphs', image)
            cv2.waitKey(10)
    except Exception:
        print(traceback.format_exc())
        signal_handler(2, None)