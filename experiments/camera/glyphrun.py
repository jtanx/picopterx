from __future__ import print_function
import cv2
from glyph import *
from threading import Thread
import signal
import sys
import traceback


class Webcam:
    def __init__(self):
        self.video_capture = cv2.VideoCapture(0)
        self.current_frame = self.video_capture.read()[1]
        self.destroy = False
          
    # create thread for capturing images
    def start(self):
        self.thread = Thread(target=self._update_frame, args=())
        self.thread.start()
  
    def _update_frame(self):
        while(not self.destroy):
            self.current_frame = self.video_capture.read()[1]
                  
    # get the current frame
    def get_current_frame(self):
        return self.current_frame
 
webcam = Webcam()
webcam.start()

def signal_handler(signal, frame):
    print("EXITING!")
    webcam.destroy = True
    webcam.thread.join()
    
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
 
QUADRILATERAL_POINTS = 4
SHAPE_RESIZE = 100
BLACK_THRESHOLD = 100
WHITE_THRESHOLD = 155


if __name__ == "__main__":    
    try:
        template = cv2.imread("glyph1.png")
        scale = 100.0/template.shape[1] #Screw you too, python2.
        print(scale)
        template = cv2.resize(template, (100, int(template.shape[0]*scale)))
        template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        template = cv2.GaussianBlur(template, (7,7), 0)
        template = cv2.threshold(template, BLACK_THRESHOLD, 255, cv2.THRESH_BINARY)[1]
        #template = cv2.Canny(template, 100, 200)
        cv2.imshow("TEMPLATE", template)
        
        while True:
            #Capture and convert to grey
            image = webcam.get_current_frame()
            gray2 = gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            #Blur before applying canny edge detection
            gray = cv2.GaussianBlur(gray, (5,5), 0)
            edges = cv2.Canny(gray, 100, 200)
            cv2.imshow('Canny', edges)
            #cv2.imshow('Original', image)
            
            #Find contours in the image (similar to connected components)
            contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            #Get the 10 largest contours
            contours = sorted(contours, key=cv2.contourArea, reverse=True)[:10]
             
            for contour in contours:
                #Determine if we're dealing with squares.
                perimeter = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.01*perimeter, True)
                 
                if len(approx) == QUADRILATERAL_POINTS:
                    topdown_quad = get_topdown_quad(gray2, approx.reshape(4, 2))
                    
                    #Discard non-square images by computing the height.
                    rf = float(SHAPE_RESIZE) / topdown_quad.shape[1]
                    rh = topdown_quad.shape[0]*rf
                    if (rh < 50) or (rh > 150): continue
                    
                    #Check inside shape to see if it's black.
                    resized_shape = cv2.resize(topdown_quad, (SHAPE_RESIZE, SHAPE_RESIZE))
                    #Threshold the image
                    resized_shape = cv2.threshold(resized_shape, BLACK_THRESHOLD, 255, cv2.THRESH_BINARY)[1]
                    #if (resized_shape[5,5]): continue
                   
                    #Compute the edges of the warped ROI
                    #edged = cv2.Canny(resized_shape, 100, 200)
                    edged = resized_shape
                    
                    #Perform template matching
                    result = cv2.matchTemplate(edged, template, cv2.TM_CCORR_NORMED)
                    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
                    
                    #Check goodness of fit.
                    if (maxVal > 0.8):
                        print("DETECTED!", maxVal)
                    else:
                        #print("REJECTED!", maxVal)
                        pass
                        
                    cv2.imshow('test', edged)
                    
                    
            cv2.imshow('Original', image)
            cv2.waitKey(10)
    except Exception:
        print(traceback.format_exc())
        signal_handler(2, None)