from .Detection.Lanes.lane_detection import detect_lanes
import cv2

class Car():
    
    def drive_car(self, frame):
        img = frame[0:640, 238:1042]
        img = cv2.resize(img, (320,240))
        
        detect_lanes(img)
