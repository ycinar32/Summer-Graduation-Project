from __future__ import print_function
from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import imutils
import time
import cv2
import numpy as np
import dlib
from math import hypot

ap = argparse.ArgumentParser()
ap.add_argument("-n", "--num-frames", type=int, default=100,
    help="# of frames to loop over for FPS test")
ap.add_argument("-d", "--display", type=int, default=-1,
    help="Whether or not frames should be displayed")
args = vars(ap.parse_args())

detector = dlib.get_frontal_face_detector()
print("Predictor verileri alınmaya başlandı.")
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")
print("Predictor verileri alındı.")

def midpoint(p1 ,p2):
    return int((p1.x + p2.x)/2), int((p1.y + p2.y)/2)

def get_blinking_ratio(eye_points, facial_landmarks):
    left_point = (facial_landmarks.part(eye_points[0]).x, facial_landmarks.part(eye_points[0]).y)
    right_point = (facial_landmarks.part(eye_points[3]).x, facial_landmarks.part(eye_points[3]).y)
    center_top = midpoint(facial_landmarks.part(eye_points[1]), facial_landmarks.part(eye_points[2]))
    center_bottom = midpoint(facial_landmarks.part(eye_points[5]), facial_landmarks.part(eye_points[4]))

    hor_line = cv2.line(frame, left_point, right_point, (0, 255, 0), 2)
    ver_line = cv2.line(frame, center_top, center_bottom, (0, 255, 0), 2)

    hor_line_lenght = hypot((left_point[0] - right_point[0]), (left_point[1] - right_point[1]))
    ver_line_lenght = hypot((center_top[0] - center_bottom[0]), (center_top[1] - center_bottom[1]))

    ratio = hor_line_lenght / ver_line_lenght
    return ratio

#font = cv2.FONT_HERSHEY_PLAIN
current_counter = 0
counter = 0
blinking_length = 0
status = 1 #acık kapalı
timer_boolean = False

vs = PiVideoStream().start()
time.sleep(2.0)



while True:
    tic = time.perf_counter();

    frame = vs.read()
    frame = imutils.resize(frame, width=400)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = detector(gray)
    
    for face in faces:

        landmarks = predictor(gray, face)

        left_eye_ratio = get_blinking_ratio([36, 37, 38, 39, 40, 41], landmarks)
        right_eye_ratio = get_blinking_ratio([42, 43, 44, 45, 46, 47], landmarks)
        blinking_ratio = (left_eye_ratio + right_eye_ratio) / 2

        if blinking_ratio > 5.7:
            counter = counter + 1
            status = 0 # göz kapandı
            print(counter)
            #cv2.putText(frame, "BLINKING", (50, 150), font, 2, (255, 0, 0))
        else:
            status = 1 # göz acildi
            if current_counter != counter:
                if (counter - current_counter) > 1:
                    print("Uzun Göz Kırpma")
                if (counter - current_counter) == 1:
                    print("Kısa Göz Kırpma")
            current_counter = counter

    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    
    toc = time.perf_counter();
    
    print(1/(toc-tic))
    
cv2.destroyAllWindows()
vs.stop()
