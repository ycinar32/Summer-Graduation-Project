
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
from RPi import GPIO
import RPLCD
from pygame import mixer

mixer.init()

GPIO.setmode(GPIO.BCM)
lcd = RPLCD.CharLCD(numbering_mode=GPIO.BCM, rows=2, pin_rs=26, pin_e=19, pins_data=[21,16,10,22,13,6,5,11])

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
uzun_counter=0
kisa_counter=0
timer_boolean = False
lcd_time = 0
x = 0.0
y = 0.0

vs = PiVideoStream().start()
time.sleep(2.0)



while True:
    prev_toc = time.perf_counter();
    
    frame = vs.read()
    frame = imutils.resize(frame, width=400)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = detector(gray)
    
    for face in faces:

        landmarks = predictor(gray, face)

        left_eye_ratio = get_blinking_ratio([36, 37, 38, 39, 40, 41], landmarks)
        right_eye_ratio = get_blinking_ratio([42, 43, 44, 45, 46, 47], landmarks)
        blinking_ratio = (left_eye_ratio + right_eye_ratio) / 2
        
        if blinking_ratio > 4.5:
            counter = counter + 1
            status = 0 # göz kapandı
            #print(counter)
            #cv2.putText(frame, "BLINKING", (50, 150), font, 2, (255, 0, 0))
        else:
            status = 1 # göz acildi
            if current_counter != counter:
                if (counter - current_counter) == 3 or (counter - current_counter) > 3:
                    uzun_counter = uzun_counter + 1
                    #print("Uzun Göz Kırpma")
                if (counter - current_counter) < 3 :
                    #print("Kısa Göz Kırpma")
                    kisa_counter = kisa_counter + 1
            current_counter = counter
        
        print(blinking_ratio)
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    
    toc = time.perf_counter();
    
    if kisa_counter == 0 and uzun_counter == 0:
        lcd.clear()
        lcd.write_string("   Bekleniyor   ")
        bitis_zamani = prev_toc + 10
    
    else:
        if int(toc) - int(prev_toc) > 0 :
            #print (int(toc) - int(prev_toc))
            lcd_time = lcd_time + 1 
        
        if toc  < bitis_zamani :
            #print("tic          = " + str(toc))
            #print("bitis zamanı = " + str(bitis_zamani))
            lcd.clear()
            lcd.write_string("Uzun="+str(uzun_counter)+" Kisa="+str(kisa_counter) +"   "+ str(lcd_time))
        
        elif toc > bitis_zamani:
            #print("tic = " + str(toc))
            #print("bitis zamanı = " + str(bitis_zamani))
            lcd.clear()
            lcd.write_string("Uzun="+str(uzun_counter)+" Kisa="+str(kisa_counter)+"   " + "10")
            time.sleep(1.0)
            lcd.clear()
            if kisa_counter == 1 and uzun_counter==0:
                lcd.write_string("Kelime Algilandi")
                time.sleep(2.0)
                lcd.clear()
                lcd.write_string("      Evet      ")
                sound = mixer.Sound('evet.wav')
                sound.play()
                time.sleep(3.0)

            elif kisa_counter == 0 and uzun_counter==1:
                lcd.write_string("Kelime Algilandi")
                time.sleep(2.0)
                lcd.clear()
                lcd.write_string("      Hayir      ")
                sound = mixer.Sound('hayir.wav')
                sound.play()
                time.sleep(3.0)

            elif kisa_counter == 1 and uzun_counter==1:
                lcd.write_string("Kelime Algilandi")
                time.sleep(2.0)
                lcd.clear()
                lcd.write_string("    Aciktim     ")
                sound = mixer.Sound('aciktim.wav')
                sound.play()
                time.sleep(3.0)

            elif kisa_counter == 2 and uzun_counter==0:
                lcd.write_string("Kelime Algilandi")
                time.sleep(2.0)
                lcd.clear()
                lcd.write_string("    Susadim     ")
                sound = mixer.Sound('susadim.wav')
                sound.play()
                time.sleep(3.0)

            elif kisa_counter == 0 and uzun_counter==2:
                lcd.write_string("Kelime Algilandi")
                time.sleep(2.0)
                lcd.clear()
                lcd.write_string("   Agrim var    ")
                sound = mixer.Sound('agrim_var.wav')
                sound.play()
                time.sleep(3.0)

            elif kisa_counter == 2 and uzun_counter==1:
                lcd.write_string("Kelime Algilandi")
                time.sleep(2.0)
                lcd.clear()
                lcd.write_string("     Usudum     ")
                sound = mixer.Sound('usudum.wav')
                sound.play()
                time.sleep(3.0)

            elif kisa_counter == 1 and uzun_counter==2:
                lcd.write_string("Kelime Algilandi")
                time.sleep(2.0)
                lcd.clear()
                lcd.write_string("     Iyiyim     ")
                sound = mixer.Sound('iyiyim.wav')
                sound.play()
                time.sleep(3.0)

            elif kisa_counter == 3 and uzun_counter==0:
                lcd.write_string("Kelime Algilandi")
                time.sleep(2.0)
                lcd.clear()
                lcd.write_string("    Yoruldum    ")
                sound = mixer.Sound('yoruldum.wav')
                sound.play()
                time.sleep(3.0)

            elif kisa_counter == 3 and uzun_counter==1:
                lcd.write_string("Kelime algilandi")
                time.sleep(2.0)
                lcd.clear()
                lcd.write_string("    Tuvalet!    ")
                sound = mixer.Sound('wc.wav')
                sound.play()
                time.sleep(3.0)

            elif kisa_counter == 4 and uzun_counter==0:
                lcd.write_string("Kelime algilandi")
                time.sleep(2.0)
                lcd.clear()
                lcd.write_string("     Bay Bay   ")
                sound = mixer.Sound('bay.wav')
                sound.play()
                time.sleep(3.0)
            else:
                lcd.write_string("   Kelime yok   ")
                time.sleep(2.0)
            tic = time.perf_counter();
            bitis_zamani = tic + 10 
            kisa_counter=0
            uzun_counter=0
            lcd_time = 0
            toc = time.perf_counter();
            lcd.clear()
     
cv2.destroyAllWindows()
vs.stop()
GPIO.cleanup()


