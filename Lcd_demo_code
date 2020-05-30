from time import sleep
from RPi import GPIO

import RPLCD

GPIO.setmode(GPIO.BCM)

lcd = RPLCD.CharLCD(numbering_mode=GPIO.BCM, rows=2, pin_rs=26, pin_e=19, pins_data=[21,16,10,22,13,6,5,11])

lcd.write_string("TOBB ETU")
sleep(5)

lcd.clear()
GPIO.cleanup()
