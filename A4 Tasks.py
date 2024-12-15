import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Device, LED, Button, Servo, AngularServo
from time import sleep


TRIG = 17
ECHO = 27

Device.pin_factory = PiGPIOFactory()
s=AngularServo(22,min_angle=0,max_angle=180,min_pulse_width=0.5/1000,max_pulse_width = 25/10000)


GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)

def distance():
    GPIO.output(TRIG, False)
    time.sleep(0.5)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    pulse_start = time.time()

    while GPIO.input(ECHO)==0:
        pulse_start = time.time()
    while GPIO.input(ECHO)==1:
        pulse_end = time.time()
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    
    

    #return distance

while True:
    if distance > 40:
        s.angle=120
        sleep(1)

    if distance < 20:
        s.angle=60
        sleep(1)

    else:
        s.angle=90
        sleep(1)
    #print(distance())

GPIO.cleanup()