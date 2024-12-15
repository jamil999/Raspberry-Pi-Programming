import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
TRIG = 20
ECHO = 21
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
return distance
print(distance())
GPIO.cleanup()
[16:08]
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
TRIG = 17
ECHO = 27
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
    return distance
while True:
    print(distance())
GPIO.cleanup()