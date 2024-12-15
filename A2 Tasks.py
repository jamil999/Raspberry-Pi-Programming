from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Device, Servo, AngularServo
from time import sleep
Device.pin_factory = PiGPIOFactory()
s = AngularServo(18,min_angle = 0, max_angle =
180,min_pulse_width=0.5/1000,max_pulse_width = 25/10000)
while True:
s.angle=120# (120 degree to the left)
sleep(1)
#right
s.angle=60 # 60 degree to the right
sleep(1)

#NEW CODE

from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Device, Servo, AngularServo
from time import sleep
Device.pin_factory = PiGPIOFactory()
s = AngularServo(18,min_angle = 0, max_angle =
180,min_pulse_width=0.5/1000,max_pulse_width = 25/10000)
while True:
s.angle=120# (120 degree to the left)
sleep(1)
#right
s.angle=60 # 60 degree to the right
sleep(1)

#NEW CODE

from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Device, LED, Button, Servo, AngularServo
from time import sleep
button1= Button(27) #pin 13
button2= Button(22) #pin 15
led=LED(4) #pin 7
Device.pin_factory = PiGPIOFactory()
s = AngularServo(17,min_angle = 0, max_angle =
180,min_pulse_width=0.5/1000,max_pulse_width = 25/10000)
while True:
button1.wait_for_press()
s.angle=120# (120 degree to the left)
led.on()
sleep(1)
led.off()
#right
button2.wait_for_press()
s.angle=60 # 60 degree to the right
led.on()
sleep(1)
led.off()