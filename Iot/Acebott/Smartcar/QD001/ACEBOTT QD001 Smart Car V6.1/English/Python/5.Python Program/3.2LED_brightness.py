import time
from machine import Pin, PWM
#freq is an integer that sets the frequency (in Hz) of the PWM cycle;
left_led = PWM(Pin(2), freq=1000)
right_led = PWM(Pin(12), freq=1000)

#Change the red part of the parameter, the parameter range 0-1023, 
#so that the lights display different brightness
left_led.duty(500) 
right_led.duty(500)
