import time
from machine import Pin, PWM

left_led = PWM(Pin(2), freq=1000)
right_led = PWM(Pin(12), freq=1000)

while True:
    # Gradually Turn on the headlights
    for i in range(0, 1023, 1):
        left_led.duty(i)
        right_led.duty(i)
        time.sleep(0.01) #The delay is 1 millisecond
    
    # Gradually extinguish the headlights
    for i in range(1023, 0, -1):
        left_led.duty(i)
        right_led.duty(i)
        time.sleep(0.01)
