import time
from machine import Pin #Import the pin

left_led = Pin(2, Pin.OUT) #Define the pin number of the LED
right_led = Pin(12, Pin.OUT)

while True:
    #Turn on the LED, "1 "means high level and "0" means low level
    left_led.value(1)
    right_led.value(1)

