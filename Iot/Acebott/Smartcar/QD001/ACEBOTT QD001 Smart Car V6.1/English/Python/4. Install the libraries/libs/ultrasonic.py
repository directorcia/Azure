from machine import Pin
import time

class ACB_Ultrasonic:
    def __init__(self, trig_pin, echo_pin):
        self.trig = Pin(trig_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)

    def get_distance(self):
        # Generate a 10us square wave
        self.trig.value(0)
        time.sleep_us(2)
        self.trig.value(1)
        time.sleep_us(10)
        self.trig.value(0)

        # Initialize timeout counter and start time
        timeout = time.ticks_us() + 1000000  # Set a 1 second timeout
        start = 0

        # Wait for the echo
        while self.echo.value() == 0:
            if time.ticks_us() > timeout:
                return float('inf')  # Return a very large value if timed out
            start = time.ticks_us()

        # If echo detected, continue waiting until the echo ends
        while self.echo.value() == 1:
            end = time.ticks_us()

        # Calculate distance and return
        dis = (end - start) * 0.0343 / 2
        return round(dis, 2)  # Round to two decimal places
