from machine import Pin, PWM

class Servo:
    pwm_max = 2500
    pwm_min = 500
    period = 65535  # 0xFFFF
    
    def __init__(self):
        self.current_angle = None
    
    def attach(self,pin):
        self.pwm = PWM(Pin(pin, Pin.OUT))
        self.pwm.freq(50)
    
    def write(self, angle):
        angle = max(0, min(180, angle))
        self.current_angle = angle
        high_level_time = (angle / 180) * (self.pwm_max - self.pwm_min) + self.pwm_min
        self.writeMicroseconds(high_level_time)
        
    def writeMicroseconds(self, microseconds):
        high_level_time = microseconds
        duty_cycle_value = int((high_level_time / 20000) * self.period)
        self.pwm.duty_u16(duty_cycle_value)
        
    def read(self):
        return self.current_angle
    
    def detach(self):
        self.pwm.deinit()
        self.pwm = None