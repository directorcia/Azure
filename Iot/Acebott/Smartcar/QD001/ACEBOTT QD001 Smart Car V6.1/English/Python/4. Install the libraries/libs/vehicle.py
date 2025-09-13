from machine import Pin, PWM

class ACB_Vehicle:
    # PIN Definitions
    PWM1_PIN = 19
    PWM2_PIN = 23
    SHCP_PIN = 18
    EN_PIN = 16
    DATA_PIN = 5
    STCP_PIN = 17

    # Direction and Speed Constants
    Forward = 163
    Backward = 92
    Move_Left = 106
    Move_Right = 149
    Top_Left = 34
    Bottom_Left = 72
    Top_Right = 129
    Bottom_Right = 20
    Stop = 0
    Contrarotate = 83
    Clockwise = 172

    # Motor Commands
    Moedl1 = 25
    Moedl2 = 26
    Moedl3 = 27
    Moedl4 = 28
    MotorLeft = 230
    MotorRight = 231
    M1_Forward = 128
    M1_Backward = 64
    M2_Forward = 32
    M2_Backward = 16
    M3_Forward = 2
    M3_Backward = 4
    M4_Forward = 1
    M4_Backward = 8

    def __init__(self):
        """ Initialize the motor controller pins and PWM """
        self.pwm1_pin = PWM(Pin(self.PWM1_PIN)) 
        self.pwm2_pin = PWM(Pin(self.PWM2_PIN))
        self.shcp_pin = Pin(self.SHCP_PIN, Pin.OUT) 
        self.en_pin = Pin(self.EN_PIN, Pin.OUT)
        self.data_pin = Pin(self.DATA_PIN, Pin.OUT)
        self.stcp_pin = Pin(self.STCP_PIN, Pin.OUT)

        # Set PWM frequency for both motors
        self.pwm1_pin.freq(500) 
        self.pwm2_pin.freq(500) 

    def move(self, dir, speed):
        """ Move the motors in a specified direction with a given speed """
        self.en_pin.off()  # Enable the motor
        speed = 4 * speed  # Adjust speed as necessary
        self.pwm1_pin.duty(speed) 
        self.pwm2_pin.duty(speed)  

        self.stcp_pin.off()  # Prepare to shift out data
        self.shift_out(self.data_pin, self.shcp_pin, 'MSBFIRST', dir)
        self.stcp_pin.on()  # Latch the data

    def shift_out(self, data_pin, clock_pin, order, value):
        """ Shift out a byte of data to the shift register """
        for i in range(8):
            if order == 'MSBFIRST':
                bit = (value >> (7 - i)) & 1
            else:
                bit = (value >> i) & 1
            
            data_pin.off() if bit == 0 else data_pin.on()  
            clock_pin.on()
            clock_pin.off()