from machine import Pin
import time
import os
import machine
import time
import micropython

from machine import Pin, PWM
micropython.alloc_emergency_exception_buf(100)

mark = 1
mark_ir = ''

class ACB_IR(object):
#     CODE = {
#         98: "12", 34: "14", 2: "16", 194: "15", 
#         168: "13", 66: "10", 82: "11", 48: "4", 
#         122: "6", 152: "2", 104: "1", 176: "3", 
#         24: "5", 74: "0", 16: "7", 90: "9", 
#         56: "8"
#     }
    CODE = {
        98: "Up", 34: "Left", 2: "Ok", 194: "Right", 
        168: "Down", 66: "*", 82: "#", 48: "4", 
        122: "6", 152: "2", 104: "1", 176: "3", 
        24: "5", 74: "0", 16: "7", 90: "9", 
        56: "8"
    }
    
    def __init__(self, gpioNum):
        self.irRecv = machine.Pin(gpioNum, machine.Pin.IN, machine.Pin.PULL_UP)
        self.irRecv.irq(
             trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING,
             handler=self.__logHandler)

        self.ir_step = 0
        self.ir_count = 0
        self.buf64 = [0 for i in range(64)]
        self.recived_ok = False
        self.cmd = None
        self.cmd_last = None
        self.repeat = 0
        self.repeat_last = None
        self.t_ok = None
        self.t_ok_last = None
        self.start = 0
        self.start_last = 0        
        self.changed = False

    def __logHandler(self, source):
        thisComeInTime = time.ticks_us()

        # update time
        curtime = time.ticks_diff(thisComeInTime, self.start)
        self.start = thisComeInTime
        

        if curtime >= 8500 and curtime <= 9500:
            self.ir_step = 1
            return

        if self.ir_step == 1:
            if curtime >= 4000 and curtime <= 5000:
                self.ir_step = 2
                self.recived_ok = False
                self.ir_count = 0
                self.repeat = 0
            elif curtime >= 2000 and curtime <= 3000:  # Long press to repeat
                self.ir_step = 3
                self.repeat += 1

        elif self.ir_step == 2:  # receive 4 bytes
            self.buf64[self.ir_count] = curtime
            self.ir_count += 1
            if self.ir_count >= 64:
                self.recived_ok = True
                self.t_ok = self.start #Record the last ok time
                self.ir_step = 0

        elif self.ir_step == 3:  # repeat
            if curtime >= 500 and curtime <= 650:
                self.repeat += 1

        # elif self.ir_step == 4:  # End code, if there is no end code, it is possible to receive a duplicate code and start from step=1
        #     if curtime >= 500 and curtime <= 650:
        #         self.ir_step = 0

    def __check_cmd(self):
        byte4 = 0
        for i in range(32):
            x = i * 2
            t = self.buf64[x] + self.buf64[x+1]
            byte4 <<= 1
            if t >= 1800 and t <= 2800:
                byte4 += 1
        user_code_hi = (byte4 & 0xff000000) >> 24
        user_code_lo = (byte4 & 0x00ff0000) >> 16
        data_code = (byte4 & 0x0000ff00) >> 8
        data_code_r = byte4 & 0x000000ff
        self.cmd = data_code

    def scan(self):        
        # data received
        if self.recived_ok:
            self.__check_cmd()
            self.recived_ok = False
            
        # data has changed()
        if self.cmd != self.cmd_last or self.repeat != self.repeat_last or self.t_ok != self.t_ok_last:
            self.changed = True
        else:
            self.changed = False
            self.repeat = 0
            self.t_ok = None
            
        # renew
        self.cmd_last = self.cmd
        self.repeat_last = self.repeat
        self.t_ok_last = self.t_ok
        # Corresponding button character
        s = self.CODE.get(self.cmd)
        if self.changed==False:
            s = None
        return self.changed, s, self.repeat, self.t_ok

    def read_IR(self):
        global info, lcd_print, servo_angle, servo_mark, mark, mark_ir
        IR_re = self.scan()
        return IR_re[1]


