import libs.ir
import time

IR_PIN = 4;
irm = libs.ir.ACB_IR(IR_PIN)

while True:
    ir_cmd = irm.read_IR() 
    if ir_cmd != None:
        print(ir_cmd) 
    time.sleep(0.11)
