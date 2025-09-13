import libs.vehicle
from machine import Pin, PWM, Timer, ADC
import libs.ir
import time

IR_PIN = 4;
irm = libs.ir.ACB_IR(IR_PIN)
speeds = 220

car = libs.vehicle.ACB_Vehicle()
car.move(car.Stop, 0)

while True:
    ir_cmd = irm.read_IR() 
    if ir_cmd != None:
        print(ir_cmd) 

    if ir_cmd=="Up": #Press "up" button to move forward
        car.move(car.Forward, speeds)
    
    elif ir_cmd=="Down":#Press "down" button to move backward
        car.move(car.Backward, speeds)
        
    elif ir_cmd=="Left": #Press "left" button to turn left   
        car.move(car.Contrarotate, speeds)
        
    elif ir_cmd=="Right": #Press "right" button to turn right  
        car.move(car.Clockwise, speeds)
        
    elif ir_cmd=="1": #Press button "1" to move left  
        car.move(car.Move_Left, speeds)
        
    elif ir_cmd=="3": #Press button "3" to move right 
        car.move(car.Move_Right, speeds)
    
    else:
        car.move(car.Stop, 0)
    
    time.sleep(0.2)

    
    

