import libs.vehicle
import libs.ultrasonic
import libs.servo
import time

Trig_PIN = 13
Echo_PIN = 14

servo_pin = 25 
servo = libs.servo.Servo()
servo.attach(servo_pin)

car = libs.vehicle.ACB_Vehicle()
car.move(car.Stop, 0)

leftDistance = 0
middleDistance = 0
rightDistance = 0

Ultrasonic = libs.ultrasonic.ACB_Ultrasonic(Trig_PIN,Echo_PIN)

while True:
    middleDistance = Ultrasonic.get_distance()
    servo.write(90)
    
    if middleDistance <= 25:
        car.move(car.Stop, 0)
        time.sleep(0.01)
        for i in range(90, 0, -1):#Servo car from 90 degrees to 0 degrees
            servo.write(i)
            time.sleep(0.01)
        
        time.sleep(0.5)
        rightDistance = Ultrasonic.get_distance()
        for i in range(0, 180, 1):#Servo car from 0 degrees to 180 degrees
            servo.write(i)
            time.sleep(0.01)
        
        time.sleep(0.5)
        leftDistance = Ultrasonic.get_distance()
        if rightDistance<20 and leftDistance<20:
            car.move(car.Backward, 180)
            time.sleep(0.5)
            car.move(car.Contrarotate, 180)
            time.sleep(1)

        elif rightDistance > leftDistance:
            car.move(car.Backward, 180)
            time.sleep(0.5)
            car.move(car.Clockwise, 180)
            time.sleep(1)
        
        elif rightDistance < leftDistance:
            car.move(car.Backward, 180)
            time.sleep(0.5)
            car.move(car.Contrarotate, 180)
            time.sleep(1)
            
        else:
            car.move(car.Backward, 180)
            time.sleep(0.5)
            car.move(car.Clockwise, 180)
            time.sleep(1)
          
    else:
        car.move(car.Forward, 150)
        
        
