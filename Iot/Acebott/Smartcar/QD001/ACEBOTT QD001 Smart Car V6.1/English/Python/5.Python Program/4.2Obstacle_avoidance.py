import libs.vehicle 
import libs.ultrasonic
import time

Trig_PIN = 13
Echo_PIN = 14

car = libs.vehicle.ACB_Vehicle()
Ultrasonic = libs.ultrasonic.ACB_Ultrasonic(Trig_PIN,Echo_PIN)

while True:
    UT_distance = Ultrasonic.get_distance()
    #The distance is less than 25cm to achieve the effect of turning
    if UT_distance <= 25:
        car.move(car.Contrarotate, 180)
        time.sleep(1.5) #The turning time is modified to realize the rotation of different angles
        car.move(car.Stop, 0)
    else:
        car.move(car.Forward, 150) #If the distance is greater than 25, move forward