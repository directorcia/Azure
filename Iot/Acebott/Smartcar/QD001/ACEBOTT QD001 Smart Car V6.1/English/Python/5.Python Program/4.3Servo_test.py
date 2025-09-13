import libs.servo
import time

servo_pin = 25 #Declare the pin of the servo
servo = libs.servo.Servo() #create servo object to control a servo
servo.attach(servo_pin)

servo.write(0) #initialize servo motor

while True:
    #Servo motor from 0 degrees to 180 degrees
    for i in range(0, 180, 1):
        servo.write(i)
        time.sleep(0.01) 
    #Servo motor from 180 degrees to 0 degrees
    for i in range(180, 0, -1):
        servo.write(i)
        time.sleep(0.01) 