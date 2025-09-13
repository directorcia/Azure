import libs.vehicle #Import car library
import time

car = libs.vehicle.ACB_Vehicle()#Creating a car object

car.move(car.Forward, 255)#Control car forward moving
time.sleep(3)#Control the movement time of the car
car.move(car.Stop, 0)#Control car stop