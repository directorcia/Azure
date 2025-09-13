import libs.vehicle 
import time

car = libs.vehicle.ACB_Vehicle()

car.move(car.Move_Right, 255)#Control car to move right
time.sleep(0.75)
car.move(car.Stop, 0)
