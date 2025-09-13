import libs.vehicle 
import time

car = libs.vehicle.ACB_Vehicle()

car.move(car.Clockwise, 255) #The car rotates clockwise
time.sleep(0.75)
car.move(car.Stop, 0)
