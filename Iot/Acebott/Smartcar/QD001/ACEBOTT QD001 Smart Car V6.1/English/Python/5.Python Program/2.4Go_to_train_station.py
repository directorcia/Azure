import libs.vehicle 
import time

car = libs.vehicle.ACB_Vehicle()

car.move(car.Forward, 255) #Control car forward moving
time.sleep(3)
car.move(car.Clockwise, 255) #Control car counterclockwise rotate
time.sleep(0.75) #Modify the parameters in red
car.move(car.Forward, 255) #Control car forward moving
time.sleep(1.5)
car.move(car.Clockwise, 255) #Control car counterclockwise rotate
time.sleep(0.75) #Modify the parameters in red
car.move(car.Forward, 255) #Control car forward moving
time.sleep(3)
car.move(car.Contrarotate, 255)#Control car contrarotate
time.sleep(0.75) #Modify the parameters in red
car.move(car.Forward, 255) #Control car forward moving
time.sleep(1.5)
car.move(car.Contrarotate, 255)#Control car contrarotate
time.sleep(0.75) #Modify the parameters in red
car.move(car.Forward, 255) #Control car forward moving
time.sleep(3)
car.move(car.Move_Right, 255) #Control car left moving
time.sleep(0.75) #Modify the parameters in red
car.move(car.Stop, 0) #Control car stop
