import libs.vehicle
from machine import ADC
import time

Left_sensor = 35 #Declare the pin of left tracking sensor
Right_sensor = 39 #Declare the pin of right tracking sensor
adc1 = ADC(Left_sensor)
adc1.width(ADC.WIDTH_12BIT)
adc1.atten(ADC.ATTN_11DB)
adc2 = ADC(Right_sensor)
adc2.width(ADC.WIDTH_12BIT)
adc2.atten(ADC.ATTN_11DB)
Black_Line = 2000
car = libs.vehicle.ACB_Vehicle()
car.move(car.Stop, 0)

while True:
    Left_Tra_Value = adc1.read() #Read the value of left tracking sensor
    Right_Tra_Value = adc2.read()

    time.sleep(0.0005)
    #Both sensors are on the black line
    if Left_Tra_Value < Black_Line and Right_Tra_Value < Black_Line:
        car.move(car.Forward, 150) #Smart car forward
    
    #left sensor is on black line,right sensor is on white background
    elif Left_Tra_Value >= Black_Line and Right_Tra_Value < Black_Line:
        car.move(car.Contrarotate, 150) #Smart car turn left
    
    #right sensor is on black line,left sensor is on the white background
    elif Left_Tra_Value < Black_Line and Right_Tra_Value >= Black_Line:
        car.move(car.Clockwise, 150) #Smart car turn right
    
    #Both sensors are on the white background
    elif Left_Tra_Value >= Black_Line and Right_Tra_Value >= Black_Line:
        car.move(car.Stop, 0) #Smart car stop
        
        
        
        





