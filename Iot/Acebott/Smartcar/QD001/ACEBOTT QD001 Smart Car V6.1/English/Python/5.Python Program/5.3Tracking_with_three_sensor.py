import libs.vehicle
from machine import ADC
import time

Left_sensor = 35 #Declare the pin of left tracking sensor
Middle_sensor = 36 #Declare the pin of middle tracking sensor
Right_sensor = 39 #Declare the pin of right tracking sensor

adc1 = ADC(Left_sensor)
adc1.width(ADC.WIDTH_12BIT)
adc1.atten(ADC.ATTN_11DB)
adc2 = ADC(Right_sensor)
adc2.width(ADC.WIDTH_12BIT)
adc2.atten(ADC.ATTN_11DB)
adc3 = ADC(Middle_sensor)
adc3.width(ADC.WIDTH_12BIT)
adc3.atten(ADC.ATTN_11DB)
    
Black_Line = 2000
car = libs.vehicle.ACB_Vehicle()
car.move(car.Stop, 0)

while True:
    
    Left_Tra_Value = adc1.read() 
    Right_Tra_Value = adc2.read()
    Middle_Tra_Value = adc3.read()
    time.sleep(0.05)

    if Left_Tra_Value < Black_Line and Middle_Tra_Value >= Black_Line and Right_Tra_Value < Black_Line:
        car.move(car.Forward, 150)
        
    if Left_Tra_Value < Black_Line and Middle_Tra_Value >= Black_Line and Right_Tra_Value >= Black_Line:
        car.move(car.Forward, 120)
    
    if Left_Tra_Value >= Black_Line and Middle_Tra_Value >= Black_Line and Right_Tra_Value < Black_Line:
        car.move(car.Forward, 120)
        
    elif Left_Tra_Value >= Black_Line and Middle_Tra_Value < Black_Line and Right_Tra_Value < Black_Line:
        car.move(car.Contrarotate, 120)
    
    elif Left_Tra_Value < Black_Line and Middle_Tra_Value < Black_Line and Right_Tra_Value >= Black_Line:
        car.move(car.Clockwise, 120)
        
    elif Left_Tra_Value >= Black_Line and Middle_Tra_Value >= Black_Line and Right_Tra_Value >= Black_Line:
        car.move(car.Forward, 120)
        
        


