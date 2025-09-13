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

while True:
    Left_Tra_Value = adc1.read() #Read the value of left tracking sensor
    Right_Tra_Value = adc2.read() #Read the value of right tracking sensor
    print("Left_sensor_value:",Left_Tra_Value)
    print("Right_sensor_value:",Right_Tra_Value)
    print(" ")
    time.sleep(1)