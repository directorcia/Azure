import time
from machine import Pin, PWM

buzzer = 33
# Define a list of tone frequencies
C4 = 262
D4 = 294
E4 = 330
F4 = 350
G4 = 393
A4 = 441
B4 = 495
N = 0

# little star
tune0 = [C4, N, C4, G4, N, G4, A4, N, A4, G4, N, F4, N, F4, E4, N, E4, D4, N, D4, C4]
durt0 = [0.99, 0.01, 1, 0.99, 0.01, 1, 0.99, 0.01, 1, 1.95, 0.05, 0.99, 0.01, 1, 0.99, 0.01, 1, 0.99, 0.01, 1, 2]


buzzer_pin = PWM(Pin(buzzer), freq=1000, duty=0) #Buzzer initialization


# Define the function to play the music
def play_music(tune, durt, delay_time):

    for i in range(len(tune)):
        if tune[i] is not None:
            freq = int(tune[i])
            if freq < 1:
                freq = 1
            elif freq > 40000000:
                freq = 40000000
            buzzer_pin.freq(freq)
            buzzer_pin.duty_u16(32767)  # 50% duty cycle
        time.sleep(delay_time * durt[i])
        buzzer_pin.duty_u16(0)  # Turn off the buzzer
        time.sleep(delay_time * durt[i] * 0.1)  # Add a small delay between notes
    
play_music(tune0, durt0, 0.5)
