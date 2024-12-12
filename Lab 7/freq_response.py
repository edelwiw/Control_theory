from ev3dev.ev3 import LargeMotor 
import time 
from math import sin 
	 
motorA = LargeMotor('outA') 
measurement_time = 3 


freq_array = [0.1, 0.3, 0.5, 0.7, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5] # Rad/s

for test_num in range(len(freq_array)):
    file = open("freq_response_" + str(test_num) + ".txt", "w")
    print("Test number: " + str(test_num))
    timeStart = time.time() 
    timer = 0
    startPos = motorA.position
    omega = freq_array[test_num]
    while time.time() - timeStart < measurement_time:
        pos = (motorA.position - startPos) / 180 * 3.14159
        timer = time.time() - timeStart
        motorA.run_direct(duty_cycle_sp=sin(timer * omega) * 100) 
        
        file.write(str(timer) + " " + str(pos) + "\n")
    motorA.stop(stop_action='brake') 
    file.close()
    print("Test finished")