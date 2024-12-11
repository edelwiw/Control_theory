from ev3dev.ev3 import LargeMotor 
import time 
	 
motorA = LargeMotor('outA') 
measurement_time = 3 


freq_array = [5, 4.5, 4, 3.5, 3, 2.5, 2, 1.5, 1, 0.7, 0.5, 0.3, 0.1] # Rad/s

for test_num in range(len(freq_array)):
    file = open(f"freq_response_{test_num}.txt", "w")
    print(f"Test number: {test_num}")
    timeStart = time.time() 
    timer = 0
    startPos = motorA.position
    omega = freq_array[test_num]
    while time.time() - timeStart < measurement_time:
        pos = (motorA.position - startPos) / 180 * 3.14159
        timer = time.time() - timeStart
        motorA.run_direct(duty_cycle_sp=sin(timer * omega) * 100) 
        
        file.write(f"{timer} {pos}\n")
    motorA.stop(stop_action='brake') 
    file.close()
    print(f"Test finished")