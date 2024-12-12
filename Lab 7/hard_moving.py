from ev3dev.ev3 import LargeMotor 
import time 
from math import sin, cos
	 
motorA = LargeMotor('outA') 
measurement_time = 30

G = lambda A1, A2, w1, w2: lambda t: max(min(1, A1 * cos(w1 * t) + A2 * sin(w2 * t)), -1)


def test(control, measurement_time, file_path):
    time_start = time.time()
    start_pos = motorA.position
    file = open(file_path, "w")
    print("Starting test")
    timer = 0
    time_delta = 0
    while timer < measurement_time:
        pos = (motorA.position - start_pos) / 180 * 3.14159 # get position 
        motorA.run_direct(duty_cycle_sp=control(timer) * 100)  
        
        time_delta = time.time() - timer
        timer = time.time() - time_start
        file.write(str(timer) + " " + str(pos) + " " + str(control) + "\n")
    file.close()
    motorA.stop(stop_action='brake')
    

test(G(0.2, 0, 0.4, 0), measurement_time, "hard_moving_1.txt")
time.sleep(2)
test(G(0.1, 0.8, 0.5, 0.4), measurement_time, "hard_moving_2.txt")