from ev3dev.ev3 import LargeMotor 
import time 
	 
  
motorA = LargeMotor('outA') 
measurement_time = 5

file = open("step_response.txt", "w")

timeStart = time.time() 
startPos = motorA.position
motorA.run_direct(duty_cycle_sp=100) # 100% duty cycle
while time.time() - timeStart < measurement_time:
    pos = (motorA.position - startPos) / 180 * 3.14159 # rad 
    file.write(f"{time.time() - timeStart} {pos}\n")

motorA.stop(stop_action='brake') 
file.close()
    