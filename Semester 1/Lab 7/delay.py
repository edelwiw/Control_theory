from ev3dev.ev3 import LargeMotor 
import time 
	 
motorA = LargeMotor('outA') 
measurement_time = 10

capturing_period = 0.05 # s 


class Controller (object):
    def __init__(self):
        self.control_constrains = 1
    
    def control_func(self, error, time_delta):
        pass
    
    def constrain_control(self, control):
        return max(-1, min(1, control))
    
    
class P_controller(Controller):
    def __init__(self, Kp, delay_cycles):
        self.Kp = Kp 
        self.buffer = [0] * delay_cycles
        
    def control_func(self, error, time_delta):
        # self.buffer.append(error)
        # temp = self.buffer.pop(0)
        temp = error
        return self.constrain_control(self.Kp * temp)
    
    
def test(target_func, controller, measurement_time, file_path, delay):
    time_start = time.time()
    start_pos = motorA.position
    file = open(file_path, "w")
    print("Starting test")
    timer = 0
    time_delta = 0
    while timer < measurement_time:
        pos = (motorA.position - start_pos) / 180 * 3.14159 # get position 
        error = target_func(timer) - pos # get error
        # calculate control
        control = controller.control_func(error, time_delta) * 100 # percentage 
        motorA.run_direct(duty_cycle_sp=control)  
        
        time_delta = time.time() - timer
        timer = time.time() - time_start
        file.write(str(timer) + " " + str(pos) + " " + str(target_func(timer)) + " " + str(error) + " " + str(control) + "\n")
        time.sleep(delay)
    file.close()
    motorA.stop(stop_action='brake')
    
A = 5
const_target = lambda time: A 


delay_arr = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]

for delay_time in delay_arr:
    delay_cycles = int(delay_time / capturing_period)
    test(const_target, P_controller(1, delay_cycles), measurement_time, "P_controller_delayed_" + str(delay_time) + ".txt", delay_time)
    time.sleep(2)


motorA.stop(stop_action='brake')
    