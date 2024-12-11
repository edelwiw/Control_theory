from ev3dev.ev3 import LargeMotor 
import time 
	 
motorA = LargeMotor('outA') 
measurement_time = 5

capturing_period = 0.05 # s 


class Controller (object):
    def __init__(self):
        self.control_constrains = 1
    
    def control_func(self, error, time_delta):
        pass
    
    def constrain_control(self, control):
        return max(-self.control_constrains, min(self.control_constrains, control))
    
    
class P_controller(Controller):
    def __init__(self, Kp, delay_cycles):
        self.Kp = Kp 
        self.buffer = [0] * delay_cycles
        
    def control_func(self, error, time_delta):
        self.buffer.append(error)
        temp = self.buffer.pop(0)
        return self.constrain_control(self.Kp * temp)
    
    
def test(target_func, controller, measurement_time, file_path):
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
        file.write(f"{timer} {pos} {target_func(timer)} {error} {control}\n")
    file.close()
    motorA.stop(stop_action='brake')
    
    
const_target = lambda time: A 

delay_time = 0
delay_cycles = int(delay_time / capturing_period)
test(const_target, P_controller(1, delay_cycles), measurement_time, f"P_controller_delayed_{delay_time}.txt")
time.sleep(2)

delay_time = 0.1
delay_cycles = int(delay_time / capturing_period) 
test(const_target, P_controller(1, delay_cycles), measurement_time, f"P_controller_delayed_{delay_time}.txt")
time.sleep(2)

delay_time = 0.2
delay_cycles = int(delay_time / capturing_period)
test(const_target, P_controller(1, delay_cycles), measurement_time, f"P_controller_delayed_{delay_time}.txt")
time.sleep(2)

delay_time = 0.3
delay_cycles = int(delay_time / capturing_period)
test(const_target, P_controller(1, delay_cycles), measurement_time, f"P_controller_delayed_{delay_time}.txt")

motorA.stop(stop_action='brake')
    