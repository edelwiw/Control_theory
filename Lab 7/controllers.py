from ev3dev.ev3 import LargeMotor 
import time 
	 
motorA = LargeMotor('outA') 
measurement_time = 5

# constants for targets 
A = 10 # rad 
V = 2.5 # rad/s 
A = 0.5 # rad/s^2

# sin wave
Amp = 2
phase = 0.5
omega = 2

# PI controller
Kp = 2.0 
Ki = 0.5 

# special controller # TODO: set right values
a = 5
b = 4
c = 4


class Controller (object):
    def __init__(self):
        self.control_constrains = 1
    
    def control_func(self, error, time_delta):
        pass
    
    def constrain_control(self, control):
        return max(-self.control_constrains, min(self.control_constrains, control))

    
class P_controller(Controller):
    def __init__(self, Kp):
        self.Kp = Kp 
        
    def control_func(self, error, time_delta):
        return self.constrain_control(self.Kp * error)
    

class PI_controller(Controller):
        def __init__(self, Kp, Ki):
            self.Kp = Kp 
            self.Ki = Ki
            self.first_error_int = 0
        
        def control_func(self, error, time_delta):
            self.first_error_int += error
            control =  self.Kp * error + self.Ki * self.first_error_int * time_delta
            return self.constrain_control(control)
        
        
class Special_controller(Controller):
    def __init__(self, a, b, c, omega):
        self.a = a
        self.b = b
        self.c = c
        self.omega = omega
        
        # derivatives
        self.err_first_derivate = 0
        self.err_second_derivate = 0
        self.control_first_derivate = 0
        self.control_second_derivate = 0
        
        # utils 
        self.prev_error = 0
        self.prev_error_first_derivate = 0
        self.prev_error_second_derivate = 0
        self.prev_control = 0
        self.prev_control_first_derivate = 0
        self.prev_control_second_derivate = 0
        
    
    def __calc__(self, error, time_delta):
        self.err_first_derivate = (error - self.prev_error) / time_delta
        self.err_second_derivate = (self.err_first_derivate - self.prev_error_first_derivate) / time_delta
        self.control_first_derivate = (self.control - self.prev_control) / time_delta
        self.control_second_derivate = (self.control_first_derivate - self.prev_control_first_derivate) / time_delta
        
        
        self.prev_error = error
        self.prev_error_first_derivate = self.err_first_derivate
        self.prev_control = self.control
        self.prev_error_second_derivate = self.err_second_derivate
    
        
    def control_func(self, error, time_delta):
        self.__calc__(error, time_delta) 
        control = (self.a * err_second_derivate + self.b * err_first_derivate + self.c * error - self.control_second_derivate) / self.omega ** 2
        return self.constrain_control(control)



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
const_speed = lambda time: V * time
const_acceleration = lambda time: (A * time ** 2) / 2
wave = lambda time: Amp * sin(omega * time + phase)

p_controller = P_controller(Kp)
PI_controller = PI_controller(Kp, Ki)
special_controller = Special_controller(a, b, c, omega)

# P-controller 
test(const_target, p_controller, measurement_time, "p_controller_const_target.txt")
time.sleep(2)
test(const_speed, p_controller, measurement_time, "p_controller_const_speed.txt")
time.sleep(2)

# PI-controller
test(const_speed, pi_controller, measurement_time, "pi_controller_const_speed.txt")
time.sleep(2)
test(const_acceleration, pi_controller, measurement_time, "pi_controller_const_acceleration.txt")
time.sleep(2)

# special controller 
test(wave, special_controller, measurement_time, "special_controller_wave.txt")
time.sleep(2)