%% for simulation
a = 5;
b = 4;
c = 4;
w = 2;


%% ================== Task 1 ==================
% format f"{time.time() - timeStart} {pos}\n"
data_file_path = "step_response.txt"; 
% function to approximate 
% par(1) = k, par(2) = T
step_response = @(par, t) par(1) * t - par(1) * par(2) + par(1) * par(2) * exp(-t / par(2)); 

% read time and angle from file 
data = readmatrix(data_file_path); 
len = length(data); % read file and get data length 
time = data(1:len, 1); % extract time  
pos = data(1:len, 2); % extract angle and convert from degrees to rad 

% approximation 
par_start = [50; 0.1]; % params by default
opt=optimoptions('lsqcurvefit', 'StepTolerance',1e-6, 'FunctionTolerance', 1e-6, 'OptimalityTolerance', 1e-10, 'MaxFunctionEvaluations', 10e5, 'MaxIterations', 10e5); % set options
par = lsqcurvefit(step_response, par_start, time, pos, [], [], opt); % approximate and get right params

T = par(2); 
k = par(1); 
system = tf([k], [T 1 0]);

fprintf("Approximated params: k = %f, T = %f\n", k, T);

plotter({{time, pos, "Experimental"}, {time, step_response(par, time), "Theory"}}, "plots/task1_step_response_cmp.png", "Time, s", "Angle, rad", "Step response");



%% ================== Task 2 ==================
% format f"{time} {pos} {target_func(time)} {error} {control}\n"

function read_results(data_path, plot_path, title)
    data = readmatrix(data_path); 
    len = length(data); % read file and get data length
    time = data(1:len, 1); % extract time
    angle = data(1:len, 2); % extract angle (rad)
    target = data(1:len, 3); % extract target
    error = data(1:len, 4); % extract error
    control = data(1:len, 5); % extract control

    set_error = error(len); % set error
    fprintf("Set error: %f\n", set_error);
    x_label = "Time, s";
    y_label = "Value";
    plotter({{time, angle, "Angle"}, {time, target, "Target"}}, sprintf("%s_pos.png", plot_path), x_label, y_label, title);
    plotter({{time, error, "Error"}}, sprintf("%s_error.png", plot_path), x_label, y_label, title);
    plotter({{time, control, "Control"}}, sprintf("%s_control.png", plot_path), x_label, y_label, title);
end


% P-controller
read_results("task2_p_controller_const_target.txt", "plots/task2_p_controller_const_target", "P-controller, constant target");
read_results("task2_p_controller_const_speed.txt", "plots/task2_p_controller_const_speed", "P-controller, constant speed");

% PI-controller
read_results("task2_pi_controller_const_speed.txt", "plots/task2_pi_controller_const_speed", "PI-controller, constant speed");
read_results("task2_pi_controller_const_acceleration.txt", "plots/task2_pi_controller_const_acceleration", "PI-controller, constant acceleration");

% Special controller
read_results("task2_special_controller_wave.txt", "plots/task2_special_controller_wave", "Special controller, sin");



%% ================== Task 3 ==================
% format f"{timer} {pos}\n" 

function [amp, phase] = get_freq_response(data_path, freq)
    data = readmatrix(data_path); 
    len = length(data); % read file and get data length
    time = data(1:len, 1); % extract time
    pos = data(1:len, 2); % extract angle (rad)
    
    % approximation
    par_start = [20; 0]; % params by default    
    sin_approx = @(par, t) par(1) * sin(freq * t + par(2));
    opt=optimoptions('lsqcurvefit', 'StepTolerance', 1e-6, 'FunctionTolerance', 1e-6, 'OptimalityTolerance', 1e-10, 'MaxFunctionEvaluations', 10e5, 'MaxIterations', 10e5); % set options
    par = lsqcurvefit(sin_approx, par_start, time, pos, [], [], opt); % approximate and get right params

    amp = par(1);
    phase = par(2);
    
    fprintf("Approximated params for w = %f: A = %f, phi = %f\n", freq, amp, phase);
    plotter({{time, pos, "Experimental"}, {time, sin_approx(par, time), "Theory"}}, sprintf("plots/task3_freq_response_cmp_%f.png", freq), "Time, s", "Angle, rad", "Frequency response");
end


% par(1) = A, par(2) = phi
omega_arr = [0.1, 0.3, 0.5, 0.7, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5]; % frequencies

freq_response_results = zeros(length(omega_arr), 2); % store results

for test_num = 1:length(omega_arr)
    [amp, phase] = get_freq_response(sprintf("freq_response_%d.txt", test_num - 1), omega_arr(test_num));
    freq_response_results(test_num, 1) = amp;
    freq_response_results(test_num, 2) = phase;
end

% plot theory freq response
omega = 0.01:0.01:max(omega_arr);
h = freqs(system.num{1}, system.den{1}, omega);
mag_theory = abs(h);
phase_theory = angle(h);

plot_freq_response({{mag_theory, phase_theory, omega, "Theory"}}, "plots/task3_freq_response_theory.png", "lin");
plot_freq_response({{freq_response_results(:, 1), freq_response_results(:, 2), omega_arr, "Experiment"}}, "plots/task3_freq_response_experimental.png", "points");
plot_freq_response({{freq_response_results(:, 1), freq_response_results(:, 2), omega_arr, "Experiment"}, {mag_theory, phase_theory, omega, "Theory"}}, "plots/task3_freq_response_cmp.png", "lin");



%% ================== Task 4 ==================

function tau_critical = bode_plotter(transfer_function, freq_domain_power, filename)
    figure('Position', [10 10 900 600]);
    set(gca, 'LooseInset', get(gca, 'TightInset') * 2.0);
    
    grid("on");
    grid("minor");
    hold on;
    w = logspace(-2, freq_domain_power, 1000);
    h = bodeplot(transfer_function, w);
    grid("on");
    grid("minor");
    set(findall(gcf,'type','line'),'linewidth',2);
    
    h.showCharacteristic('AllStabilityMargins')
    [Gm, Pm, Wcg, Wcp] = margin(transfer_function);
    fprintf("Gm = %f dB, Pm = %f degrees, Wcg = %f rad/s, Wcp = %f rad/s\n", Gm, Pm, Wcg, Wcp);
    tau_critical = Pm / 180 * pi / Wcp;
    
    saveas(gcf, filename);
    % close(gcf);
end


function printusik(delay)
    data = readmatrix(sprintf("P_controller_delayed_%.1f.txt", delay));
    len = length(data); % read file and get data length
    time = data(1:len, 1); % extract time
    pos = data(1:len, 2); % extract angle (rad)
    target = data(1:len, 3); % extract target
    error = data(1:len, 4); % extract error
    control = data(1:len, 5); % extract control

    plotter({{time, pos, "Angle"}, {time, target, "Target"}}, sprintf("plots/task4_delay_pos_%d.png", delay), "Time, s", "Value", "Frequency response");
    plotter({{time, error, "Error"}}, sprintf("plots/task4_delay_error_%d.png", delay), "Time, s", "Value", "Frequency response");
    plotter({{time, control, "Control"}}, sprintf("plots/task4_delay_control_%d.png", delay), "Time, s", "Value", "Frequency response");
end

kp = 0.2;
p_controller_system = system * tf([kp], [1]);
tau_critical = bode_plotter(p_controller_system, 3, "plots/task4_bode.png");
fprintf("Critical time: %f\n", tau_critical);

delay_arr = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6];

for i = 1:length(delay_arr)
    printusik(delay_arr(i));
end



%% ================== Task 5 ==================
% format f"{timer} {pos} {control}\n"



function check_res(theory_function, data_path, n)
    data = readmatrix(data_path); 
    len = length(data); % read file and get data length
    time = data(1:len, 1); % extract time
    pos = data(1:len, 2); % extract angle (rad)
    control = data(1:len, 3); % extract control

    theory = theory_function(time);

    plotter({{time, pos, "Experimental"}, {time, theory, "Theory"}}, sprintf("plots/task5_hard_move_pos_%d.png", n), "Time, s", "Angle", "Hard move pos");
    plotter({{time, control, "Control"}}, sprintf("plots/task5_hard_move_control_%d.png", n), "Time, s", "Control", "Hard move control");
end

A1 = 0.2;
omega1 = 0.4;
y1 = @(t) A1 * k * omega1 / 2 * (-(T^2 * exp(-t / T) )/(T^2 * omega1^2 + 1) + (-T * omega1 * sin(t * omega1 - 5) - cos(t * omega1 - 5))/(omega1^2 * (T^2 * omega1 ^ 2 + 1)) + 1/omega1^2) - 4;
check_res(y1, "hard_moving_1.txt", 1);

A1 = 0.7;
A2 = 0.2;
omega1 = 2;
omega2 = 10;

y2 = @(t) k * ((A1 * (sin(t * omega1) / omega1 - T * cos(t * omega1)) / (T^2 * omega1^2 + 1)) - (A2 * (T * omega2 * sin(t * omega2) + cos(t * omega2)) / (omega2 * (T^2 * omega2^2 + 1))) + exp(-t/T) * (-A2 * T^5 * omega1^2 * omega2 + A1 * T^4 * omega2^2 - A2 * T^3 * omega2 + A1 * T^2) / (T * (T^2 * omega1^2 + 1) * (T^2 * omega2^2 + 1)) + A2 / omega2);
check_res(y2, "hard_moving_2.txt", 2);