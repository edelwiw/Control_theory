km = 0.3074;
ke = 0.3074;
J = 0.0019;
R = 4.6730;
L = 1.0337;

% km = 0.3361;
% ke = 0.3361;
% J = 0.0019;
% R = 4.7441;
% L = 1.0749;

sin_node = @(a, b) @(t) (a * sin(2 * 3.14 * b * t));
cos_node = @(a, b) @(t) (a * cos(2 * 3.14 * b * t));


test_freq_arr = [0.1, 0.5, 1, 5, 10, 20, 30, 40, 50, 70, 100, 150, 200, 300, 400, 500, 700, 900, 1000];
% test_freq_arr = [5];
test_amp = 1; 

function [A, time_delta] = get_amp_and_phase(y, u, t) 
    % get the derivative of signal, to find the phase
    du = diff(u) ./ diff(t);
    dy = diff(y) ./ diff(t);

    % find extremums by derivative
    signs_du_changes = du(1:end-1) .* du(2:end);
    signs_dy_changes = dy(1:end-1) .* dy(2:end);

    extremums_du = find(signs_du_changes < 0);
    extremums_dy = find(signs_dy_changes < 0);

    max_u_indexes = extremums_du(du(extremums_du) > 0);
    min_u_indexes = extremums_du(du(extremums_du) < 0);

    max_y_indexes = extremums_dy(dy(extremums_dy) > 0);
    min_y_indexes = extremums_dy(dy(extremums_dy) < 0);

    % delete indexes that less than u indexes
    max_y_indexes = max_y_indexes(max_y_indexes > max_u_indexes(1));
    min_y_indexes = min_y_indexes(min_y_indexes > min_u_indexes(1));

    % find min amount of indexes
    min_len = min([length(max_y_indexes), length(min_y_indexes), length(max_u_indexes), length(min_u_indexes)]);

    max_y_indexes = max_y_indexes(1:min_len);
    min_y_indexes = min_y_indexes(1:min_len);
    max_u_indexes = max_u_indexes(1:min_len);
    min_u_indexes = min_u_indexes(1:min_len);

    amp_y_array = y(max_y_indexes) - y(min_y_indexes);
    amp_u_array = u(max_u_indexes) - u(min_u_indexes);

    A = mean(amp_y_array) / mean(amp_u_array);
    time_delta = mean(t(max_u_indexes) - t(max_y_indexes));

end


%% task 1
% amp_arr = {};
% phase_arr = {};

% for i = 1:length(test_freq_arr)
%     test_freq = test_freq_arr(i);
%     func = sin_node(test_amp, test_freq);

%     sim_time = 1/test_freq * 10;
%     res = sim("scheme1.slx", sim_time); 
%     plotter({{res.y.Time, res.y.Data, "y"}, {res.u.Time, res.u.Data, "u"}}, sprintf("media/plots/simres/task1_F=%.1f.png", test_freq), "t", "y", sprintf("\\omega=%.1f", test_freq));

%     % get amplitude and phase 

%     % skip 20% of signal (for skipping transient)
%     skip = round(length(res.y.Data) * 0.2);
%     y = res.y.Data(skip:end);
%     u = res.u.Data(skip:end);
%     t = res.y.Time(skip:end);

%     % find max and min of y, u and its indexes
%     [A, time_delta] = get_amp_and_phase(y, u, t);
%     amp_arr{end + 1} = A;
%     phase_arr{end + 1} = time_delta * test_freq * 2 * 3.14;

% end

% plotter({{test_freq_arr, cell2mat(amp_arr), "A"}}, "media/plots/task1_A.png", "\omega", "A", "");
% plotter({{test_freq_arr, cell2mat(phase_arr), "phi"}}, "media/plots/task1_phi.png", "\omega", "\phi", "");

% fprintf("Amplitudes: %s\n", strjoin(cellfun(@num2str, amp_arr, 'UniformOutput', false), ", "));
% fprintf("Phases: %s\n", strjoin(cellfun(@num2str, phase_arr, 'UniformOutput', false), ", "));

function solve(transfer_function, mag_func, phase_func, time_domain_time, impulse_response_func, step_response_func, freq_domain_time, path)
    % find impuls and step response 
    time = linspace(0, time_domain_time, 1000);

    % by matlab function
    impulse_response_exp = impulse(transfer_function, time);
    step_response_exp = step(transfer_function, time);

    % plotter({{time, impulse_response_exp, "Impulse response"}}, sprintf("media/plots/%s_impulse_response_exp.png", path), "t", "y", "");
    % plotter({{time, step_response_exp, "Step response"}}, sprintf("media/plots/%s_step_response_exp.png", path), "t", "y", "");

    % by equations
    impulse_response_eq = arrayfun(impulse_response_func, time);
    step_response_eq = arrayfun(step_response_func, time);

    % plotter({{time, impulse_response_eq, "Impulse response"}}, sprintf("media/plots/%s_impulse_response_eq.png", path), "t", "y", "");
    % plotter({{time, step_response_eq, "Step response"}}, sprintf("media/plots/%s_step_response_eq.png", path), "t", "y", "");

    % compare 
    plotter({{time, impulse_response_exp, "Experemental"}, {time, impulse_response_eq, "Theoretical"}}, sprintf("media/plots/%s_impulse_response_cmp.png", path), "t", "y", "");
    plotter({{time, step_response_exp, "Experemental"}, {time, step_response_eq, "Theoretical"}}, sprintf("media/plots/%s_step_response_cmp.png", path), "t", "y", "");

    %% find freq response
    
    omega = logspace(-3, log10(freq_domain_time), 1000);

    % by matlab function
    numerator = transfer_function.Numerator{1};
    denominator = transfer_function.Denominator{1};

    h = freqs(numerator, denominator, omega);
    mag_exp = abs(h);
    phase_exp = angle(h);

    % plot_freq_response({{mag_exp, phase_exp, omega, "Experemental"}},  sprintf("media/plots/%s_freq_resp_exp_loglog.png", path), "loglog");
    % plot_freq_response({{mag_exp, phase_exp, omega_lin, "Experemental"}},  sprintf("media/plots/%s_freq_resp_exp_lin.png", path), "lin");

    % by equations
    mag_eq = arrayfun(mag_func, omega);
    phase_eq = arrayfun(phase_func, omega);

    % plot_freq_response({{mag_eq, phase_eq, omega_lin, "Theoretical"}}, sprintf("media/plots/%s_freq_resp_eq_lin.png", path), "lin");
    % plot_freq_response({{mag_eq, phase_eq, omega, "Theoretical"}}, sprintf("media/plots/%s_freq_resp_eq_loglog.png", path), "loglog");

    % compare
    plot_freq_response({{mag_eq, phase_eq, omega, "Theoretical"}, {mag_exp, phase_exp, omega, "Experemental"}}, sprintf("media/plots/%s_freq_resp_cmp_lin.png", path), "lin");
    plot_freq_response({{mag_eq, phase_eq, omega, "Theoretical"}, {mag_exp, phase_exp, omega, "Experemental"}}, sprintf("media/plots/%s_freq_resp_cmp_loglog.png", path), "loglog");

end 


%% task 1 

T = J * R / (ke * km);
transfer_function = tf([1 / ke], [T, 1]);

impulse_response_func = @(t) (1 / (T * ke)) * exp(-t / T);
step_response_func = @(t) 1 / ke * (1 - exp(-t / T));

mag_function = @(omega) 1 / (ke * sqrt(1 + (omega * T)^2));
phase_function = @(omega) -atan(omega * T);

% solve(transfer_function, mag_function, phase_function, 3, impulse_response_func, step_response_func, 3, "task1");

%% task2 
transfer_function = tf([km], [L * J, R * J, km * ke]);

% transfer_function = tf([1/ke], [T^2, 2 * T * beta, 1]);
poles = pole(transfer_function);
disp(poles);

D = R^2 * J^2 - 4 * L * J * km * ke;
s1 = (-R * J + sqrt(D)) / (2 * L * J);
s2 = (-R * J - sqrt(D)) / (2 * L * J);
fprintf("s1: %.4f + %.4fj, s2:%.4f + %.4fj", real(s1), imag(s1), real(s2), imag(s2));


T = sqrt((J * L) / (km * ke));
beta = (R * J) / (2 * T * km * ke);

lambda = beta / T; 
delta = sqrt(1 - beta^2) / T;


impulse_response_func = @(t) ((1/ke) / (delta * T^2)) * exp(-lambda * t) * sin(delta * t);
step_response_func = @(t) (1/ke) * (1 - exp(-lambda * t) * (cos(delta * t) + (lambda / delta) * sin(delta * t)));

mag_function = @(omega) 1/ke / sqrt((1 - omega^2 * T^2)^2 + (2 * beta * omega * T)^2);
phase_function = @(omega) -atan2(2 * beta * omega * T, 1 - omega^2 * T^2);

% solve(transfer_function, mag_function, phase_function, 3, impulse_response_func, step_response_func, 3, "task2");

%% task3
C = 419; 
transfer_function = tf([1], [C, 0]);

impulse_response_func = @(t) (1 / C);
step_response_func = @(t) t / C;

mag_function = @(omega) 1 / (omega * C);
phase_function = @(omega) -pi / 2;

% solve(transfer_function, mag_function, phase_function, 3, impulse_response_func, step_response_func, 3, "task3");

%% task4
m = 14;
k = 71; 

transfer_function = tf([1], [m, 0, k]);

T = sqrt(m / k);
impulse_response_func = @(t) 1 / (k * T) * sin(t / T);
step_response_func = @(t) 1 / k * (1 - cos(t / T));

mag_function = @(omega) (1 / k) / abs(1 - T^2 * omega^2);
phase_function = @(omega) -atan2(0, (1 / k) / (1 - T^2 * omega^2));

% solve(transfer_function, mag_function, phase_function, 3, impulse_response_func, step_response_func, 3, "task4");

%% task5
R1 = 1296;
R2 = 16853;
C = 419 * 10^-6;

T1 = R1 * C;
T2 = R2 * C;

transfer_function = tf([T2, 1], [T1, 0]);

delta = @(x) double(x==0);
impulse_response_func = @(t) 1 / T1 - T2 * delta(t) / T1;
step_response_func = @(t) 1 * (T2 + t) / T1;

mag_function = @(omega) sqrt((T2 / T1)^2 + 1/(T1^2 * omega^2));
phase_function = @(omega) -atan2(1 / (T1 * omega), 1 * T2 / T1);

solve(transfer_function, mag_function, phase_function, 3, impulse_response_func, step_response_func, 10, "task5");