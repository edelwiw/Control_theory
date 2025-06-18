const_node = @(a) @(t) a;
linear_node = @(a, b) @(t) (a * t + b);
sin_node = @(a, b) @(t) (a * sin(b * t));
cos_node = @(a, b) @(t) (a * cos(b * t));
power_node = @(a, b) @(t) (a * t^b);

error_func = @(a, b) abs(a - b);

%% task 1

a2 = 1; a1 = -1; a0 = -2; 
y0 = 2; dy0 = 3;
k0 = 0; k1 = 0; 

free_res = sim("scheme1.slx", 10);
plotter({{free_res.y.Time, free_res.y.Data, "y"}}, "media/plots/task1_freeout.png", "t", "y");

k0 = -3; k1 = -3; 

closed_res = sim("scheme1.slx", 10);

plotter({{closed_res.y.Time, closed_res.y.Data, "y"}}, "media/plots/task1_out.png", "t", "y");
plotter({{closed_res.u.Time, closed_res.u.Data, "g"}}, "media/plots/task1_in.png", "t", "g");
% 
closed_res = sim("scheme1.slx", 1); % lover time for better comparison
k0 = 0; k1 = 0; 
free_res = sim("scheme1.slx", 1);
plotter({{free_res.y.Time, free_res.y.Data, "y_{free}"}, {closed_res.y.Time, closed_res.y.Data, "y_{closed}"}}, "media/plots/task1_comp.png", "t", "y");


%% task 2

k0 = -3; k1 = -3; 
res_arr = {};
err_arr = {};

T = 0.73; 
closed_res = sim("scheme2.slx", 50);

res_arr{end + 1} = {closed_res.y.Time; closed_res.y.Data; "y_{T=0.73}"};
err_arr{end + 1} = {closed_res.y.Time, arrayfun(error_func, closed_res.y.Data, arrayfun(const_node(2), closed_res.y.Time)), "error_{T=0.73}"};

% plotter({{closed_res.y.Time, closed_res.y.Data, "y"}}, "media/plots/task2_out.png", "t", "y");
% plotter({{closed_res.u.Time, closed_res.u.Data, "g"}}, "media/plots/task2_in.png", "t", "g");

T = 0.74; 
closed_res = sim("scheme2.slx", 50);

res_arr{end + 1} = {closed_res.y.Time, closed_res.y.Data, "y_{T=0.74}"};
err_arr{end + 1} = {closed_res.y.Time, arrayfun(error_func, closed_res.y.Data, arrayfun(const_node(2), closed_res.y.Time)), "error_{T=0.74}"};

% plotter({{closed_res.y.Time, closed_res.y.Data, "y"}}, "media/plots/task2_out2.png", "t", "y");
% plotter({{closed_res.u.Time, closed_res.u.Data, "g"}}, "media/plots/task2_in2.png", "t", "g");

T = 0.001; 
closed_res = sim("scheme2.slx", 50);

res_arr{end + 1} = {closed_res.y.Time, closed_res.y.Data, "y_{T=0.001}"};
err_arr{end + 1} = {closed_res.y.Time, arrayfun(error_func, closed_res.y.Data, arrayfun(const_node(2), closed_res.y.Time)), "error_{T=0.001}"};

% plotter({{closed_res.y.Time, closed_res.y.Data, "y"}}, "media/plots/task2_out3.png", "t", "y");
% plotter({{closed_res.u.Time, closed_res.u.Data, "g"}}, "media/plots/task2_in3.png", "t", "g");

plotter(res_arr, "media/plots/task2_out.png", "t", "y");
plotter(err_arr, "media/plots/task2_error.png", "t", "error");

%% task 3
A = 2;
func = const_node(A);
res_arr = {};
err_arr = {};

k = -0.1;
res = sim("scheme3.slx", 10);
set_value = res.y.Data(end);
disp(fprintf("k = %f, y = %f, error = %f", k, set_value, abs(A - set_value)));
res_arr{end + 1} = {res.u.Time, res.u.Data, "g"};

res_arr{end + 1} = {res.y.Time, res.y.Data, "y_{k=-0.1}"};
err_arr{end + 1} = {res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error_{k=-0.1}"};

% plotter({{res.y.Time, res.y.Data, "y"}, {res.u.Time, res.u.Data, "g"}}, "media/plots/task3_out.png", "t", "y");

k = 0.1;
res = sim("scheme3.slx", 10);
set_value = res.y.Data(end);
disp(fprintf("k = %f, y = %f, error = %f", k, set_value, abs(A - set_value)));

res_arr{end + 1} = {res.y.Time, res.y.Data, "y_{k=0.1}"};
err_arr{end + 1} = {res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error_{k=0.1}"};

% plotter({{res.y.Time, res.y.Data, "y"}, {res.u.Time, res.u.Data, "g"}}, "media/plots/task3_out2.png", "t", "y");

k = 1;
res = sim("scheme3.slx", 10);
set_value = res.y.Data(end);
disp(fprintf("k = %f, y = %f, error = %f", k, set_value, abs(A - set_value)));

res_arr{end + 1} = {res.y.Time, res.y.Data, "y_{k=1}"};
err_arr{end + 1} = {res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error_{k=1}"};

% plotter({{res.y.Time, res.y.Data, "y"}, {res.u.Time, res.u.Data, "g"}}, "media/plots/task3_out3.png", "t", "y");

k = 10; 
res = sim("scheme3.slx", 10);
set_value = res.y.Data(end);
disp(fprintf("k = %f, y = %f, error = %f", k, set_value, abs(A - set_value)));

res_arr{end + 1} = {res.y.Time, res.y.Data, "y_{k=10}"};
err_arr{end + 1} = {res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error_{k=10}"};

% plotter({{res.y.Time, res.y.Data, "y"}, {res.u.Time, res.u.Data, "g"}}, "media/plots/task3_out4.png", "t", "y");

plotter(res_arr, "media/plots/task3_out1.png", "t", "y");
plotter(err_arr, "media/plots/task3_error1.png", "t", "error");

V = 1;
func = linear_node(A, 0); 

res_arr = {};
err_arr = {};

k = -0.1;
res = sim("scheme3.slx", 20);
res_arr{end + 1} = {res.u.Time, res.u.Data, "g"};

res_arr{end + 1} = {res.y.Time, res.y.Data, "y_{k=-0.1}"};
err_arr{end + 1} = {res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error_{k=-0.1}"};

% plotter({{res.y.Time, res.y.Data, "y"}, {res.u.Time, res.u.Data, "g"}}, "media/plots/task3_out5.png", "t", "y");
% plotter({{res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error"}}, "media/plots/task3_error5.png", "t", "error");

k = 0.1;
res = sim("scheme3.slx", 20);

res_arr{end + 1} = {res.y.Time, res.y.Data, "y_{k=0.1}"};
err_arr{end + 1} = {res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error_{k=0.1}"};

% plotter({{res.y.Time, res.y.Data, "y"}, {res.u.Time, res.u.Data, "g"}}, "media/plots/task3_out6.png", "t", "y");
% plotter({{res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error"}}, "media/plots/task3_error6.png", "t", "error");

k = 1;
res = sim("scheme3.slx", 20);

res_arr{end + 1} = {res.y.Time, res.y.Data, "y_{k=1}"};
err_arr{end + 1} = {res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error_{k=1}"};

% plotter({{res.y.Time, res.y.Data, "y"}, {res.u.Time, res.u.Data, "g"}}, "media/plots/task3_out7.png", "t", "y");
% plotter({{res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error"}}, "media/plots/task3_error7.png", "t", "error");

k = 10;
res = sim("scheme3.slx", 20);

res_arr{end + 1} = {res.y.Time, res.y.Data, "y_{k=10}"};
err_arr{end + 1} = {res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error_{k=10}"};

% plotter({{res.y.Time, res.y.Data, "y"}, {res.u.Time, res.u.Data, "g"}}, "media/plots/task3_out8.png", "t", "y");
% plotter({{res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error"}}, "media/plots/task3_error8.png", "t", "error");

plotter(res_arr, "media/plots/task3_out2.png", "t", "y");
plotter(err_arr, "media/plots/task3_error2.png", "t", "error");

%% task 4

func = const_node(A);

res_arr = {};
err_arr = {};

k = 0.1;
res = sim("scheme4.slx", 60);
res_arr{end + 1} = {res.u.Time, res.u.Data, "g"};


res_arr{end + 1} = {res.y.Time, res.y.Data, "y_{k=0.1}"};
err_arr{end + 1} = {res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error_{k=0.1}"};

% plotter({{res.y.Time, res.y.Data, "y"}, {res.u.Time, res.u.Data, "g"}}, "media/plots/task4_out.png", "t", "y");
% plotter({{res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error"}}, "media/plots/task4_error.png", "t", "error");


k = 0.3;
res = sim("scheme4.slx", 60);

res_arr{end + 1} = {res.y.Time, res.y.Data, "y_{k=0.3}"};
err_arr{end + 1} = {res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error_{k=0.3}"};

% plotter({{res.y.Time, res.y.Data, "y"}, {res.u.Time, res.u.Data, "g"}}, "media/plots/task4_out2.png", "t", "y");
% plotter({{res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error"}}, "media/plots/task4_error2.png", "t", "error");

k = 0.5;
res = sim("scheme4.slx", 60);

res_arr{end + 1} = {res.y.Time, res.y.Data, "y_{k=0.5}"};
err_arr{end + 1} = {res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error_{k=0.5}"};

% plotter({{res.y.Time, res.y.Data, "y"}, {res.u.Time, res.u.Data, "g"}}, "media/plots/task4_out3.png", "t", "y");
% plotter({{res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error"}}, "media/plots/task4_error3.png", "t", "error");
plotter(res_arr, "media/plots/task4_out1.png", "t", "y");
plotter(err_arr, "media/plots/task4_error1.png", "t", "error");


func = linear_node(V, 0);
% 

res_arr = {};
err_arr = {};

k = 0.1;
res = sim("scheme4.slx", 60);
res_arr{end + 1} = {res.u.Time, res.u.Data, "g"};

res_arr{end + 1} = {res.y.Time, res.y.Data, "y_{k=0.1}"};
err_arr{end + 1} = {res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error_{k=0.1}"};

% plotter({{res.y.Time, res.y.Data, "y"}, {res.u.Time, res.u.Data, "g"}}, "media/plots/task4_out4.png", "t", "y");
% plotter({{res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error"}}, "media/plots/task4_error4.png", "t", "error");
error = arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time));
disp(fprintf("k = %f, error = %f", k, error(end)));

k = 0.3;
res = sim("scheme4.slx", 60);

res_arr{end + 1} = {res.y.Time, res.y.Data, "y_{k=0.3}"};
err_arr{end + 1} = {res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error_{k=0.3}"}; 

% plotter({{res.y.Time, res.y.Data, "y"}, {res.u.Time, res.u.Data, "g"}}, "media/plots/task4_out5.png", "t", "y");
% plotter({{res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error"}}, "media/plots/task4_error5.png", "t", "error");
error = arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time));
disp(fprintf("k = %f, error = %f", k, error(end)));

k = 0.5;
res = sim("scheme4.slx", 60);

res_arr{end + 1} = {res.y.Time, res.y.Data, "y_{k=0.5}"};
err_arr{end + 1} = {res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error_{k=0.5}"};

% plotter({{res.y.Time, res.y.Data, "y"}, {res.u.Time, res.u.Data, "g"}}, "media/plots/task4_out6.png", "t", "y");
% plotter({{res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error"}}, "media/plots/task4_error6.png", "t", "error");
error = arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time));
disp(fprintf("k = %f, error = %f", k, error(end)));

plotter(res_arr, "media/plots/task4_out2.png", "t", "y");
plotter(err_arr, "media/plots/task4_error2.png", "t", "error");

a = 0.25; 
func = power_node(a, 2);

res_arr = {};
err_arr = {};


k = 0.1;
res = sim("scheme4.slx", 100);
res_arr{end + 1} = {res.u.Time, res.u.Data, "g"};

res_arr{end + 1} = {res.y.Time, res.y.Data, "y_{k=0.1}"};
err_arr{end + 1} = {res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error_{k=0.1}"};

% plotter({{res.y.Time, res.y.Data, "y"}, {res.u.Time, res.u.Data, "g"}}, "media/plots/task4_out7.png", "t", "y");
% plotter({{res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error"}}, "media/plots/task4_error7.png", "t", "error");

k = 0.3;
res = sim("scheme4.slx", 100);

res_arr{end + 1} = {res.y.Time, res.y.Data, "y_{k=0.3}"};
err_arr{end + 1} = {res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error_{k=0.3}"};

% plotter({{res.y.Time, res.y.Data, "y"}, {res.u.Time, res.u.Data, "g"}}, "media/plots/task4_out8.png", "t", "y");
% plotter({{res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error"}}, "media/plots/task4_error8.png", "t", "error");

k = 0.5;
res = sim("scheme4.slx", 100);

res_arr{end + 1} = {res.y.Time, res.y.Data, "y_{k=0.5}"};
err_arr{end + 1} = {res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error_{k=0.5}"};

% plotter({{res.y.Time, res.y.Data, "y"}, {res.u.Time, res.u.Data, "g"}}, "media/plots/task4_out9.png", "t", "y");
% plotter({{res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error"}}, "media/plots/task4_error9.png", "t", "error");

plotter(res_arr, "media/plots/task4_out3.png", "t", "y");
plotter(err_arr, "media/plots/task4_error3.png", "t", "error");

%% task 5 

kp_arr = [1, 5, 10];
ki_arr = [0.1, 0.3, 0.5];

func = linear_node(V, 0);

res = sim("scheme5.slx", 100);

for kp = kp_arr
    to_plot = {{res.u.Time, res.u.Data, "g"}};
    to_plot_error = {};

    for ki = ki_arr
        res = sim("scheme5.slx", 100);
        % plotter({{res.y.Time, res.y.Data, "y"}, {res.u.Time, res.u.Data, "g"}}, sprintf("media/plots/task5_out_kp_%.1f_ki_%.1f_2.png", kp, ki), "t", "y");
        to_plot{end + 1} = {res.y.Time, res.y.Data, sprintf("y_{kp=%.1f, ki=%.1f}", kp, ki)};
        % plotter({{res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error"}}, sprintf("media/plots/task5_error_kp_%.1f_ki_%.1f_2.png", kp, ki), "t", "error");
        to_plot_error{end + 1} = {res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), sprintf("error_{kp=%.1f, ki=%.1f}", kp, ki)};
    end

    plotter(to_plot, sprintf("media/plots/task5_out_kp_%.1f_1.png", kp), "t", "y");
    plotter(to_plot_error, sprintf("media/plots/task5_error_kp_%.1f_1.png", kp), "t", "error");
end



func = sin_node(A, 0.25);
res = sim("scheme5.slx", 100);

for kp = kp_arr
    to_plot = {{res.u.Time, res.u.Data, "g"}};
    to_plot_error = {};

    for ki = ki_arr
        res = sim("scheme5.slx", 100);
        % plotter({{res.y.Time, res.y.Data, "y"}, {res.u.Time, res.u.Data, "g"}}, sprintf("media/plots/task5_out_kp_%.1f_ki_%.1f_2.png", kp, ki), "t", "y");
        to_plot{end + 1} = {res.y.Time, res.y.Data, sprintf("y_{kp=%.1f, ki=%.1f}", kp, ki)};
        % plotter({{res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error"}}, sprintf("media/plots/task5_error_kp_%.1f_ki_%.1f_2.png", kp, ki), "t", "error");
        to_plot_error{end + 1} = {res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), sprintf("error_{kp=%.1f, ki=%.1f}", kp, ki)};
    end

    plotter(to_plot, sprintf("media/plots/task5_out_kp_%.1f_2.png", kp), "t", "y");
    plotter(to_plot_error, sprintf("media/plots/task5_error_kp_%.1f_2.png", kp), "t", "error");
end



%% task 6 

func = sin_node(A, 0.25);

res = sim("scheme6.slx", 100);
plotter({{res.y.Time, res.y.Data, "y"}, {res.u.Time, res.u.Data, "g"}}, "media/plots/task6_out.png", "t", "y");
plotter({{res.y.Time, arrayfun(error_func, res.y.Data, arrayfun(func, res.y.Time)), "error"}}, "media/plots/task6_error.png", "t", "error");