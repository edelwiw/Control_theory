%% TASK 1 

A = [5, -2, 8; 
    4, -3, 4; 
    -4, 0, -7];
B = [-7; -5; 7];
x1 = [-2; -3; 3];
t1 = 3;

%% control matrix 
U = [B, A * B, A^2 * B];
fprintf("Control matrix \nU = \n");
print_matrix(U, 0);

fprintf("Rank of control matrix: %d\n\n", rank(U));

% gramian 
sys = ss(A, B, [], []);

opt = gramOptions('TimeIntervals', [0, t1]);
gramian = gram(sys, 'c', opt);
print_matrix(gramian, 2);


%% control 
u_func = @(tau) B' * expm(A' * (t1 - tau)) * inv(gramian) * x1;

% simulation
t = 0:0.01:t1;
u = zeros(1, length(t));
for i = 1:length(t)
    u(i) = u_func(t(i));
end

[y, t, x] = lsim(sys, u, t);

plotter({{t, u, "Control signal"}}, "media/plots/task1_control_signal.png", "t", "u", "");
plotter({{t, x(:, 1), "x_1"}, {t, x(:, 2), "x_2"}, {t, x(:, 3), "x_3"}}, "media/plots/task1_states.png", "t", "x", "");
fprintf("final state: %.2f %.2f %.2f\n", x(end, 1), x(end, 2), x(end, 3));


%% TASK 2
B = [-1; -3; 3];
x1 = [-2; -3; 3];

U = [B, A * B, A^2 * B];
fprintf("Control matrix \nU = \n");
print_matrix(U, 0);
fprintf("Rank of control matrix: %d\n\n", rank(U));

sys = ss(A, B, [], []);

opt = gramOptions('TimeIntervals', [0, t1]);
gramian = gram(sys, 'c', opt);
print_matrix(gramian, 2);

%% 
U_hat = [U, x1];
fprintf("rank of U_hat: %d\n", rank(U_hat));

%% control 
u_func = @(tau) B' * expm(A' * (t1 - tau)) * inv(gramian) * x1;

% simulation
t = 0:0.01:t1;
u = zeros(1, length(t));
for i = 1:length(t)
    u(i) = u_func(t(i));
end

[y, t, x] = lsim(sys, u, t);

plotter({{t, u, "Control signal"}}, "media/plots/task2_control_signal.png", "t", "u", "");
plotter({{t, x(:, 1), "x_1"}, {t, x(:, 2), "x_2"}, {t, x(:, 3), "x_3"}}, "media/plots/task2_states.png", "t", "x", "");
fprintf("final state: %.2f %.2f %.2f\n", x(end, 1), x(end, 2), x(end, 3));