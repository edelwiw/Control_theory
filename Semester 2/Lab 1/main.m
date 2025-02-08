function U = get_control_matrix(A, B) 
    U = [B, A * B, A^2 * B];
end

function s = check_state_controllability(A, B, x1)
    U = get_control_matrix(A, B);
    U_hat = [U, x1];
    s = rank(U) == rank(U_hat);
end

function P = get_gramian(A, B, mode, t1)
    sys = ss(A, B, [], []);
    opt = gramOptions('TimeIntervals', [0, t1]);
    P = gram(sys, mode, opt);
end

function [t, u] = get_control(A, B, x1, t1, num_points)
    U = get_control_matrix(A, B);
    P = get_gramian(A, B, 'c', t1);
    if check_state_controllability(A, B, x1) == 0
        fprintf(2, "System is not state controllable\n");
        return;
    end
    u_func = @(tau) B' * expm(A' * (t1 - tau)) * pinv(P) * x1;
    t = linspace(0, t1, num_points);
    u = zeros(1, length(t));
    for i = 1:length(t)
        u(i) = u_func(t(i));
    end
end

%% TASK 1 

A = [5, -2, 8; 
    4, -3, 4; 
    -4, 0, -7];
B = [-7; -5; 7];
x1 = [-2; -3; 3];
t1 = 3;

U = get_control_matrix(A, B);
fprintf("\nControl matrix \nU = ");
print_matrix(U, 0); 
fprintf("\nRank of control matrix: %d\n\n", rank(U));

P = get_gramian(A, B, 'c', t1);
fprintf("\nGramian matrix \nP(%d) = ", t1);
print_matrix(P, 2);
fprintf("\n");

[t, u] = get_control(A, B, x1, t1, 1000);   
plotter({{t, u, "Control signal"}}, "media/plots/task1_control_signal.png", "t", "u", "");

sys = ss(A, B, [], []);
[y, t, x] = lsim(sys, u, t);

plotter({{t, x(:, 1), "x_1"}, {t, x(:, 2), "x_2"}, {t, x(:, 3), "x_3"}}, "media/plots/task1_states.png", "t", "x", "");
fprintf("final state: %.2f %.2f %.2f\n", x(end, 1), x(end, 2), x(end, 3));

%% TASK 2

A = [5, -2, 8; 
    4, -3, 4; 
    -4, 0, -7];
B = [-1; -3; 3];
x1 = [-2; -3; 3];
x2 = [-3; -3; 4];
t1 = 3;

if check_state_controllability(A, B, x1) == 1 
    fprintf("System is state controllable with x1\n");
else if check_state_controllability(A, B, x2) == 1
    fprintf("System is state controllable with x2\n");
    x1 = x2;
else
    fprintf(2, "System is not state controllable\n");
end 
end

[t, u] = get_control(A, B, x1, t1, 1000);
plotter({{t, u, "Control signal"}}, "media/plots/task2_control_signal.png", "t", "u", "");

sys = ss(A, B, [], []);
[y, t, x] = lsim(sys, u, t);

plotter({{t, x(:, 1), "x_1"}, {t, x(:, 2), "x_2"}, {t, x(:, 3), "x_3"}}, "media/plots/task2_states.png", "t", "x", "");
fprintf("final state: %.2f %.2f %.2f\n", x(end, 1), x(end, 2), x(end, 3));