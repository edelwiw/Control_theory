function U = get_control_matrix(A, B) 
    U = [B, A * B, A^2 * B];
end

function W = get_observability_matrix(A, C)
    W = [C; C * A; C * A^2];
end

function s = check_state_controllability(A, B, x1)
    U = get_control_matrix(A, B);
    U_hat = [U, x1];
    s = rank(U) == rank(U_hat);
end

function P = get_gramian(A, B, C, D, mode, t1)
    sys = ss(A, B, C, D);
    opt = gramOptions('TimeIntervals', [0, t1]);
    P = gram(sys, mode, opt);
end

function P = get_gramian_manual(A, B, C, D, mode, t1)
    if mode == 'c'
        P = integral(@(tau) expm(A * tau) * B * B' * expm(A' * tau), 0, t1, 'ArrayValued', true);
    end
    if mode == 'o'
        P = integral(@(tau) expm(A' * tau) * C' * C * expm(A * tau), 0, t1, 'ArrayValued', true);
    end
end

function [t, u] = get_control(A, B, x1, t1, num_points)
    U = get_control_matrix(A, B);
    P = get_gramian(A, B, [], [], 'c', t1);
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

function x0 = get_initial_state(A, C, y, t1)
    Q = get_gramian_manual(A, [], C, [], 'o', t1);
    x0 = pinv(Q) * integral(@(tau) expm(A' * tau) * C' * y(tau), 0, t1, 'ArrayValued', true);
end

function H = get_hautus_matrix(A, B, C, lambda, mode)
    if mode == 'c'
        H = [A - lambda * eye(size(A)), B];
    end
    if mode == 'o'
        H = [A - lambda * eye(size(A)); C];
    end
end

function spectral(A, B, C, mode)
    [V, D] = eig(A);
    fprintf("Eigenvalues: \n");
    for i = 1:size(D, 1)
        if imag(D(i, i)) == 0
            fprintf("\\lambda_%d = %.2f", i, D(i, i));
        else
            if imag(D(i, i)) > 0
                fprintf("\\lambda_%d = %.2f + %.2fj", i, real(D(i, i)), imag(D(i, i)));
            else
                fprintf("\\lambda_%d = %.2f - %.2fj", i, real(D(i, i)), -imag(D(i, i)));
            end
        end

        H = get_hautus_matrix(A, B, C, D(i, i), mode);
        print_matrix(H, 0);
        fprintf("\n\\text{rank}(H_%d) = %d\n\n", i, rank(H));
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

P = get_gramian(A, B, [], [], 'c', t1);
fprintf("\nGramian matrix \nP(%d) = ", t1);
print_matrix(P, 2);
fprintf("\n");

P = get_gramian_manual(A, B, [], [], 'c', t1);
fprintf("\nGramian matrix (manual) \nP(%d) = ", t1);
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

%% TASK 3
A = [-10, -7, -18;
    -3, -4, -8; 
    8, 2, 11];
C = [2, -3, 1];

W = get_observability_matrix(A, C);
fprintf("\nObservability matrix \nW = ");
print_matrix(W, 0);
fprintf("\nRank of observability matrix: %d\n\n", rank(W));

Q = get_gramian_manual(A, [], C, [], 'o', 3);
fprintf("\nGramian matrix \nQ(3) = ");
print_matrix(Q, 2);

y = @(t) exp(-2 .* t) .* cos(5 .* t) - exp(-2 .* t) .* sin(5 .* t);
x0 = get_initial_state(A, C, y, 3);
fprintf("\nInitial state: %.2f %.2f %.2f\n", x0(1), x0(2), x0(3));

sys = ss(A, [0; 0; 0], C, []);
% initial state is x0
t = linspace(0, 3, 1000);
u = zeros(1, length(t));
[y_hat, t, x] = lsim(sys, u, t, x0);

plotter({{t, x(:, 1), "x_1"}, {t, x(:, 2), "x_2"}, {t, x(:, 3), "x_3"}}, "media/plots/task3_states.png", "t", "x", "");
plotter({{t, y_hat, "y_{real}"}, {t, y(t), "y_{expected}"}}, "media/plots/task3_output.png", "t", "y", "");
plotter({{t, y_hat - y(t), "y_{error}"}}, "media/plots/task3_error.png", "t", "y", "");

%% TASK 4 
A = [-10, -7, -18;
    -3, -4, -8; 
    8, 2, 11];
C = [0, -1, -1];

W = get_observability_matrix(A, C);
fprintf("\nObservability matrix \nW = ");
print_matrix(W, 0);
fprintf("\nRank of observability matrix: %d\n\n", rank(W));

spectral(A, [], C, "o");

Q = get_gramian_manual(A, [], C, [], 'o', 3);
fprintf("\nGramian matrix \nQ(3) = ");
print_matrix(Q, 2);

y = @(t) exp(-2 .* t) .* cos(5 .* t) - exp(-2 .* t) .* sin(5 .* t);
x0 = get_initial_state(A, C, y, 3);
fprintf("\nInitial state: %.2f %.2f %.2f\n", x0(1), x0(2), x0(3));

sys = ss(A, [0; 0; 0], C, []);
% initial state is x0
t = linspace(0, 3, 1000);
u = zeros(1, length(t));
[y_hat, t, x] = lsim(sys, u, t, x0);

plotter({{t, x(:, 1), "x_1"}, {t, x(:, 2), "x_2"}, {t, x(:, 3), "x_3"}}, "media/plots/task4_states.png", "t", "x", "");
plotter({{t, y_hat, "y_{real}"}, {t, y(t), "y_{expected}"}}, "media/plots/task4_output.png", "t", "y", "");
plotter({{t, y_hat - y(t), "y_{error}"}}, "media/plots/task4_error.png", "t", "y", "");

%% 
function sym_hat(A, C, x0, y, n)
    x0_hat = x0 + [0; 0; 0];
    fprintf("\nInitial state: %.2f %.2f %.2f\n", x0_hat(1), x0_hat(2), x0_hat(3));

    sys = ss(A, [0; 0; 0], C, []);
    % initial state is x0
    t = linspace(0, 3, 1000);
    u = zeros(1, length(t));
    [y_hat, t, x] = lsim(sys, u, t, x0_hat);

    plotter({{t, x(:, 1), "x_1"}, {t, x(:, 2), "x_2"}, {t, x(:, 3), "x_3"}}, sprintf("media/plots/task4_states_hat_%d.png", n), "t", "x", "");  
    plotter({{t, y_hat, "y_{real}"}, {t, y(t), "y_{expected}"}}, sprintf("media/plots/task4_output_hat_%d.png", n), "t", "y", "");
    plotter({{t, y_hat - y(t), "y_{error}"}}, sprintf("media/plots/task4_error_hat_%d.png", n), "t", "y", "");
end 

x0 = get_initial_state(A, C, y, 3);
t = linspace(0, 3, 1000);
y = @(t) exp(-2 .* t) .* cos(5 .* t) - exp(-2 .* t) .* sin(5 .* t);

sym_hat(A, C, x0 + [-1; -1; 1], y, 1);
sym_hat(A, C, x0 + [-1; -1; 1] * 20, y, 2);
sym_hat(A, C, x0 + [-1; -1; 1] * 300, y, 3);

