% cvx setup
cvx_setup

%%


function U = get_control_matrix(A, B) 
    dim = size(A, 1);
    for i = 1:dim
        if i == 1
            U = B;
        else
            U = [U, A^(i - 1) * B];
        end
    end
end

function W = get_observability_matrix(A, C)
    dim = size(A, 1);
    for i = 1:dim
        if i == 1
            W = C;
        else
            W = [W; C * A^(i - 1)];
        end
    end
    % W = [C; C * A; C * A^2];
end

function H = get_hautus_matrix(A, B, C, lambda, mode)
    if mode == 'c'
        H = [A - lambda * eye(size(A)), B];
    end
    if mode == 'o'
        H = [A - lambda * eye(size(A)); C];
    end
end

function [A, P] = get_jordan_form(A) 
    [Pj, Aj] = jordan(A);
    [P, A] = cdf2rdf(Pj, Aj);
end


function K = find_controller(A, B, sigma)
    [Aj, P] = get_jordan_form(A);
    % fprintf("Jordan form of A: \n");
    % print_matrix(Aj, 2);
    % fprintf("P matrix: \n");
    % print_matrix(P, 2);
    Bj = inv(P) * B;
    % fprintf("Bj matrix: \n");
    % print_matrix(Bj, 2);

    % crop matrix 
    removed = [];
    dim = size(A, 1);
    for i = 1:dim
        eigval = Aj(i, i);
        if rank(get_hautus_matrix(A, B, [], eigval, 'c')) < dim
            % fprintf("Uncontrollable eigenvalue: %d, %d\n", eigval, i);
            % fill line with zeros
            Aj(i, :) = 0;
            Bj(i, :) = 0;
            % fill column with zeros
            Aj(:, i) = 0;
            removed = [removed, i];
        end
    end
    % clear zero rows and columns
    Aj = Aj(any(Aj, 2), :);
    Aj = Aj(:, any(Aj, 1));
    Bj = Bj(any(Bj, 2), :);
    % fprintf("Cropped Jordan form of A: \n");
    % print_matrix(Aj, 2);
    % fprintf("Cropped Bj matrix: \n");
    % print_matrix(Bj, 2);

    Kj = acker(Aj, Bj, sigma);
    % fprintf("Kj matrix: \n");
    % print_matrix(Kj, 2);

    % insert zeros to recover original size
    i = 1;
    Kr = [];
    for j = 1:dim
        if ismember(j, removed)
            Kr = [Kr, 0];
        else
            Kr = [Kr, Kj(i)]; 
            i = i + 1;
        end
    end

    % fprintf("Kr matrix: \n");
    % print_matrix(Kr, 2);

    K = Kr * inv(P);
end


%% 
A = [8, 1, 11; 4, 0, 4; -4, -3, -7];
B = [-1; -3; 3];
%% 
fprintf("TASK 1\n");
sigma = [-3, -3]; 
K = find_controller(A, B, sigma);
fprintf("K matrix: \n");
print_matrix(K, 2);
res = sim("task1", 5);
u_arr = res.u;
x_arr = res.x;
t_arr = res.tout;

system_matrix = A - B * K; 
fprintf("System matrix: \n");
print_matrix(system_matrix, 2);

plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, x_arr(:, 2), "x_2"}, {t_arr, x_arr(:, 3), "x_3"}}, "media/plots/task1_x_2.png", "t", "x", "");
plotter({{t_arr, u_arr(:, 1), "u"}}, "media/plots/task1_u_2.png", "t", "u", "");

%% 
fprintf("TASK 2\n");
sigma = [-30, -300]; 
K = find_controller(A, B, sigma);
fprintf("K matrix: \n");
print_matrix(K, 2);
res = sim("task1", 2);
u_arr = res.u;
x_arr = res.x;
t_arr = res.tout;

system_matrix = A - B * K; 
fprintf("System matrix: \n");
print_matrix(system_matrix, 2);

plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, x_arr(:, 2), "x_2"}, {t_arr, x_arr(:, 3), "x_3"}}, "media/plots/task1_x_4.png", "t", "x", "");
plotter({{t_arr, u_arr(:, 1), "u"}}, "media/plots/task1_u_4.png", "t", "u", "");

%%
fprintf("TASK 3\n");
sigma = [-3+9j, -3-9j]; 
K = find_controller(A, B, sigma);
fprintf("K matrix: \n");
print_matrix(K, 2);
res = sim("task1", 5);
u_arr = res.u;
x_arr = res.x;
t_arr = res.tout;

system_matrix = A - B * K; 
fprintf("System matrix: \n");
print_matrix(system_matrix, 2);

plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, x_arr(:, 2), "x_2"}, {t_arr, x_arr(:, 3), "x_3"}}, "media/plots/task1_x_6.png", "t", "x", "");
plotter({{t_arr, u_arr(:, 1), "u"}}, "media/plots/task1_u_6.png", "t", "u", "");

%% TASK 2 
A = [-40, 16, 9, 7; -64, 25, 14 ,12; -26, 11, 7, 3; -48, 18, 14, 8];
C = [-3, 2, -2, 1]; 

[Aj, P] = get_jordan_form(A);
fprintf("Jordan form of A: \n");
print_matrix(Aj, 2);
fprintf("P matrix: \n");
print_matrix(P, 2);
Cj = C * P; 
fprintf("Cj matrix: \n");
print_matrix(Cj, 2);

%% 
% find observer
G1 = [-1, 1, 0, 0;
    0, -1, 1, 0;
    0, 0, -1, 1;
    0, 0, 0, -1];

Y = [1; 1; 1; 1];
cvx_begin sdp
    variable Q(4, 4);
    G1 * Q - Q * A == Y * C;
cvx_end
L1 = Q \ Y;

fprintf("L1 matrix: \n");
print_matrix(L1, 2);

print_matrix(A + L1 * C, 5);

disp(eig(A + L1 * C));

G2 = [-1, 1, 0, 0;
    0, -10, 1, 0;
    0, 0, -100, 1;
    0, 0, 0, -100];

Y = [1; 1; 1; 1];
cvx_begin sdp
    variable Q(4, 4);
    G2 * Q - Q * A == Y * C;
cvx_end
L2 = Q \ Y;

fprintf("L2 matrix: \n");
print_matrix(L2, 2);

print_matrix(A + L2 * C, 5);

disp(eig(A + L2 * C));

G3 = [-1, -2, 0, 0;
    2, -1, 1, 0;
    0, 0, -1, -3;
    0, 0, 3, -1];

Y = [1; 1; 1; 1];
cvx_begin sdp
    variable Q(4, 4);
    G3 * Q - Q * A == Y * C;
cvx_end
L3 = Q \ Y;

fprintf("L2 matrix: \n");
print_matrix(L3, 2);

print_matrix(A + L3 * C, 5);

disp(eig(A + L3 * C));


%% 

L = L1;
res = sim("task2", 15);
x_arr = res.x;
xhat_arr = res.hatx;
t_arr = res.tout;

% plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, x_arr(:, 2), "x_2"}, {t_arr, x_arr(:, 3), "x_3"}, {t_arr, x_arr(:, 4), "x_4"}}, "media/plots/task2_x_1.png", "t", "x", "");
% plotter({{t_arr, xhat_arr(:, 1), "xhat_1"}, {t_arr, xhat_arr(:, 2), "xhat_2"}, {t_arr, xhat_arr(:, 3), "xhat_3"}, {t_arr, xhat_arr(:, 4), "xhat_4"}}, "media/plots/task2_xhat_1.png", "t", "xhat", "");
plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, xhat_arr(:, 1), "xhat_1"}}, "media/plots/task2_x1_1.png", "t", "x", "");
plotter({{t_arr, x_arr(:, 2), "x_2"}, {t_arr, xhat_arr(:, 2), "xhat_2"}}, "media/plots/task2_x2_1.png", "t", "x", "");
plotter({{t_arr, x_arr(:, 3), "x_3"}, {t_arr, xhat_arr(:, 3), "xhat_3"}}, "media/plots/task2_x3_1.png", "t", "x", "");
diffx = x_arr - xhat_arr;
plotter({{t_arr, diffx(:, 1), "diffx_1"}, {t_arr, diffx(:, 2), "diffx_2"}, {t_arr, diffx(:, 3), "diffx_3"}, {t_arr, diffx(:, 4), "diffx_4"}}, "media/plots/task2_diffx_1.png", "t", "diffx", "");

%%
L = L2;
res = sim("task2", 1);
x_arr = res.x;
xhat_arr = res.hatx;
t_arr = res.tout;

% plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, x_arr(:, 2), "x_2"}, {t_arr, x_arr(:, 3), "x_3"}, {t_arr, x_arr(:, 4), "x_4"}}, "media/plots/task2_x_2.png", "t", "x", "");
% plotter({{t_arr, xhat_arr(:, 1), "xhat_1"}, {t_arr, xhat_arr(:, 2), "xhat_2"}, {t_arr, xhat_arr(:, 3), "xhat_3"}, {t_arr, xhat_arr(:, 4), "xhat_4"}}, "media/plots/task2_xhat_2.png", "t", "xhat", "");
plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, xhat_arr(:, 1), "xhat_1"}}, "media/plots/task2_x1_2.png", "t", "x", "");
plotter({{t_arr, x_arr(:, 2), "x_2"}, {t_arr, xhat_arr(:, 2), "xhat_2"}}, "media/plots/task2_x2_2.png", "t", "x", "");
plotter({{t_arr, x_arr(:, 3), "x_3"}, {t_arr, xhat_arr(:, 3), "xhat_3"}}, "media/plots/task2_x3_2.png", "t", "x", "");
diffx = x_arr - xhat_arr
plotter({{t_arr, diffx(:, 1), "diffx_1"}, {t_arr, diffx(:, 2), "diffx_2"}, {t_arr, diffx(:, 3), "diffx_3"}, {t_arr, diffx(:, 4), "diffx_4"}}, "media/plots/task2_diffx_2.png", "t", "diffx", "");

%%
L = L3;
res = sim("task2", 10);
x_arr = res.x;
xhat_arr = res.hatx;
t_arr = res.tout;

% plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, x_arr(:, 2), "x_2"}, {t_arr, x_arr(:, 3), "x_3"}, {t_arr, x_arr(:, 4), "x_4"}}, "media/plots/task2_x_3.png", "t", "x", "");
% plotter({{t_arr, xhat_arr(:, 1), "xhat_1"}, {t_arr, xhat_arr(:, 2), "xhat_2"}, {t_arr, xhat_arr(:, 3), "xhat_3"}, {t_arr, xhat_arr(:, 4), "xhat_4"}}, "media/plots/task2_xhat_3.png", "t", "xhat", "");
plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, xhat_arr(:, 1), "xhat_1"}}, "media/plots/task2_x1_3.png", "t", "x", "");
plotter({{t_arr, x_arr(:, 2), "x_2"}, {t_arr, xhat_arr(:, 2), "xhat_2"}}, "media/plots/task2_x2_3.png", "t", "x", "");
plotter({{t_arr, x_arr(:, 3), "x_3"}, {t_arr, xhat_arr(:, 3), "xhat_3"}}, "media/plots/task2_x3_3.png", "t", "x", "");
diffx = x_arr - xhat_arr;
plotter({{t_arr, diffx(:, 1), "diffx_1"}, {t_arr, diffx(:, 2), "diffx_2"}, {t_arr, diffx(:, 3), "diffx_3"}, {t_arr, diffx(:, 4), "diffx_4"}}, "media/plots/task2_diffx_3.png", "t", "diffx", "");


%% TASK 3 

A = [6, 0, -12, 6; 0, 6, -6, 12; -12, -6, 6, 0; 6, 12, 0, 6];
B = [6; 12; 6; 4];
C = [-6, 6, 6, 6; 3, 0, 0, 3];
D = [2; 2];

disp(rank(get_control_matrix(A, B)));
disp(rank(get_observability_matrix(A, C)));

[Aj, P] = get_jordan_form(A);
fprintf("Jordan form of A: \n");
print_matrix(Aj, 2);
fprintf("P matrix: \n");
print_matrix(P, 2);
Bj = inv(P) * B;
fprintf("Bj matrix: \n");
print_matrix(Bj, 2);
Cj = C * P;
fprintf("Cj matrix: \n");
print_matrix(Cj, 2);

%%
K = -acker(A, B, [-4, -1, -2, -3]);
fprintf("K matrix: \n");
print_matrix(K, 2);

disp(eig(A + B * K));

%% 

G = [-1, 0, 0, 0;
    0, -2, 0, 0;
    0, 0, -3, 0;
    0, 0, 0, -12];

Y = [1, 1; 1, 1; 1, 1; 1, 1];
cvx_begin sdp
    variable Q(4, 4);
    G * Q - Q * A == Y * C;
cvx_end
L = Q \ Y;

fprintf("L matrix: \n");
print_matrix(L, 2);

print_matrix(A + L * C, 5);

disp(eig(A + L * C));

%% 
res = sim("task3", 15);
x = res.x;
xhat = res.hatx;
t = res.tout;
u = res.u;

plotter({{t, x(:, 1), "x_1"}, {t, x(:, 2), "x_2"}, {t, x(:, 3), "x_3"}, {t, x(:, 4), "x_4"}}, "media/plots/task3_x_1.png", "t", "x", "");
plotter({{t, xhat(:, 1), "xhat_1"}, {t, xhat(:, 2), "xhat_2"}, {t, xhat(:, 3), "xhat_3"}, {t, xhat(:, 4), "xhat_4"}}, "media/plots/task3_xhat_1.png", "t", "xhat", "");
diffx = x - xhat;
plotter({{t, diffx(:, 1), "diffx_1"}, {t, diffx(:, 2), "diffx_2"}, {t, diffx(:, 3), "diffx_3"}, {t, diffx(:, 4), "diffx_4"}}, "media/plots/task3_diffx_1.png", "t", "diffx", "");
plotter({{t, u(:, 1), "u"}}, "media/plots/task3_u_1.png", "t", "u", "");


%% TASK 4 
disp(rank(get_control_matrix(A, B)));
disp(rank(get_observability_matrix(A, C)));

[Aj, P] = get_jordan_form(A);
fprintf("Jordan form of A: \n");
print_matrix(Aj, 2);
fprintf("P matrix: \n");
print_matrix(P, 2);
Bj = inv(P) * B;
fprintf("Bj matrix: \n");
print_matrix(Bj, 2);
Cj = C * P;
fprintf("Cj matrix: \n");
print_matrix(Cj, 2);

%%
K = -acker(A, B, [-15, -14, -10, -2]);
fprintf("K matrix: \n");
print_matrix(K, 2);

disp(eig(A + B * K));


%% 
Y = [1, 0;
    1, 0];
G = [-3, 0;
    0, -6];

cvx_begin sdp
    variable Q(2, 4);
    G * Q - Q * A == Y * C;
cvx_end

% 2x2 * 2x4 - 2x4 * 4x4 = 2x2 * 2x4 

fprintf("Q matrix: \n");
print_matrix(Q, 2);

CQi = inv([C; Q]);
print_matrix(CQi, 2);

%% 
res = sim("task4", 5);
x_arr = res.x;
xhat_arr = res.xhat;
t_arr = res.tout;
zhat_arr = res.zhat;
u_arr = res.u;

xdiff = x_arr - xhat_arr;
plotter({{t_arr, xdiff(:, 1), "xdiff_1"}, {t_arr, xdiff(:, 2), "xdiff_2"}, {t_arr, xdiff(:, 3), "xdiff_3"}, {t_arr, xdiff(:, 4), "xdiff_4"}}, "media/plots/task4_xdiff_1.png", "t", "xdiff", "");
plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, xhat_arr(:, 1), "xhat_1"}}, "media/plots/task4_x1_1.png", "t", "x", "");
plotter({{t_arr, x_arr(:, 2), "x_2"}, {t_arr, xhat_arr(:, 2), "xhat_2"}}, "media/plots/task4_x2_1.png", "t", "x", "");
plotter({{t_arr, x_arr(:, 3), "x_3"}, {t_arr, xhat_arr(:, 3), "xhat_3"}}, "media/plots/task4_x3_1.png", "t", "x", "");
plotter({{t_arr, x_arr(:, 4), "x_4"}, {t_arr, xhat_arr(:, 4), "xhat_4"}}, "media/plots/task4_x4_1.png", "t", "x", "");
plotter({{t_arr, zhat_arr(:, 1), "zhat_1"}, {t_arr, zhat_arr(:, 2), "zhat_2"}}, "media/plots/task4_zhat_1.png", "t", "zhat", "");
plotter({{t_arr, u_arr(:, 1), "u"}}, "media/plots/task4_u_1.png", "t", "u", "");