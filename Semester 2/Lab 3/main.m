cvx_setup;
cvx_quiet true;  


function K = FindControllerLyap(A, B, alpha)
    [Am, An] = size(A);
    [Bm, Bn] = size(B);

    cvx_begin sdp
        variable P(Am, An)
        variable Y(Bn, Bm)
        P > 0.0001 * eye(Am);
        P * A' + A * P + 2 * alpha * P + Y' * B' + B * Y <= 0;
    cvx_end

    K = Y / P;
end

function [K, mu] = FindControllerLyapMin(A, B, x0, alpha) 
    [Am, An] = size(A);
    [Bm, Bn] = size(B);

    cvx_begin sdp
        variable P(Am, An)
        variable Y(Bn, Bm)
        variable mumu;
        minimize mumu;
        P > 0.0001 * eye(Am);
        P * A' + A * P + 2 * alpha * P + Y' * B' + B * Y <= 0;
        [P x0;
         x0' 1] > 0;
        [P Y';
         Y mumu * eye(Bn) ] > 0;
    cvx_end

    K = Y / P;
    mu = sqrt(mumu);
end

function [L, mu] = FindObserverMin(A, C, e0, alpha) 
    [Am, An] = size(A);
    [Cm, Cn] = size(C);

    cvx_begin sdp
        variable Q(Am, An)
        variable Y(Cn, Cm)
        variable mumu;
        minimize mumu;
        Q > 0.0001 * eye(Am);
        A' * Q + Q * A + 2 * alpha * Q + C' * Y' + Y * C <= 0;        
        [Q e0;
         e0' 1] > 0;
        [Q Y;
         Y' mumu * eye(Cm)] > 0;
    cvx_end

    L = Q \ Y;
    mu = sqrt(mumu);
end

function K = RegulExp(A, B, V, beta, r, Q, R)    
    syms P_ [2 2]
    K_ = -(inv(R + B' * P_ * B) * B' * P_ * (A - beta * eye(2)));
    eqs = (A + B * K_ - beta * eye(2))' * P_ * (A + B * K_ - beta * eye(2)) - r^2 * P_ == -Q;

    s = vpasolve(eqs, [P_], Random=true);
    P_ = [s.P_1_1 s.P_1_2;
          s.P_2_1 s.P_2_2];
    K = [0 -(inv(R + B' * P_ * B) * B' * P_ * (A - beta * eye(2)))] * V^-1;

    disp('K = ');
    disp(K);
end


function PlotEigenvalues(eigenvalues, beta, r, path)
    fig = figure('Position', [10 10 600 600]);
    set(gca, 'LooseInset', get(gca, 'TightInset') * 2.0);
    hold on;

    % equal axis
    axis equal;

    real_parts = real(eigenvalues);
    imag_parts = imag(eigenvalues);

    scatter(real_parts, imag_parts, 100, 'filled');

    theta = linspace(0, 2*pi, 100);
    x_circle = beta + r * cos(theta);
    y_circle = r * sin(theta);
    plot(x_circle, y_circle, '-.', 'LineWidth', 2, "MarkerSize", 2);

    xlabel('Re');
    ylabel('Im');

    leg = legend('Eig', 'r circle');

    fontsize(leg, 18, 'points');
    fontsize(gca, 14, 'points');

    saveas(fig, path);
    close(fig); 
end


%% TASK 1 

A = [8, 1, 11; 4, 0, 4; -4, -3, -7];
B = [-1; -3; 3];

%% 
alpha1 = 3;
K1 = FindControllerLyap(A, B, alpha1);
fprintf('K1 = \n');
print_matrix(K1, 2);

% check if eigenvalues correct 
fprintf('Eigenvalues of A + B * K1 = \n');
e = eig(A + B * K1);
print_matrix(e, 2);

K = K1;
res = sim("task1", 1.4);
x_arr = res.x;
u_arr = res.u;
t_arr = res.tout;

plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, x_arr(:, 2), "x_2"}, {t_arr, x_arr(:, 3), "x_3"}}, "media/plots/task1_1_x.png", "t", "x", "");
plotter({{t_arr, u_arr(:, 1), "u"}}, "media/plots/task1_1_u.png", "t", "u", "");

%% 

alpha2 = 1;
K2 = FindControllerLyap(A, B, alpha2);
fprintf('K2 = \n');
print_matrix(K2, 2);

% check if eigenvalues correct 
fprintf('Eigenvalues of A + B * K2 = \n');
e = eig(A + B * K2);
print_matrix(e, 2);

K = K2;
res = sim("task1", 1.4);
x_arr = res.x;
u_arr = res.u;
t_arr = res.tout;

plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, x_arr(:, 2), "x_2"}, {t_arr, x_arr(:, 3), "x_3"}}, "media/plots/task1_2_x.png", "t", "x", "");
plotter({{t_arr, u_arr(:, 1), "u"}}, "media/plots/task1_2_u.png", "t", "u", "");

%% 

x0 = [1; 1; 1];
[H1, mu1] = FindControllerLyapMin(A, B, x0, alpha1);
fprintf('H1 = \n');
print_matrix(H1, 2);
fprintf('mu1 = %f\n', mu1);

% check if eigenvalues correct
fprintf('Eigenvalues of A + B * H1 = \n');
e = eig(A + B * H1);
print_matrix(e, 2);

K = H1;
res = sim("task1", 2);
x_arr = res.x;
u_arr = res.u;
t_arr = res.tout;

plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, x_arr(:, 2), "x_2"}, {t_arr, x_arr(:, 3), "x_3"}}, "media/plots/task1_3_x.png", "t", "x", "");
plotter({{t_arr, u_arr(:, 1), "u"}}, "media/plots/task1_3_u.png", "t", "u", "");

%% 

[H2, mu2] = FindControllerLyapMin(A, B, x0, alpha2);
fprintf('K4 = \n');
print_matrix(H2, 2);
fprintf('mu2 = %f\n', mu2);

% check if eigenvalues correct
fprintf('Eigenvalues of A + B * H2 = \n');
e = eig(A + B * H2);
print_matrix(e, 2);

K = H2;
res = sim("task1", 7);
x_arr = res.x;
u_arr = res.u;
t_arr = res.tout;

plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, x_arr(:, 2), "x_2"}, {t_arr, x_arr(:, 3), "x_3"}}, "media/plots/task1_4_x.png", "t", "x", "");
plotter({{t_arr, u_arr(:, 1), "u"}}, "media/plots/task1_4_u.png", "t", "u", "");


%% 
nu = 2;
R = 1;

alphas = [3, 1];

for alpha = alphas
    Q = eye(3);

    Aa = A + eye(3) * alpha;
    [P, K, e] = icare(Aa, sqrt(nu) * B, Q, R);
    K3 = -inv(R) * B' * P;
    
end

%% TASK 2 

A = [6, 0, -12, 6; 0, 6, -6, 12; -12, -6, 6, 0; 6, 12, 0, 6];
B = [6; 12; 6; 4];
C = [-6, 6, 6, 6; 3, 0, 0, 3];
x0 = [1; 1; 1; 1];

alpha1 = 12;
alpha2 = 1; 

K1 = FindControllerLyapMin(A, B, x0, alpha1);
fprintf('K1 = \n');
print_matrix(K1, 2);
% check if eigenvalues correct
fprintf('Eigenvalues of A + B * K1 = \n');
e = eig(A + B * K1);
print_matrix(e, 2);

K2 = FindControllerLyapMin(A, B, x0, alpha2);
fprintf('K2 = \n');
print_matrix(K2, 2);
% check if eigenvalues correct
fprintf('Eigenvalues of A + B * K2 = \n');
e = eig(A + B * K2);
print_matrix(e, 2);


%% 
e0 = [0; 0; 0; 0];
L1 = FindObserverMin(A, C, e0, alpha1);
fprintf('L1 = \n');
print_matrix(L1, 2);
% check if eigenvalues correct
fprintf('Eigenvalues of A + L1 * C = \n');
e = eig(A + L1 * C);
print_matrix(e, 2);

L2 = FindObserverMin(A, C, e0, alpha2);
fprintf('L2 = \n');
print_matrix(L2, 2);
% check if eigenvalues correct
fprintf('Eigenvalues of A + L2 * C = \n');
e = eig(A + L2 * C);
print_matrix(e, 2);

%%

K = K1;
L = L1;
res = sim("task2", 2);
x_arr = res.x;
xhat_arr = res.xhat;
u_arr = res.u;
t_arr = res.tout;

plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, x_arr(:, 2), "x_2"}, {t_arr, x_arr(:, 3), "x_3"}, {t_arr, x_arr(:, 4), "x_4"}}, "media/plots/task2_1_x.png", "t", "x", "");
plotter({{t_arr, u_arr(:, 1), "u"}}, "media/plots/task2_1_u.png", "t", "u", "");
err = x_arr - xhat_arr;
plotter({{t_arr, err(:, 1), "e_1"}, {t_arr, err(:, 2), "e_2"}, {t_arr, err(:, 3), "e_3"}, {t_arr, err(:, 4), "e_4"}}, "media/plots/task2_1_e.png", "t", "e", "");
plotter({{t_arr, x_arr(:, 1), "x_1", "style", "-", "color", "#0072BD"}, {t_arr, xhat_arr(:, 1), "hatx_1", "style", "--", "color", "#0072BD"}, {t_arr, x_arr(:, 2), "x_2", "style", "-", "color", "#D95319"}, {t_arr, xhat_arr(:, 2), "hatx_2", "style", "--", "color", "#D95319"}, {t_arr, x_arr(:, 3), "x_3", "style", "-", "color", "#EDB120"}, {t_arr, xhat_arr(:, 3), "hatx_3", "style", "--", "color", "#EDB120"}, {t_arr, x_arr(:, 4), "x_4", "style", "-", "color", "#7E2F8E"}, {t_arr, xhat_arr(:, 4), "hatx_4", "style", "--", "color", "#7E2F8E"}}, "media/plots/task2_1_xh.png", "t", "", "");
K = K1;
L = L2;
res = sim("task2", 2);
x_arr = res.x;
xh_arr = res.xhat;
u_arr = res.u;
t_arr = res.tout;

plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, x_arr(:, 2), "x_2"}, {t_arr, x_arr(:, 3), "x_3"}, {t_arr, x_arr(:, 4), "x_4"}}, "media/plots/task2_2_x.png", "t", "x", "");
plotter({{t_arr, u_arr(:, 1), "u"}}, "media/plots/task2_2_u.png", "t", "u", "");
err = x_arr - xh_arr;
plotter({{t_arr, err(:, 1), "e_1"}, {t_arr, err(:, 2), "e_2"}, {t_arr, err(:, 3), "e_3"}, {t_arr, err(:, 4), "e_4"}}, "media/plots/task2_2_e.png", "t", "e", "");
plotter({{t_arr, x_arr(:, 1), "x_1", "style", "-", "color", "#0072BD"}, {t_arr, xh_arr(:, 1), "hatx_1", "style", "--", "color", "#0072BD"}, {t_arr, x_arr(:, 2), "x_2", "style", "-", "color", "#D95319"}, {t_arr, xh_arr(:, 2), "hatx_2", "style", "--", "color", "#D95319"}, {t_arr, x_arr(:, 3), "x_3", "style", "-", "color", "#EDB120"}, {t_arr, xh_arr(:, 3), "hatx_3", "style", "--", "color", "#EDB120"}, {t_arr, x_arr(:, 4), "x_4", "style", "-", "color", "#7E2F8E"}, {t_arr, xh_arr(:, 4), "hatx_4", "style", "--", "color", "#7E2F8E"}}, "media/plots/task2_2_xh.png", "t", "", "");

K = K2;
L = L1;
res = sim("task2", 2);
x_arr = res.x;
xhat_arr = res.xhat;
u_arr = res.u;
t_arr = res.tout;

plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, x_arr(:, 2), "x_2"}, {t_arr, x_arr(:, 3), "x_3"}, {t_arr, x_arr(:, 4), "x_4"}}, "media/plots/task2_3_x.png", "t", "x", "");
plotter({{t_arr, u_arr(:, 1), "u"}}, "media/plots/task2_3_u.png", "t", "u", "");
err = x_arr - xhat_arr;
plotter({{t_arr, err(:, 1), "e_1"}, {t_arr, err(:, 2), "e_2"}, {t_arr, err(:, 3), "e_3"}, {t_arr, err(:, 4), "e_4"}}, "media/plots/task2_3_e.png", "t", "e", "");
plotter({{t_arr, x_arr(:, 1), "x_1", "style", "-", "color", "#0072BD"}, {t_arr, xhat_arr(:, 1), "hatx_1", "style", "--", "color", "#0072BD"}, {t_arr, x_arr(:, 2), "x_2", "style", "-", "color", "#D95319"}, {t_arr, xhat_arr(:, 2), "hatx_2", "style", "--", "color", "#D95319"}, {t_arr, x_arr(:, 3), "x_3", "style", "-", "color", "#EDB120"}, {t_arr, xhat_arr(:, 3), "hatx_3", "style", "--", "color", "#EDB120"}, {t_arr, x_arr(:, 4), "x_4", "style", "-", "color", "#7E2F8E"}, {t_arr, xhat_arr(:, 4), "hatx_4", "style", "--", "color", "#7E2F8E"}}, "media/plots/task2_3_xh.png", "t", "", "");

%% 

K = K2;
L = L2;
res = sim("task2", 5);
x_arr = res.x;
xhat_arr = res.xhat;
u_arr = res.u;
t_arr = res.tout;

plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, x_arr(:, 2), "x_2"}, {t_arr, x_arr(:, 3), "x_3"}, {t_arr, x_arr(:, 4), "x_4"}}, "media/plots/task2_4_x.png", "t", "x", "");
plotter({{t_arr, u_arr(:, 1), "u"}}, "media/plots/task2_4_u.png", "t", "u", "");
err = x_arr - xhat_arr;
plotter({{t_arr, err(:, 1), "e_1"}, {t_arr, err(:, 2), "e_2"}, {t_arr, err(:, 3), "e_3"}, {t_arr, err(:, 4), "e_4"}}, "media/plots/task2_4_e.png", "t", "e", "");
plotter({{t_arr, x_arr(:, 1), "x_1", "style", "-", "color", "#0072BD"}, {t_arr, xhat_arr(:, 1), "hatx_1", "style", "--", "color", "#0072BD"}, {t_arr, x_arr(:, 2), "x_2", "style", "-", "color", "#D95319"}, {t_arr, xhat_arr(:, 2), "hatx_2", "style", "--", "color", "#D95319"}, {t_arr, x_arr(:, 3), "x_3", "style", "-", "color", "#EDB120"}, {t_arr, xhat_arr(:, 3), "hatx_3", "style", "--", "color", "#EDB120"}, {t_arr, x_arr(:, 4), "x_4", "style", "-", "color", "#7E2F8E"}, {t_arr, xhat_arr(:, 4), "hatx_4", "style", "--", "color", "#7E2F8E"}}, "media/plots/task2_4_xh.png", "t", "", "");


%% 
A = [8, 1, 11; 4, 0, 4; -4, -3, -7];
B = [-1; -3; 3];
x0 = [1; 1; 1];

[V, J] = jordan(A);
[V, J] = cdf2rdf(V, J);

A_jord = J;
B_jord = V \ B;

A_usech = A_jord;
A_usech(1, :) = [];
A_usech(:, 1) = [];

B_usech = B_jord;
B_usech(1, :) = [];

beta = -3;
r = 2;

%% 
disp('Q = I. R = 1');

Q1 = eye(2);
R1 = 1;

eigs_1 = [1; 1; 1]; % заглушка

while any(eigs_1 >= 0)  
    K1 = double(RegulExp(A_usech, B_usech, V, beta, r, Q1, R1));
    eigs_1 = eig(A + B * K1);
end

disp('Found K');
K = K1;
print_matrix(K, 2)

% check if eigenvalues correct
fprintf('Eigenvalues of A + B * K1 = \n');
e = eig(A + B * K);
print_matrix(e, 2);

PlotEigenvalues(eigs_1, beta, r, 'media/plots/task3_eigs_1.png');

res = sim("task1", 5);
x_arr = res.x;
u_arr = res.u;
t_arr = res.tout;

plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, x_arr(:, 2), "x_2"}, {t_arr, x_arr(:, 3), "x_3"}}, "media/plots/task3_1_x.png", "t", "x", "");
plotter({{t_arr, u_arr(:, 1), "u"}}, "media/plots/task3_1_u.png", "t", "u", "");

%% 
disp('Q = I. R = 0');
Q2 = eye(2);
R2 = 0;

eigs_2 = [1; 1; 1]; % заглушка

while any(eigs_2 >= 0)  
    K2 = double(RegulExp(A_usech, B_usech, V, beta, r, Q2, R2));
    eigs_2 = eig(A + B * K2);
end

disp('Found K');
K = K2;
print_matrix(K, 2)

fprintf('Eigenvalues of A + B * K2 = \n');
e = eig(A + B * K);
print_matrix(e, 2);

PlotEigenvalues(eigs_2, beta, r, 'media/plots/task3_eigs_2.png');

res = sim("task1", 5);
x_arr = res.x;
u_arr = res.u;
t_arr = res.tout;

plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, x_arr(:, 2), "x_2"}, {t_arr, x_arr(:, 3), "x_3"}}, "media/plots/task3_2_x.png", "t", "x", "");
plotter({{t_arr, u_arr(:, 1), "u"}}, "media/plots/task3_2_u.png", "t", "u", "");

%% 
disp('Q = 0. R = 1');
Q3 = zeros(2);
R3 = 1;

eigs_3 = [1; 1; 1]; % заглушка

while any(eigs_3 >= 0)  
    K3 = double(RegulExp(A_usech, B_usech, V, beta, r, Q3, R3));
    eigs_3 = eig(A + B * K3);
end

disp('Found K');
K = K3;
print_matrix(K, 2)

fprintf('Eigenvalues of A + B * K3 = \n');
e = eig(A + B * K);
print_matrix(e, 2);

PlotEigenvalues(eigs_3, beta, r, 'media/plots/task3_eigs_3.png');

res = sim("task1", 5);
x_arr = res.x;
u_arr = res.u;
t_arr = res.tout;

plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, x_arr(:, 2), "x_2"}, {t_arr, x_arr(:, 3), "x_3"}}, "media/plots/task3_3_x.png", "t", "x", "");
plotter({{t_arr, u_arr(:, 1), "u"}}, "media/plots/task3_3_u.png", "t", "u", "");

%% 
disp('Q = 0. R = 0');

Q4 = zeros(2);
R4 = 0;


r = 2;

eigs_4 = [1; 1; 1]; % заглушка

while any(eigs_4 >= 0)  
    K4 = double(RegulExp(A_usech, B_usech, V, beta, r, Q4, R4));
    eigs_4 = eig(A + B * K4);
end

disp('Found K');
K = K4;
print_matrix(K, 2)

fprintf('Eigenvalues of A + B * K4 = \n');
e = eig(A + B * K);
print_matrix(e, 2);


PlotEigenvalues(eigs_4, beta, r, 'media/plots/task3_eigs_4.png');

res = sim("task1", 10);
x_arr = res.x;
u_arr = res.u;
t_arr = res.tout;

plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, x_arr(:, 2), "x_2"}, {t_arr, x_arr(:, 3), "x_3"}}, "media/plots/task3_4_x.png", "t", "x", "");
plotter({{t_arr, u_arr(:, 1), "u"}}, "media/plots/task3_4_u.png", "t", "u", "");

%% 

