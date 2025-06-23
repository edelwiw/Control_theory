%% 
cvx_setup;
cvx_quiet true;  


%% TASK 1 
A = [8, 1, 11; 4, 0, 4; -4, -3, -7];
B = [-1; -3; 3];
x0 = [1; 1; 1];

Q1 = 2 * eye(3);
R1 = 2; 

Q2 = 2 * eye(3);
R2 = 20;

Q3 = 20 * eye(3);
R3 = 2;

Q4 = 20 * eye(3);
R4 = 20;

[K1, J1] = TAU.FindLQRController(A, B, Q1, R1, x0); 
[K2, J2] = TAU.FindLQRController(A, B, Q2, R2, x0);
[K3, J3] = TAU.FindLQRController(A, B, Q3, R3, x0);
[K4, J4] = TAU.FindLQRController(A, B, Q4, R4, x0);

fprintf('K_1 = \n');
print_matrix(K1, 2);
fprintf('J_1 = %f\n', J1);
fprintf('K_2 = \n');
print_matrix(K2, 2);
fprintf('J_2 = %f\n', J2);
fprintf('K_3 = \n');
print_matrix(K3, 2);
fprintf('J_3 = %f\n', J3);
fprintf('K_4 = \n');
print_matrix(K4, 2);
fprintf('J_4 = %f\n', J4);

%% 

function [t, x, u, J] = test_controller_lqr(K, Q, R, x0, Jmin, time, path, n)
    if ~exist(path, "dir")
        mkdir(path);
    end
    simIn = Simulink.SimulationInput('lqr');
    simIn = simIn.setVariable('K', K);
    simIn = simIn.setVariable('Q', Q);
    simIn = simIn.setVariable('R', R);
    simIn = simIn.setVariable('x0', x0);
    res = sim(simIn.setModelParameter('StopTime', num2str(time)));

    t = res.tout; 
    x = res.x;
    u = res.u;
    J = res.J;

    plotter({{t, x(:, 1), "$x_1$"}, {t, x(:, 2), "$x_2$"}, {t, x(:, 3), "$x_3$"}}, sprintf("%s/state_%d.png", path, n), "t, s", "state", "");
    plotter({{t, u, "$u$"}}, sprintf("%s/u_%d.png", path, n), "t, s", "U", "");
    plotter({{t, J, "$J$"}, {t, Jmin * ones(size(t)), "$J_{min}$"}}, sprintf("%s/J_%d.png", path, n), "t, s", "J", "");
end


path = "media/plots/lqr_task1";
time = 3;
[t1, x1, u1, j1m] = test_controller_lqr(K1, Q1, R1, x0, J1, time, path, 1);
[t2, x2, u2, j2m] = test_controller_lqr(K2, Q2, R2, x0, J2, time, path, 2);
[t3, x3, u3, j3m] = test_controller_lqr(K3, Q3, R3, x0, J3, time, path, 3);
[t4, x4, u4, j4m] = test_controller_lqr(K4, Q4, R4, x0, J4, time, path, 4);

plotter({{t1, u1, "$u_1$"}, {t2, u2, "$u_2$"}, {t3, u3, "$u_3$"}, {t4, u4, "$u_4$"}}, sprintf("%s/u_cmp.png", path), "t, s", "U", "");
plotter({{t1, j1m, "$J_1$", "style", "-", "color", "#0072BD"}, ...
        {t2, j2m, "$J_2$", "style", "-", "color", "#D95319"}, ...
        {t3, j3m, "$J_3$", "style", "-", "color", "#EDB120"}, ...
        {t4, j4m, "$J_4$", "style", "-", "color", "#7E2F8E"}, ...
        {t1, J1 * ones(size(t1)), "$J_{1,min}$", "style", "-.", "color", "#0072BD"}, ...
        {t2, J2 * ones(size(t2)), "$J_{2,min}$", "style", "-.", "color", "#D95319"}, ... 
        {t3, J3 * ones(size(t3)), "$J_{3,min}$", "style", "-.", "color", "#EDB120"}, ...
        {t4, J4 * ones(size(t4)), "$J_{4,min}$", "style", "-.", "color", "#7E2F8E"}}, ...
        sprintf("%s/J_cmp.png", path), "t, s", "J", "");

plotter({{t1, x1(:, 1), "$x_{1,1}$"}, {t2, x2(:, 1), "$x_{2,1}$"}, {t3, x3(:, 1), "$x_{3,1}$"}, {t4, x4(:, 1), "$x_{4,1}$"}}, ...
        sprintf("%s/x1_cmp.png", path), "t, s", "x_1", "");
plotter({{t1, x1(:, 2), "$x_{1,2}$"}, {t2, x2(:, 2), "$x_{2,2}$"}, {t3, x3(:, 2), "$x_{3,2}$"}, {t4, x4(:, 2), "$x_{4,2}$"}}, ...
        sprintf("%s/x2_cmp.png", path), "t, s", "x_2", "");
plotter({{t1, x1(:, 3), "$x_{1,3}$"}, {t2, x2(:, 3), "$x_{2,3}$"}, {t3, x3(:, 3), "$x_{3,3}$"}, {t4, x4(:, 3), "$x_{4,3}$"}}, ...
        sprintf("%s/x3_cmp.png", path), "t, s", "x_3", ""); 



%% TASK 2 
A = [-40, 16, 9, 7; -64, 25, 14 ,12; -26, 11, 7, 3; -48, 18, 14, 8];
C = [-3, 2, -2, 1]; 
varf = 1;
vare = 0.5;
x0 = [1; 1; 1; 1];
hatx0 = [0; 0; 0; 0];

Q1 = 2 * eye(4);
R1 = 2; 

Q2 = 2 * eye(4);
R2 = 200;

Q3 = 200 * eye(4);
R3 = 2;

Q4 = 200 * eye(4);
R4 = 200;

Q5 = varf * eye(4);
R5 = vare; 

L1 = TAU.FindObserverKalman(A, C, Q1, R1);
L2 = TAU.FindObserverKalman(A, C, Q2, R2);
L3 = TAU.FindObserverKalman(A, C, Q3, R3);
L4 = TAU.FindObserverKalman(A, C, Q4, R4);
L5 = TAU.FindObserverKalman(A, C, Q5, R5);

fprintf('L_1 = \n');
print_matrix(L1, 2);
fprintf('L_2 = \n');
print_matrix(L2, 2);
fprintf('L_3 = \n');
print_matrix(L3, 2);
fprintf('L_4 = \n');
print_matrix(L4, 2);
fprintf('L_5 = \n');
print_matrix(L5, 2);

%%
function [t, x, xhat, y, yf] = test_kalman_filter(L, verf, vare, time, x0, path, n)
    if ~exist(path, "dir")
        mkdir(path);
    end
    simIn = Simulink.SimulationInput('kalman');
    simIn = simIn.setVariable('L', L);
    simIn = simIn.setVariable('x0', x0);
    simIn = simIn.setVariable('varf', verf);
    simIn = simIn.setVariable('vare', vare);
    res = sim(simIn.setModelParameter('StopTime', num2str(time)));

    t = res.tout; 
    x = res.x;
    xhat = res.xhat;
    y = res.y;
    yf = res.yf;

    plotter({{t, x(:, 1), "$x_1$"}, {t, x(:, 2), "$x_2$"}, {t, x(:, 3), "$x_3$"}, {t, x(:, 4), "$x_4$"}}, sprintf("%s/state_%d.png", path, n), "t, s", "state", "");
    plotter({{t, xhat(:, 1), "$\hat{x}_1$"}, {t, xhat(:, 2), "$\hat{x}_2$"}, {t, xhat(:, 3), "$\hat{x}_3$"}, {t, xhat(:, 4), "$\hat{x}_4$"}}, sprintf("%s/est_state_%d.png", path, n), "t, s", "state", "");
    plotter({{t, x(:, 1), "$x_1$", "style", "-", "color", "#0072BD"}, ... 
            {t, xhat(:, 1), "$\hat{x}_1$", "style", "--", "color", "#0072BD"}, ... 
            {t, x(:, 2), "$x_2$", "style", "-", "color", "#D95319"}, ...
            {t, xhat(:, 2), "$\hat{x}_2$", "style", "--", "color", "#D95319"}, ...
            {t, x(:, 3), "$x_3$", "style", "-", "color", "#EDB120"}, ...
            {t, xhat(:, 3), "$\hat{x}_3$", "style", "--", "color", "#EDB120"}, ...
            {t, x(:, 4), "$x_4$", "style", "-", "color", "#7E2F8E"}, ...
            {t, xhat(:, 4), "$\hat{x}_4$", "style", "--", "color", "#7E2F8E"}}, ...
            sprintf("%s/state_cmp_%d.png", path, n), "t, s", "state", "");
    err = x - xhat;
    plotter({{t, err(:, 1), "$e_1$"}, {t, err(:, 2), "$e_2$"}, {t, err(:, 3), "$e_3$"}, {t, err(:, 4), "$e_4$"}}, sprintf("%s/err_%d.png", path, n), "t, s", "error", "");

    plotter({{t, y, "$y$"}}, sprintf("%s/y_%d.png", path, n), "t, s", "y", "");
    plotter({{t, yf, "$y_f$"}}, sprintf("%s/yf_%d.png", path, n), "t, s", "y_f", "");
    plotter({{t, yf, "$y_f$"}, {t, y, "$y$"}}, sprintf("%s/y_cmp_%d.png", path, n), "t, s", "y", "");
end

path = "media/plots/kalman_task2";
time = 30;
[t1, x1, xhat1, y1, yf1] = test_kalman_filter(L1, varf, vare, time, x0, path, 1);
[t2, x2, xhat2, y2, yf2] = test_kalman_filter(L2, varf, vare, time, x0, path, 2);
[t3, x3, xhat3, y3, yf3] = test_kalman_filter(L3, varf, vare, time, x0, path, 3);
[t4, x4, xhat4, y4, yf4] = test_kalman_filter(L4, varf, vare, time, x0, path, 4);
[t5, x5, xhat5, y5, yf5] = test_kalman_filter(L5, varf, vare, time, x0, path, 5);   

%% TASK 3

function [A, P] = get_jordan_form(A) 
    [Pj, Aj] = jordan(A);
    [P, A] = cdf2rdf(Pj, Aj);
end

A = [4, -2, 0, 6; -2, 4, -6, 0; 0, -6, 4, 2; 6, 0, 2, 4];
B = [11, 0; -1, 0; 7, 0; 9, 0];
C = [1, -1, 1, 1; 1, 3, -1, 3];
D = [0, 3; 0, 4];


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

f = @(t) [3*sin(2*t); 2*cos(3*t); sin(3*t); cos(1*t)];
e = @(t) [3 * cos(6 * t); sin(2 * t)];

x0 = [1; 1; 1; 1];

QK = 30 * eye(4);
RK = 20; 

QL = 60 * eye(4);
RL = 2;

[K, J] = TAU.FindLQRController(A, B, QK, RK, x0);
L = TAU.FindObserverKalman(A, C, QL, RL);

fprintf('K = \n');
print_matrix(K, 2);
fprintf('J = %f\n', J);
fprintf('L = \n');
print_matrix(L, 2);

%% 
function [t, x, xhat, u, y] = test_lqr(K, L, x0, time, path, n)
    if ~exist(path, "dir")
        mkdir(path);
    end
    simIn = Simulink.SimulationInput('lqg');
    simIn = simIn.setVariable('K', K);
    simIn = simIn.setVariable('L', L);
    simIn = simIn.setVariable('x0', x0);
    res = sim(simIn.setModelParameter('StopTime', num2str(time)));

    t = res.tout; 
    x = res.x;
    xhat = res.xhat;
    y = res.y;
    u = res.u;

    plotter({{t, x(:, 1), "$x_1$"}, {t, x(:, 2), "$x_2$"}, {t, x(:, 3), "$x_3$"}, {t, x(:, 4), "$x_4$"}}, sprintf("%s/state_%d.png", path, n), "t, s", "state", "");
    plotter({{t, xhat(:, 1), "$\hat{x}_1$"}, {t, xhat(:, 2), "$\hat{x}_2$"}, {t, xhat(:, 3), "$\hat{x}_3$"}, {t, xhat(:, 4), "$\hat{x}_4$"}}, sprintf("%s/est_state_%d.png", path, n), "t, s", "state", "");
    plotter({{t, x(:, 1), "$x_1$", "style", "-", "color", "#0072BD"}, ... 
            {t, xhat(:, 1), "$\hat{x}_1$", "style", "--", "color", "#0072BD"}, ... 
            {t, x(:, 2), "$x_2$", "style", "-", "color", "#D95319"}, ...
            {t, xhat(:, 2), "$\hat{x}_2$", "style", "--", "color", "#D95319"}, ...
            {t, x(:, 3), "$x_3$", "style", "-", "color", "#EDB120"}, ...
            {t, xhat(:, 3), "$\hat{x}_3$", "style", "--", "color", "#EDB120"}, ...
            {t, x(:, 4), "$x_4$", "style", "-", "color", "#7E2F8E"}, ...
            {t, xhat(:, 4), "$\hat{x}_4$", "style", "--", "color", "#7E2F8E"}}, ...
            sprintf("%s/state_cmp_%d.png", path, n), "t, s", "state", "");
    err = x - xhat;
    plotter({{t, err(:, 1), "$e_1$"}, {t, err(:, 2), "$e_2$"}, {t, err(:, 3), "$e_3$"}, {t, err(:, 4), "$e_4$"}}, sprintf("%s/err_%d.png", path, n), "t, s", "error", "");

    plotter({{t, u(:, 1), "$u_1$"}, {t, u(:, 2), "$u_2$"}}, sprintf("%s/u_%d.png", path, n), "t, s", "U", "");
    plotter({{t, y(:, 1), "$y_1$"}, {t, y(:, 2), "$y_2$"}}, sprintf("%s/y_%d.png", path, n), "t, s", "y", "");
end 


time = 15;
[t, x, xhat, u, y] = test_lqr(K, L, x0, time, "media/plots/lqr_task3", 1);
