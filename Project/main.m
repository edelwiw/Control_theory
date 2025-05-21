%% 
cvx_setup;
cvx_quiet true;  
%% 

rng(30, "philox");
M = randi([100000 1000000]) / 1000 / sqrt(2);
m = randi([1000 10000]) / 1000 * sqrt(3);
l = randi([100 1000]) / 100 / sqrt(5);
l = l / 2;
g = 9.81;   

% (m * l * l * (-m * l * power(u[4], 2) * sin(u[3]) + u[1]) + m * l * cos(u[3]) * (m * g * l * sin(u[3]) + u[2])) / (m * l * l * (Mcart + m - m * power(cos(u[3]), 2)))
% ((m * g * l * sin(u[3]) + u[2]) * (Mcart + m) + m * l * cos(u[3]) * (-m * l * power(u[4], 2) * sin(u[3]) + u[1])) / (m * l * l * (Mcart + m - m * power(cos(u[3]), 2)))

function [A, B, D, C] = get_matrices(M, m, l, g)
    A = [0, 1, 0, 0;
        0, 0, m * g / M, 0;
        0, 0, 0, 1;
        0, 0, g * (M + m) / (l * M), 0];
    B = [0; 1 / M; 0; 1 / (l * M)];
    D = [0; 1/(l * M); 0; (M + m) / (m * l * l * M)];
    C = [1, 0, 0, 0; 0, 0, 1, 0];
end

function print_system(M, m, l, g, A, B, C, D)
    fprintf("M = %.3f, m = %.3f, l = %.3f\n", M, m, l);
    fprintf("A = ");
    print_matrix(A, 2);
    fprintf("B = ");
    print_matrix(B, 4);
    fprintf("D = ");
    print_matrix(D, 3);
    fprintf("C = ");
    print_matrix(C, 0);
end

function U = get_controllability_matrix(A, B) 
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
end

function K = FindControllerSylvester(A, B, Gamma)
    [Am, An] = size(A);
    [Bm, Bn] = size(B);
    Y = [1, 1, 1, 1];
    cvx_begin sdp 
        variable P(Am, Am);
        A * P - P * Gamma == B * Y;
    cvx_end
    K = -Y * inv(P);
end


[A, B, D, C] = get_matrices(M, m, l, g);

%% Print system matrices
print_system(M, m, l, g, A, B, C, D);
% find eigenvectors 
[eivA, eigA] = eig(A);
fprintf("Eigenvectors of A = ");
print_matrix(eivA, 2);
eigA = eig(A);
fprintf("Eigenvalues of A = ");
print_matrix(eigA, 2);

%% Controllability and Observability
U = get_controllability_matrix(A, B);
fprintf("Controllability matrix U = ");
print_matrix(U, 4);
W = get_observability_matrix(A, C);
fprintf("Observability matrix W = ");
print_matrix(W, 4);
fprintf("Rank of U = %d\n", rank(U));
fprintf("Rank of W = %d\n", rank(W));

%% Transfer matrix
sys_u = ss(A, B, C, 0);
sys_f = ss(A, D, C, 0);

Wuy = tf(sys_u);
Wfy = tf(sys_f);

fprintf("W_{u \\rightarrow y}(s) = ")
tf2latex(Wuy);
fprintf("W_{f \\rightarrow y}(s) = ")
tf2latex(Wfy);

%% Simulation of free motion
theta0_arr = [0, 0.1, -0.1, 0.3, pi/2, pi];
path = "Report/media/plots/free_motion";
if ~exist(path, "dir")
    mkdir(path);
end
for i = 1:length(theta0_arr)
    theta0 = theta0_arr(i);
    res = sim("free_motion.slx", 0.4);
    t = res.tout;
    x = res.x;
    ang = res.ang;
    xlin = res.xlin;
    anglin = res.anglin;

    % pos + ang 
    plotter({{t, x, "cart pos"}, {t, ang, "angle"}}, sprintf("%s/nonlin_%d.png", path, i), "t (s)", "position (m) / angle (rad)", "");
    plotter({{t, xlin, "cart pos"}, {t, anglin, "angle"}}, sprintf("%s/lin_%d.png", path, i), "t (s)", "position (m) / angle (rad)", "");

    % pos cmp 
    plotter({{t, x, "cart pos"}, {t, xlin, "cart pos (lin)"}}, sprintf("%s/pos_cmp_%d.png", path, i), "t (s)", "position (m)", "");
    % ang cmp
    plotter({{t, ang, "angle"}, {t, anglin, "angle (lin)"}}, sprintf("%s/ang_cmp_%d.png", path, i), "t (s)", "angle (rad)", "");

    % err 
    plotter({{t, x - xlin, "cart pos diff"}, {t, ang - anglin, "angle diff"}}, sprintf("%s/err_%d.png", path, i), "t (s)", "error (m) / angle (rad)", "");
end

%% Simulation of free motion with different initial conditions
theta0 = 0.1; 
res = sim("free_motion.slx", 5);
t = res.tout;
x = res.x;
ang = res.ang;
xlin = res.xlin;
anglin = res.anglin;

 % pos + ang 
plotter({{t, x, "cart pos"}, {t, ang, "angle"}}, sprintf("%s/long.png", path), "t (s)", "position (m) / angle (rad)", "");
plotter({{t, xlin, "cart pos"}, {t, anglin, "angle"}}, sprintf("%s/long_linear.png", path), "t (s)", "position (m) / angle (rad)", "");

% pos cmp 
plotter({{t, x, "cart pos"}, {t, xlin, "cart pos (lin)"}}, sprintf("%s/long_pos_cmp.png", path), "t (s)", "position (m)", "");
% ang cmp
plotter({{t, ang, "angle"}, {t, anglin, "angle (lin)"}}, sprintf("%s/long_ang_cmp.png", path), "t (s)", "angle (rad)", "");


%% MODAL CONTROL
Gamma = [-2, 1, 0, 0;
         0, -2, 0, 0;
         0, 0, -3, 1;
         0, 0, 0, -3];
K = FindControllerSylvester(A, B, Gamma);
fprintf("K = ");
print_matrix(K, 2);

eigK = eig(A + B * K);
fprintf("Eigenvalues of A + B * K = ");
print_matrix(eigK, 2);

%% modelling 
function test_controller(K, time, theta, path, n)
    if ~exist(path, "dir")
        mkdir(path);
    end
    theta0 = theta;
    res = sim("modal_control.slx", time);
    t = res.tout;
    x = res.x;
    ang = res.ang;
    u = res.u;
    xlin = res.xlin;
    anglin = res.anglin;

    plotter({{t, x, "cart pos"}, {t, ang, "angle"}}, sprintf("%s/modal_control_out_%d.png", path, n), "t (s)", "position (m) / angle (rad)", "");
    plotter({{t, u, "control"}}, sprintf("%s/modal_control_u_%d.png", path, n), "t (s)", "control", "");
    plotter({{t, x - xlin, "cart pos diff"}, {t, ang - anglin, "angle diff"}}, sprintf("%s/modal_control_cmp_%d.png", path, n), "t (s)", "position (m)", "");
    plotter({{t, xlin, "cart pos"}, {t, anglin, "angle"}}, sprintf("%s/modal_control_linear_out_%d.png", path, n), "t (s)", "position (m) / angle (rad)", "");
end

theta0 = 0.2;
time = 10;
path = "Report/media/plots/modal_control";
test_controller(K, time, theta0, path, 0);

%% different initial conditions
theta0_arr = [0.3, 0.7, 0.9, 1.0, 1.1, 1.2];
time = 6;
path = "Report/media/plots/modal_control";
for i = 1:length(theta0_arr)
    theta0 = theta0_arr(i);
    test_controller(K, time, theta0, path, i);
end
%% different controllers 
path = "Report/media/plots/modal_controllers";
k_arr = [-4, -6, -8, -10]; 
theta0 = 0.3;
time = 5;
for i = 1:length(k_arr)
    k = k_arr(i);
    Gamma = [k, 1, 0, 0;
            0, k, 1, 0;
            0, 0, k, 1;
            0, 0, 0, k];
    K = FindControllerSylvester(A, B, Gamma);
    fprintf("K = ");
    print_matrix(K, 2);
    eigK = eig(A + B * K);
    fprintf("Eigenvalues of A + B * K = ");
    print_matrix(eigK, 2);
    test_controller(K, time, theta0, path, i);
end

%% 
path = "Report/media/plots/modal_controllers";
theta0 = 0.3;
time = 5;
Gamma = [-0.5, 1, 0, 0;
        0, -0.5, 0, 0;
        0, 0, -4, 1;
        0, 0, 0, -4];
K = FindControllerSylvester(A, B, Gamma);
fprintf("K = ");
print_matrix(K, 2);
eigK = eig(A + B * K);
fprintf("Eigenvalues of A + B * K = ");
print_matrix(eigK, 2);
test_controller(K, time, theta0, path, 5);

%% Modal observer
function L = FindObserverSylvester(A, C, Gamma)
    [Am, An] = size(A);
    [Cm, Cn] = size(C);
    Y = [1, 1, 1, 1; 1, 1, 1, 1]';
    cvx_begin sdp
        variable Q(Am, An)
        Gamma * Q - Q * A == Y * C;
    cvx_end
    L = Q \ Y;
end

function test_observer(K, L, time, theta, path, n)
    if ~exist(path, "dir")
        mkdir(path);
    end
    theta0 = theta;
    res = sim("observer.slx", time);
    t = res.tout;
    state = res.x;
    statehat = res.xhat;

    x = state(:, 1);
    dotx = state(:, 2);
    theta = state(:, 3);
    dottheta = state(:, 4);

    xhat = statehat(:, 1);
    dotxhat = statehat(:, 2);
    thetahat = statehat(:, 3);
    dotthetahat = statehat(:, 4);

    % cmp 
    plotter({{t, x, "cart pos"}, {t, xhat, "cart pos estimation"}}, sprintf("%s/observer_x_cmp_%d.png", path, n), "t (s)", "position (m)", "");
    plotter({{t, dotx, "cart vel"}, {t, dotxhat, "cart vel estimation"}}, sprintf("%s/observer_dotx_cmp_%d.png", path, n), "t (s)", "velocity (m/s)", "");
    plotter({{t, theta, "angle"}, {t, thetahat, "angle estimation"}}, sprintf("%s/observer_theta_cmp_%d.png", path, n), "t (s)", "angle (rad)", "");
    plotter({{t, dottheta, "angle vel"}, {t, dotthetahat, "angle vel estimation"}}, sprintf("%s/observer_dottheta_cmp_%d.png", path, n), "t (s)", "angular velocity (rad/s)", "");

    plotter({{t, x, "cart pos", "style", "-", "color", "#0072BD"}, {t, xhat, "cart pos estimation", "style", "--", "color", "#0072BD"}, {t, dotx, "cart vel", "style", "-", "color", "#D95319"}, {t, dotxhat, "cart vel estimation", "style", "--", "color", "#D95319"}, {t, theta, "angle", "style", "-", "color", "#EDB120"}, {t, thetahat, "angle estimation", "style", "--", "color", "#EDB120"}, {t, dottheta, "angle vel", "style", "-", "color", "#7E2F8E"}, {t, dotthetahat, "angle vel estimation", "style", "--", "color", "#7E2F8E"}}, sprintf("%s/observer_cmp_%d.png", path, n), "t (s)", "state", "");

    % errors 
    plotter({{t, x - xhat, "cart pos est err"}, {t, dotx - dotxhat, "cart vel est err"}, {t, theta - thetahat, "angle est err"}, {t, dottheta - dotthetahat, "angle vel est err"}}, sprintf("%s/observer_err_%d.png", path, n), "t (s)", "state", "");

end

%% 
GammaK = [-6, 1, 0, 0;
         0, -6, 1, 0;
         0, 0, -6, 1;
         0, 0, 0, -6];

GammaL = [-3, 1, 0, 0;
         0, -3, 1, 0;
         0, 0, -3, 1;
         0, 0, 0, -3];

K = FindControllerSylvester(A, B, GammaK);
L = FindObserverSylvester(A, C, GammaL);
fprintf("L = ");
print_matrix(L, 2);
eigL = eig(A + L * C);
fprintf("Eigenvalues of A - L * C = ");
print_matrix(eigL, 2);

%% Modelling
path = "Report/media/plots/modal_observer";
time = 5;
theta0 = 0.3;
test_observer(K, L, time, theta0, path, 1);

%% 
GammaK = [-6, 1, 0, 0;
         0, -6, 1, 0;
         0, 0, -6, 1;
         0, 0, 0, -6];

GammaL = [-10, 1, 0, 0;
         0, -10, 1, 0;
         0, 0, -10, 1;
         0, 0, 0, -10];

K = FindControllerSylvester(A, B, GammaK);
L = FindObserverSylvester(A, C, GammaL);
fprintf("L = ");
print_matrix(L, 2);
eigL = eig(A + L * C);
fprintf("Eigenvalues of A - L * C = ");
print_matrix(eigL, 2);

%% Modelling
path = "Report/media/plots/modal_observer";
time = 3;
theta0 = 0.3;
test_observer(K, L, time, theta0, path, 2);