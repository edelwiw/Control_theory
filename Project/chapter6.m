[M, m, l, g] = get_initials();
[A, B, D, C] = get_matrices(M, m, l, g); 

%% 
Q = 30 .* eye(4); 
R = 1; 
theta0 = 0.2;
x0 = [0; 0; theta0; 0];

[K, J] = TAU.FindLQRController(A, B, Q, R, x0, 1);
fprintf("K = ");
print_matrix(K, 2);
fprintf("J = %f\n", J);
%%

path = "Report/media/plots/LQR";
if ~exist(path, "dir")
    mkdir(path);
end

[t, a, ang] = test_controller(K, 70, theta0, path, 1);

%% 
theta0_arr = [0.3, 1.0, 1.2, 1.5];
for i = 1:length(theta0_arr)
    [t, a, ang] = test_controller(K, 70, theta0_arr(i), path, i + 1);
end

%% 

Q1 = 30 .* eye(4); R1 = 1; 
Q2 = 30 .* eye(4); R2 = 10;
Q3 = 200 .* eye(4); R3 = 1;
Q4 = 200 .* eye(4); R4 = 10;

K1 = TAU.FindLQRController(A, B, Q1, R1, x0, 1);
K2 = TAU.FindLQRController(A, B, Q2, R2, x0, 1);
K3 = TAU.FindLQRController(A, B, Q3, R3, x0, 1);
K4 = TAU.FindLQRController(A, B, Q4, R4, x0, 1);
fprintf("K_1 = ");
print_matrix(K1, 2);
fprintf("J_1 = %f\n", J);
fprintf("K_2 = ");
print_matrix(K2, 2);
fprintf("J_2 = %f\n", J);
fprintf("K_3 = ");
print_matrix(K3, 2);%% 
fprintf("J_3 = %f\n", J);
fprintf("K_4 = ");
print_matrix(K4, 2);
fprintf("J_4 = %f\n", J);

K_arr = {K1, K2, K3, K4};
for i = 1:length(K_arr)
    [t, a, ang] = test_controller(K_arr{i}, 70, theta0, path, i + 5);
end

%% 
Q = 30 .* eye(4); 
R = 1; 

L = TAU.FindObserverKalman(A, C, Q, R);
fprintf("L = ");
print_matrix(L, 2);

%% 
path = "Report/media/plots/kalman";
test_observer(K, L, 5, 0.2, path, 1);

%% 
theta0 = 0.2;
varf = 0.1;
vare = 0.06;

Q = varf .* eye(4);
R = vare; 

L = TAU.FindObserverKalman(A, C, Q, R);
fprintf("L = ");
print_matrix(L, 2);

K = TAU.FindControllerLMI(A, B, 2);

%% 
path = "Report/media/plots/LQG";
if ~exist(path, "dir")
    mkdir(path);
end

time = 5;
n = 1;

simIn = Simulink.SimulationInput('lqg');
simIn = simIn.setVariable('K', K);
simIn = simIn.setVariable('L', L);
simIn = simIn.setVariable('theta0', theta0);
simIn = simIn.setVariable('varf', varf);
simIn = simIn.setVariable('vare', vare);
res = sim(simIn.setModelParameter('StopTime', num2str(time)));

t = res.tout;
state = res.x;
statehat = res.xhat;
ynoise = res.toobs;
y = res.y;
pos = res.pos;
ang = res.ang;

x = state(:, 1);
dotx = state(:, 2);
theta = state(:, 3);
dottheta = state(:, 4);

xhat = statehat(:, 1);
dotxhat = statehat(:, 2);
thetahat = statehat(:, 3);
dotthetahat = statehat(:, 4);

 % cmp 
plotter({{t, pos, "$x$"}, {t, ang, "$\theta$"}}, sprintf("%s/out_%d.png", path, n), "t (s)", "position (m) / angle (rad)", "");
plotter({{t, x, "$x$"}, {t, xhat, "$\hat{x}$"}}, sprintf("%s/observer_x_cmp_%d.png", path, n), "t (s)", "position (m)", "");

plotter({{t, ynoise(:, 1), "$\hat{y}$"}, {t, y(:, 1), "$y$"}}, sprintf("%s/observer_y1_cmp_%d.png", path, n), "t (s)", "output (m)", "");
plotter({{t, ynoise(:, 2), "$\hat{y}$"}, {t, y(:, 2), "$y$"}}, sprintf("%s/observer_y2_cmp_%d.png", path, n), "t (s)", "output (rad)", "");

plotter({{t, dotx, "$\dot{x}$"}, {t, dotxhat, "$\hat{\dot{x}}$"}}, sprintf("%s/observer_dotx_cmp_%d.png", path, n), "t (s)", "velocity (m/s)", "");
plotter({{t, theta, "$\theta$"}, {t, thetahat, "$\hat{\theta}$"}}, sprintf("%s/observer_theta_cmp_%d.png", path, n), "t (s)", "angle (rad)", "");
plotter({{t, dottheta, "$\dot{\theta}$"}, {t, dotthetahat, "$\hat{\dot{\theta}}$"}}, sprintf("%s/observer_dottheta_cmp_%d.png", path, n), "t (s)", "angular velocity (rad/s)", "");

plotter({{t, x, "$x$", "style", "-", "color", "#0072BD"}, {t, xhat, "$\hat{x}$", "style", "--", "color", "#0072BD"}, {t, dotx, "$\dot{x}$", "style", "-", "color", "#D95319"}, {t, dotxhat, "$\hat{\dot{x}}$", "style", "--", "color", "#D95319"}, {t, theta, "$\theta$", "style", "-", "color", "#EDB120"}, {t, thetahat, "$\hat{\theta}$", "style", "--", "color", "#EDB120"}, {t, dottheta, "$\dot{\theta}$", "style", "-", "color", "#7E2F8E"}, {t, dotthetahat, "$\hat{\dot{\theta}}$", "style", "--", "color", "#7E2F8E"}}, sprintf("%s/observer_cmp_%d.png", path, n), "t (s)", "state", "");

% errors 
plotter({{t, x - xhat, "$x - \hat{x}$"}, {t, dotx - dotxhat, "$\dot{x} - \hat{\dot{x}}$"}, {t, theta - thetahat, "$\theta - \hat{\theta}$"}, {t, dottheta - dotthetahat, "$\dot{\theta} - \hat{\dot{\theta}}$"}}, sprintf("%s/observer_err_%d.png", path, n), "t (s)", "state", "");

%% 
path = "Report/media/plots/LQGn";
if ~exist(path, "dir")
    mkdir(path);
end

time = 5;
n = 1;

simIn = Simulink.SimulationInput('LQGn');
simIn = simIn.setVariable('K', K);
simIn = simIn.setVariable('L', L);
simIn = simIn.setVariable('theta0', theta0);
simIn = simIn.setVariable('varf', varf);
simIn = simIn.setVariable('vare', vare);
res = sim(simIn.setModelParameter('StopTime', num2str(time)));

t = res.tout;
state = res.x;
statehat = res.xhat;
ynoise = res.toobs;
y = res.y;
pos = res.pos;
ang = res.ang;

x = state(:, 1);
dotx = state(:, 2);
theta = state(:, 3);
dottheta = state(:, 4);

xhat = statehat(:, 1);
dotxhat = statehat(:, 2);
thetahat = statehat(:, 3);
dotthetahat = statehat(:, 4);

 % cmp 
plotter({{t, pos, "$x$"}, {t, ang, "$\theta$"}}, sprintf("%s/out_%d.png", path, n), "t (s)", "position (m) / angle (rad)", "");
plotter({{t, x, "$x$"}, {t, xhat, "$\hat{x}$"}}, sprintf("%s/observer_x_cmp_%d.png", path, n), "t (s)", "position (m)", "");

plotter({{t, ynoise(:, 1), "$\hat{y}$"}, {t, y(:, 1), "$y$"}}, sprintf("%s/observer_y1_cmp_%d.png", path, n), "t (s)", "output (m)", "");
plotter({{t, ynoise(:, 2), "$\hat{y}$"}, {t, y(:, 2), "$y$"}}, sprintf("%s/observer_y2_cmp_%d.png", path, n), "t (s)", "output (rad)", "");

plotter({{t, dotx, "$\dot{x}$"}, {t, dotxhat, "$\hat{\dot{x}}$"}}, sprintf("%s/observer_dotx_cmp_%d.png", path, n), "t (s)", "velocity (m/s)", "");
plotter({{t, theta, "$\theta$"}, {t, thetahat, "$\hat{\theta}$"}}, sprintf("%s/observer_theta_cmp_%d.png", path, n), "t (s)", "angle (rad)", "");
plotter({{t, dottheta, "$\dot{\theta}$"}, {t, dotthetahat, "$\hat{\dot{\theta}}$"}}, sprintf("%s/observer_dottheta_cmp_%d.png", path, n), "t (s)", "angular velocity (rad/s)", "");

plotter({{t, x, "$x$", "style", "-", "color", "#0072BD"}, {t, xhat, "$\hat{x}$", "style", "--", "color", "#0072BD"}, {t, dotx, "$\dot{x}$", "style", "-", "color", "#D95319"}, {t, dotxhat, "$\hat{\dot{x}}$", "style", "--", "color", "#D95319"}, {t, theta, "$\theta$", "style", "-", "color", "#EDB120"}, {t, thetahat, "$\hat{\theta}$", "style", "--", "color", "#EDB120"}, {t, dottheta, "$\dot{\theta}$", "style", "-", "color", "#7E2F8E"}, {t, dotthetahat, "$\hat{\dot{\theta}}$", "style", "--", "color", "#7E2F8E"}}, sprintf("%s/observer_cmp_%d.png", path, n), "t (s)", "state", "");

% errors 
plotter({{t, x - xhat, "$x - \hat{x}$"}, {t, dotx - dotxhat, "$\dot{x} - \hat{\dot{x}}$"}, {t, theta - thetahat, "$\theta - \hat{\theta}$"}, {t, dottheta - dotthetahat, "$\dot{\theta} - \hat{\dot{\theta}}$"}}, sprintf("%s/observer_err_%d.png", path, n), "t (s)", "state", "");