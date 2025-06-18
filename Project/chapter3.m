%% 
cvx_setup;
cvx_quiet true;  


[M, m, l, g] = get_initials();
[A, B, D, C] = get_matrices(M, m, l, g); 
%% 

function test_controller(K, time, theta0, path, n)
    if ~exist(path, "dir")
        mkdir(path);
    end
    simIn = Simulink.SimulationInput('modal_control');
    simIn = simIn.setVariable('K', K);
    simIn = simIn.setVariable('theta0', theta0);
    res = sim(simIn.setModelParameter('StopTime', num2str(time)));
    
    t = res.tout;
    x = res.x;
    ang = res.ang;
    u = res.u;
    xlin = res.xlin;
    anglin = res.anglin;

    plotter({{t, x, "$x$"}, {t, ang, "$\theta$"}}, sprintf("%s/out_%d.png", path, n), "t (s)", "position (m) / angle (rad)", "");
    plotter({{t, u, "$u$"}}, sprintf("%s/u_%d.png", path, n), "t (s)", "control", "");
    plotter({{t, x - xlin, "$x - x_l$"}, {t, ang - anglin, "$\theta - \theta_l$"}}, sprintf("%s/cmp_%d.png", path, n), "t (s)", "position (m)", "");
    plotter({{t, xlin, "$x$"}, {t, anglin, "$\theta$"}}, sprintf("%s/linear_out_%d.png", path, n), "t (s)", "position (m) / angle (rad)", "");
end

function test_observer(K, L, time, theta, path, n)
    if ~exist(path, "dir")
        mkdir(path);
    end
    theta0 = theta;
    simIn = Simulink.SimulationInput('observer');
    simIn = simIn.setVariable('K', K);
    simIn = simIn.setVariable('L', L);
    simIn = simIn.setVariable('theta0', theta0);
    res = sim(simIn.setModelParameter('StopTime', num2str(time)));

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
    plotter({{t, x, "$x$"}, {t, xhat, "$\hat{x}$"}}, sprintf("%s/observer_x_cmp_%d.png", path, n), "t (s)", "position (m)", "");
    plotter({{t, dotx, "$\dot{x}$"}, {t, dotxhat, "$\hat{\dot{x}}$"}}, sprintf("%s/observer_dotx_cmp_%d.png", path, n), "t (s)", "velocity (m/s)", "");
    plotter({{t, theta, "$\theta$"}, {t, thetahat, "$\hat{\theta}$"}}, sprintf("%s/observer_theta_cmp_%d.png", path, n), "t (s)", "angle (rad)", "");
    plotter({{t, dottheta, "$\dot{\theta}$"}, {t, dotthetahat, "$\hat{\dot{\theta}}$"}}, sprintf("%s/observer_dottheta_cmp_%d.png", path, n), "t (s)", "angular velocity (rad/s)", "");

    plotter({{t, x, "$x$", "style", "-", "color", "#0072BD"}, {t, xhat, "$\hat{x}$", "style", "--", "color", "#0072BD"}, {t, dotx, "$\dot{x}$", "style", "-", "color", "#D95319"}, {t, dotxhat, "$\hat{\dot{x}}$", "style", "--", "color", "#D95319"}, {t, theta, "$\theta$", "style", "-", "color", "#EDB120"}, {t, thetahat, "$\hat{\theta}$", "style", "--", "color", "#EDB120"}, {t, dottheta, "$\dot{\theta}$", "style", "-", "color", "#7E2F8E"}, {t, dotthetahat, "$\hat{\dot{\theta}}$", "style", "--", "color", "#7E2F8E"}}, sprintf("%s/observer_cmp_%d.png", path, n), "t (s)", "state", "");

    % errors 
    plotter({{t, x - xhat, "$x - \hat{x}$"}, {t, dotx - dotxhat, "$\dot{x} - \hat{\dot{x}}$"}, {t, theta - thetahat, "$\theta - \hat{\theta}$"}, {t, dottheta - dotthetahat, "$\dot{\theta} - \hat{\dot{\theta}}$"}}, sprintf("%s/observer_err_%d.png", path, n), "t (s)", "state", "");
end

function test_reduced_observer(K, Q, time, theta, path, n)
    if ~exist(path, "dir")
        mkdir(path);
    end
    theta0 = theta;
    simIn = Simulink.SimulationInput('reduced_observer');
    simIn = simIn.setVariable('K', K);
    simIn = simIn.setVariable('Q', Q);
    simIn = simIn.setVariable('theta0', theta0);
    res = sim(simIn.setModelParameter('StopTime', num2str(time)));

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
    plotter({{t, x, "$x$"}, {t, xhat, "$\hat{x}$"}}, sprintf("%s/reduced_observer_x_cmp_%d.png", path, n), "t (s)", "position (m)", "");
    plotter({{t, dotx, "$\dot{x}$"}, {t, dotxhat, "$\hat{\dot{x}}$"}}, sprintf("%s/reduced_observer_dotx_cmp_%d.png", path, n), "t (s)", "velocity (m/s)", "");
    plotter({{t, theta, "$\theta$"}, {t, thetahat, "$\hat{\theta}$"}}, sprintf("%s/reduced_observer_theta_cmp_%d.png", path, n), "t (s)", "angle (rad)", "");
    plotter({{t, dottheta, "$\dot{\theta}$"}, {t, dotthetahat, "$\hat{\dot{\theta}}$"}}, sprintf("%s/reduced_observer_dottheta_cmp_%d.png", path, n), "t (s)", "angular velocity (rad/s)", "");
    plotter({{t, x, "$x$", "style", "-", "color", "#0072BD"}, {t, xhat, "$\hat{x}$", "style", "--", "color", "#0072BD"}, {t, dotx, "$\dot{x}$", "style", "-", "color", "#D95319"}, {t, dotxhat, "$\hat{\dot{x}}$", "style", "--", "color", "#D95319"}, {t, theta, "$\theta$", "style", "-", "color", "#EDB120"}, {t, thetahat, "$\hat{\theta}$", "style", "--", "color", "#EDB120"}, {t, dottheta, "$\dot{\theta}$", "style", "-", "color", "#7E2F8E"}, {t, dotthetahat, "$\hat{\dot{\theta}}$", "style", "--", "color", "#7E2F8E"}}, sprintf("%s/reduced_observer_cmp_%d.png", path, n), "t (s)", "state", "");

    % errors
    plotter({{t, x - xhat, "$x - \hat{x}$"}, {t, dotx - dotxhat, "$\dot{x} - \hat{\dot{x}}$"}, {t, theta - thetahat, "$\theta - \hat{\theta}$"}, {t, dottheta - dotthetahat, "$\dot{\theta} - \hat{\dot{\theta}}$"}}, sprintf("%s/reduced_observer_err_%d.png", path, n), "t (s)", "state", "");
end


%% MODAL CONTROL
Gamma = [-2, 1, 0, 0;
         0, -2, 0, 0;
         0, 0, -3, 1;
         0, 0, 0, -3];

[K, sigma] = TAU.FindControllerSylvester(A, B, Gamma);
fprintf("K = ");
print_matrix(K, 2);

fprintf("Eigenvalues of A + B * K = \n\\sigma(A + B * K) = ");
print_matrix(sigma, 2);

%% 

theta = 0.2;
time = 10;
path = "Report/media/plots/modal_control";
test_controller(K, time, theta, path, 0);

%% Different initial conditions
theta0_arr = [0.3, 0.7, 0.9, 1.0, 1.1, 1.2];
time = 6;
path = "Report/media/plots/modal_control";
for i = 1:length(theta0_arr)
    theta0 = theta0_arr(i);
    test_controller(K, time, theta0, path, i);
end

%% Different controllers 
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
    K = TAU.FindControllerSylvester(A, B, Gamma);
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
K = TAU.FindControllerSylvester(A, B, Gamma);
fprintf("K = ");
print_matrix(K, 2);
eigK = eig(A + B * K);
fprintf("Eigenvalues of A + B * K = ");
print_matrix(eigK, 2);
test_controller(K, time, theta0, path, 5);

%% MODAL OBSERVER
GammaK = [-6, 1, 0, 0;
         0, -6, 1, 0;
         0, 0, -6, 1;
         0, 0, 0, -6];

GammaL = [-3, 1, 0, 0;
         0, -3, 1, 0;
         0, 0, -3, 1;
         0, 0, 0, -3];

[K, sigmaK] = TAU.FindControllerSylvester(A, B, GammaK);
[L, sigmaL] = TAU.FindObserverSylvester(A, C, GammaL);
fprintf("L = ");
print_matrix(L, 2);
fprintf("Eigenvalues of A - L * C = \n");
fprintf("\\sigma(A - L * C) = ");
print_matrix(sigmaL, 2);

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

[K, sigmaK] = TAU.FindControllerSylvester(A, B, GammaK);
[L, sigmaL] = TAU.FindObserverSylvester(A, C, GammaL);
fprintf("L = ");
print_matrix(L, 2);
fprintf("Eigenvalues of A - L * C = \n");
fprintf("\\sigma(A - L * C) = ");
print_matrix(sigmaL, 2);

%% Modelling
path = "Report/media/plots/modal_observer";
time = 3;
theta0 = 0.3;
test_observer(K, L, time, theta0, path, 2);

%% REDUCED ORDER OBSERVER

G = [-4, 0; 0, -5];
[Ql, Yl] = TAU.FindReducedOrderObserver(A, C, G);
fprintf("Ql = ");
print_matrix(Ql, 2);

path = "Report/media/plots/reduced_observer";
time = 5;
theta0 = 0.3;
Gl = [-4, 0; 0, -5];
[Ql, Yl] = TAU.FindReducedOrderObserver(A, C, Gl);
test_reduced_observer(K, Ql, time, theta0, path, 1);

Gl = [-1, 1; 0, -1];
[Ql, Yl] = TAU.FindReducedOrderObserver(A, C, Gl);
test_reduced_observer(K, Ql, time, theta0, path, 2);

%% Observer + controller 
GammaK = [-9, 1, 0, 0;
         0, -9, 1, 0;
         0, 0, -9, 1;
         0, 0, 0, -9];
Gl = [-5, 1; 0, -5];

K = TAU.FindControllerSylvester(A, B, GammaK);
[Ql, Yl] = TAU.FindReducedOrderObserver(A, C, Gl);

path = "Report/media/plots/observer_controller";
if ~exist(path, "dir")
    mkdir(path);
end
theta0 = 0.1;
time = 3;
res = sim("model_shok.slx", time);

t = res.tout;
x = res.x;
ang = res.ang;
state = res.state;
u = res.u;

plotter({{t, x, "$x$"}, {t, ang, "$\theta$"}}, sprintf("%s/observer_controller_out.png", path), "t (s)", "position (m) / angle (rad)", "");
plotter({{t, u, "u"}}, sprintf("%s/observer_controller_u.png", path), "t (s)", "control", "");