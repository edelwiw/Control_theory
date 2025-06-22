%% 
cvx_setup;
cvx_quiet true;  


[M, m, l, g] = get_initials();
[A, B, D, C] = get_matrices(M, m, l, g); 
%% 


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

%% Different x(0) values 
theta0 = 0.3;
dx0 = 0;
dtheta0 = 0;
x0_arr = [0.0, 0.2, 0.5, 1.0, 2.5, 6.0];
time = 10;
path = "Report/media/plots/modal_control_initials";
if ~exist(path, "dir")
    mkdir(path);
end
for i = 1:length(x0_arr)
    x0 = x0_arr(i);
    test_controller_initials(K, time, x0, dx0, theta0, dtheta0, path, i);
end

%% 
x0 = 0.3;
dx0 = -0.2;
theta0 = 0.6;
dtheta0 = -0.3;

test_controller_initials(K, time, x0, dx0, theta0, dtheta0, path, 7);

%% Different controllers 
path = "Report/media/plots/modal_controllers";
k_arr = [-4, -6, -8, -10]; 
theta0 = 0.3;
time = 5;
res_arr_t = {};
res_arr_x = {};
res_arr_ang = {};
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
    [t, x, ang] = test_controller(K, time, theta0, path, i);
    res_arr_t{end + 1} = t;
    res_arr_x{end + 1} = x;
    res_arr_ang{end + 1} = ang(:);
end

%% 
to_plot_x = {};
to_plot_ang = {};
for i = 1:length(alpha_arr)
    to_plot_x{end + 1} = {cell2mat(res_arr_t(i)), cell2mat(res_arr_x(i)), sprintf("$x$ $(\\alpha = %.2f)$", alpha_arr(i))};
    to_plot_ang{end + 1} = {cell2mat(res_arr_t(i)), cell2mat(res_arr_ang(i)), sprintf("$\\theta$ $(\\alpha = %.2f)$", alpha_arr(i))};
end

plotter(to_plot_x, "Report/media/plots/modal_controllers/x_cmp.png", "t (s)", "position (m)", "");
plotter(to_plot_ang, "Report/media/plots/modal_controllers/ang_cmp.png", "t (s)", "angle (rad)", "");

%% 
path = "Report/media/plots/modal_controllers";
theta0 = 0.3;
time = 15;
k = -4;
Gamma1 = [k, 1, 0, 0;
        0, k, 1, 0;
        0, 0, k, 1;
        0, 0, 0, k];

Gamma2 = [-0.5, 1, 0, 0;
        0, -0.5, 0, 0;
        0, 0, -4, 1;
        0, 0, 0, -4];
[K1, sigma1] = TAU.FindControllerSylvester(A, B, Gamma1);
[K2, sigma2] = TAU.FindControllerSylvester(A, B, Gamma2);
fprintf("K_1 = ");
print_matrix(K1, 2);
fprintf("K_2 = ");
print_matrix(K2, 2);
fprintf("Eigenvalues of A + B * K_1 = \n");
fprintf("\\sigma(A + B * K_1) = ");
print_matrix(sigma1, 2);
fprintf("Eigenvalues of A + B * K_2 = \n");
fprintf("\\sigma(A + B * K_2) = ");

res_arr_t = {};
res_arr_x = {};
res_arr_ang = {};
[t1, x1, ang1] = test_controller(K1, time, theta0, path, 5);
[t2, x2, ang2] = test_controller(K2, time, theta0, path, 5);

res_arr_t{end + 1} = t1;
res_arr_x{end + 1} = x1;
res_arr_ang{end + 1} = ang1;

res_arr_t{end + 1} = t2;
res_arr_x{end + 1} = x2;
res_arr_ang{end + 1} = ang2;

to_plot_x = {};
to_plot_ang = {};
for i = 1:2
    to_plot_x{end + 1} = {cell2mat(res_arr_t(i)), cell2mat(res_arr_x(i)), sprintf("$x$ $(K_%d)$", i)};
    to_plot_ang{end + 1} = {cell2mat(res_arr_t(i)), cell2mat(res_arr_ang(i)), sprintf("$\\theta$ $(K_%d)$", i)};
end

plotter(to_plot_x, "Report/media/plots/modal_controllers/x_cmp_eig.png", "t (s)", "position (m)", "");
plotter(to_plot_ang, "Report/media/plots/modal_controllers/ang_cmp_eig.png", "t (s)", "angle (rad)", "");


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

x = state(:, 1);
dotx = state(:, 2);
theta = state(:, 3);
dottheta = state(:, 4);
plotter({{t, x, "$x$"}, {t, dotx, "$\dot{x}$"}, {t, theta, "$\theta$"}, {t, dottheta, "$\dot{\theta}$"}}, sprintf("%s/observer_controller_state.png", path), "t (s)", "state", "");