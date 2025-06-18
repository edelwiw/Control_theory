[M, m, l, g] = get_initials();
[A, B, D, C] = get_matrices(M, m, l, g); 

%% 

alpha = 3;
[K, sigma] = TAU.FindControllerLMI(A, B, alpha);
fprintf("K = ");
print_matrix(K, 2);

fprintf("Eigenvalues of A + B * K = \n");
fprintf("\\sigma(A + B * K) = ");
print_matrix(sigma, 2);

%% different initial conditions
theta0_arr = [0.3, 0.4, 0.6, 0.8];
time = 3;
path = "Report/media/plots/nonmodal_control";
for i = 1:length(theta0_arr)
    theta0 = theta0_arr(i);
    test_controller(K, time, theta0, path, i);
end

%% different alpha values
alpha_arr = [1, 3, 6];
theta0 = 0.3;
time = 3;
res_arr_t = {};
res_arr_x = {};
res_arr_ang = {};
path = "Report/media/plots/nonmodal_controllers";
for i = 1:length(alpha_arr)
    alpha = alpha_arr(i);
    K = TAU.FindControllerLMI(A, B, alpha);
    fprintf("Alpha = %.2f\n", alpha);
    fprintf("K = ");
    print_matrix(K, 2);
    eigK = eig(A + B * K);
    fprintf("Eigenvalues of A + B * K = ");
    print_matrix(eigK, 2);
    [t, x, ang] = test_controller(K, time, theta0, path, i);
    res_arr_t{end + 1} = t;
    res_arr_x{end + 1} = x;
    res_arr_ang{end + 1} = ang;
end

%% 
to_plot_x = {};
to_plot_ang = {};
for i = 1:length(alpha_arr)
    to_plot_x{end + 1} = {cell2mat(res_arr_t(i)), cell2mat(res_arr_x(i)), sprintf("$x$ $(\\alpha = %.2f)$", alpha_arr(i))};
    to_plot_ang{end + 1} = {cell2mat(res_arr_t(i)), cell2mat(res_arr_ang(i)), sprintf("$\\theta$ $(\\alpha = %.2f)$", alpha_arr(i))};
end

plotter(to_plot_x, "Report/media/plots/nonmodal_controllers/x_cmp.png", "t (s)", "position (m)", "");
plotter(to_plot_ang, "Report/media/plots/nonmodal_controllers/ang_cmp.png", "t (s)", "angle (rad)", "");

%% U min 
theta = 0.05;
alpha = 0.5;
x0 = [0; 0; theta; 0];

path = "Report/media/plots/nonmodal_controlers_min";
[K, mu] = TAU.FindControllerLMIMin(A, B, x0, alpha);
fprintf("Alpha = %.4f\n", alpha);
fprintf("K = ");
print_matrix(K, 2);
eigK = eig(A + B * K);
fprintf("Eigenvalues of A + B * K = ");
print_matrix(eigK, 2);
fprintf("\\mu = %.2f\n", mu);

time = 15;
theta0 = 0.05;
test_controller(K, time, theta0, path, 1);
theta0 = 0.3;
test_controller(K, time, theta0, path, 2);
theta0 = 0.8;
test_controller(K, time, theta0, path, 3);

%% 
theta0 = 0.1;
alpha_arr = [0.05, 0.5, 1];
x0 = [0; 0; theta; 0];

path = "Report/media/plots/nonmodal_controlers_min";
for i = 1:length(alpha_arr)
     alpha = alpha_arr(i);
    [K, mu] = TAU.FindControllerLMIMin(A, B, x0, alpha);
    fprintf("Alpha = %.4f\n", alpha);
    fprintf("K = ");
    print_matrix(K, 2);
    eigK = eig(A + B * K);
    fprintf("Eigenvalues of A + B * K = ");
    print_matrix(eigK, 2);
    fprintf("\\mu = %.2f\n", mu);
    test_controller(K, 30, theta0, path, i + 3);
end

%% observers 
lalpha = 3;
L1 = TAU.FindObserverLMI(A, C, lalpha);
fprintf("L = ");
print_matrix(L1, 2);
eigL = eig(A + L1 * C);
fprintf("Eigenvalues of A - L1 * C = ");
print_matrix(eigL, 2);

lalpha = 10;
L2 = TAU.FindObserverLMI(A, C, lalpha);
fprintf("L = ");
print_matrix(L2, 2);
eigL = eig(A + L2 * C);
fprintf("Eigenvalues of A - L2 * C = ");
print_matrix(eigL, 2);

%% observer + controller
x0 = [0; 0; 0.1; 0];
[K1, sigmak1] = TAU.FindControllerLMI(A, B, 3);
[K2, sigmak2] = TAU.FindControllerLMI(A, B, 10);

[L1, sigmal1] = TAU.FindObserverLMI(A, C, 3);
[L2, sigmal2] = TAU.FindObserverLMI(A, C, 10);

fprintf("K1 = ");
print_matrix(K1, 2);
fprintf("K2 = ");
print_matrix(K2, 2);
fprintf("L1 = ");
print_matrix(L1, 2);
fprintf("L2 = ");
print_matrix(L2, 2);

fprintf("Eigenvalues of A + B * K1 = ");
print_matrix(sigmak1, 2);
fprintf("Eigenvalues of A + B * K2 = ");
print_matrix(sigmak2, 2);
fprintf("Eigenvalues of A - L1 * C = ");
print_matrix(sigmal1, 2);
fprintf("Eigenvalues of A - L2 * C = ");
print_matrix(sigmal2, 2);


%%
path = "Report/media/plots/nonmodal_observer_controller";
if ~exist(path, "dir")
    mkdir(path);
end
theta0 = 0.2;
time = 4;

pairs = {
    {K1, L1};
    {K2, L1};
    {K1, L2};
};
for i = 1:size(pairs, 1)
    K = pairs{i}{1};
    L = pairs{i}{2};

    res = sim("KL.slx", time);
    
    t = res.get("tout");
    x = res.get("x");
    ang = res.get("ang");
    state = res.get("state");
    statehat = res.get("state_est");

    x = state(:, 1);
    dotx = state(:, 2);
    theta = state(:, 3);
    dottheta = state(:, 4);

    xhat = statehat(:, 1);
    dotxhat = statehat(:, 2);
    thetahat = statehat(:, 3);
    dotthetahat = statehat(:, 4);
    
    plotter({{t, x, "$x$"}, {t, ang, "$\theta$"}}, sprintf("%s/kl_%d.png", path, i), "t (s)", "", "");
    plotter({{t, x - xhat, "$x - \hat{x}$"}, {t, dotx - dotxhat, "$\dot{x} - \hat{\dot{x}}$"}, {t, theta - thetahat, "$\theta - \hat{\theta}$"}, {t, dottheta - dotthetahat, "$\dot{\theta} - \hat{\dot{\theta}}$"}}, sprintf("%s/kl_err_%d.png", path, i), "t (s)", "state error", "");

end
