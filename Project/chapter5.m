[M, m, l, g] = get_initials();
[A, B, D, C] = get_matrices(M, m, l, g); 

Gamma_f = [0 -1 0 0 0 0 0 0 0 0; 
            1 0 0 0 0 0 0 0 0 0; 
            0 0 0 -2 0 0 0 0 0 0; 
            0 0 2 0 0 0 0 0 0 0; 
            0 0 0 0 0 -3 0 0 0 0; 
            0 0 0 0 3 0 0 0 0 0; 
            0 0 0 0 0 0 0 -4 0 0;
            0 0 0 0 0 0 4 0 0 0;
            0 0 0 0 0 0 0 0 0 -5; 
            0 0 0 0 0 0 0 0 5 0];

Y_f = [0.3, -0.5, 0.2, -1, 0.9, 0.4, 0.6, 0.8, -0.7, 0.1];
w0 = [1.0, 0, 0, 0, 0.3, 0, 0, 0, 0.5, 0]';

res = sim("force.slx", 10);
f = res.f;
t = res.tout; 
path = "Report/media/plots/compensation";
if ~exist(path, "dir")
    mkdir(path);
end
plotter({{t, f, "$f$"}}, sprintf("%s/force.png", path), "t, s", "f", "");

%% 
alphaK = 2;
[K, sigmaK] = TAU.FindControllerLMI(A, B, alphaK);
fprintf("K = ");
print_matrix(K, 2);
fprintf("Eigenvalues of A + B * K = \n");
print_matrix(sigmaK, 2);

%% 
function Kf = FindFeedforwardController(A, B, G_f, Y_f, C_z, D, K1)
    cvx_begin sdp
        variable P(4, 10)
        variable Y(1, 10)
        P * G_f - A * P == B * Y + D * Y_f; 
        C_z * P == 0;
    cvx_end
    Kf = Y - K1 * P;
end

Cz = [0, 0, 1, 0];
Kf = FindFeedforwardController(A, B, Gamma_f, Y_f, Cz, D, K);
fprintf("Feedforward controller Kf = ");
print_matrix(Kf, 2);

%% 
theta0 = 0.2;
res = sim("compensation.slx", 5);
f = res.f;
t = res.tout;
pos = res.pos;
posl = res.posl;
ang = res.ang;
angl = res.angl;
statel = res.statel;
state = res.state;
u = res.u;
ul = res.ul;

path = "Report/media/plots/compensation";
if ~exist(path, "dir")
    mkdir(path);
end
% pos + ang 
n = 1;
x = state(:, 1);
dotx = state(:, 2);
theta = state(:, 3);
dottheta = state(:, 4);
xl = statel(:, 1);
dotxl = statel(:, 2);
xhatl = state(:, 3);
dotxhatl = state(:, 4);

plotter({{t, x, "$x$"}, {t, ang, "$\theta$"}}, sprintf("%s/out_%d.png", path, n), "t (s)", "position (m) / angle (rad)", "");
plotter({{t, xl, "$x$"}, {t, angl, "$\theta$"}}, sprintf("%s/linear_out_%d.png", path, n), "t (s)", "position (m) / angle (rad)", "");

plotter({{t, u, "$u$"}}, sprintf("%s/u_%d.png", path, n), "t (s)", "control", "");
plotter({{t, ul, "$u$"}}, sprintf("%s/ul_%d.png", path, n), "t (s)", "control", "");

plotter({{t, x, "$x$"}, {t, dotx, "$\dot{x}$"}, {t, theta, "$\theta$"}, {t, dottheta, "$\dot{\theta}$"}}, sprintf("%s/state_%d.png", path, n), "t (s)", "state", "");
plotter({{t, xl, "$x$"}, {t, dotxl, "$\dot{x}$"}, {t, xhatl, "$\hat{x}$"}, {t, dotxhatl, "$\hat{\dot{x}}$"}}, sprintf("%s/linear_state_%d.png", path, n), "t (s)", "state", "");

%% 
function Kg = GetFollowController(A, B, G_g, Y_g, C_z, K1)
    cvx_begin sdp
        variable P(4, 10)
        variable Y(1, 10)
        P * G_g - A * P == B * Y; 
        C_z * P - Y_g == 0;
    cvx_end
    Kg = Y - K1 * P;
end

alphaK = 2;
[K, sigmaK] = TAU.FindControllerLMI(A, B, alphaK);
Cz = [0, 0, 1, 0];
Kf = GetFollowController(A, B, Gamma_f, Y_f, Cz, K);

fprintf("Follow controller K_g = ");
print_matrix(Kf, 0);

%% 
theta0 = 0.2;
res = sim("follow.slx", 10);
f = res.f;
t = res.tout;
pos = res.pos;
posl = res.posl;
ang = res.ang;
angl = res.angl;
statel = res.statel;
state = res.state;
u = res.u;
ul = res.ul;

path = "Report/media/plots/follow";
if ~exist(path, "dir")
    mkdir(path);
end
% pos + ang 
n = 1;
x = state(:, 1);
dotx = state(:, 2);
theta = state(:, 3);
dottheta = state(:, 4);
xl = statel(:, 1);
dotxl = statel(:, 2);
xhatl = state(:, 3);
dotxhatl = state(:, 4);

plotter({{t, x, "$x$"}, {t, ang, "$\theta$"}}, sprintf("%s/out_%d.png", path, n), "t (s)", "position (m) / angle (rad)", "");
plotter({{t, xl, "$x$"}, {t, angl, "$\theta$"}}, sprintf("%s/linear_out_%d.png", path, n), "t (s)", "position (m) / angle (rad)", "");

plotter({{t, u, "$u$"}}, sprintf("%s/u_%d.png", path, n), "t (s)", "control", "");
plotter({{t, ul, "$u$"}}, sprintf("%s/ul_%d.png", path, n), "t (s)", "control", "");

plotter({{t, x, "$x$"}, {t, dotx, "$\dot{x}$"}, {t, theta, "$\theta$"}, {t, dottheta, "$\dot{\theta}$"}}, sprintf("%s/state_%d.png", path, n), "t (s)", "state", "");
plotter({{t, xl, "$x$"}, {t, dotxl, "$\dot{x}$"}, {t, xhatl, "$\hat{x}$"}, {t, dotxhatl, "$\hat{\dot{x}}$"}}, sprintf("%s/linear_state_%d.png", path, n), "t (s)", "state", "");

plotter({{t, f, "$g$"}, {t, angl, "$\theta$"}}, sprintf("%s/linear_cmp%d.png", path, n), "t (s)", "angle (rad)", "");
plotter({{t, f, "$g$"}, {t, ang, "$\theta$"}}, sprintf("%s/cmp_%d.png", path, n), "t (s)", "angle (rad)", "");
% error 
plotter({{t, angl - f, "$\theta - g$"}}, sprintf("%s/linear_error_%d.png", path, n), "t (s)", "error", "");
plotter({{t, ang - f, "$\theta - g$"}}, sprintf("%s/error_%d.png", path, n), "t (s)", "error", "");