% (m * l * l * (-m * l * power(u[4], 2) * sin(u[3]) + u[1]) + m * l * cos(u[3]) * (m * g * l * sin(u[3]) + u[2])) / (m * l * l * (Mcart + m - m * power(cos(u[3]), 2)))
% ((m * g * l * sin(u[3]) + u[2]) * (Mcart + m) + m * l * cos(u[3]) * (-m * l * power(u[4], 2) * sin(u[3]) + u[1])) / (m * l * l * (Mcart + m - m * power(cos(u[3]), 2)))

%% 

[M, m, l, g] = get_initials();
[A, B, D, C] = get_matrices(M, m, l, g); 

function print_system(A, B, C, D)
    fprintf("A = ");
    print_matrix(A, 2);
    fprintf("B = ");
    print_matrix(B, 4);
    fprintf("D = ");
    print_matrix(D, 3);
    fprintf("C = ");
    print_matrix(C, 0);
end

fprintf("\nSystem parameters:\n");
fprintf("M = %.3f, m = %.3f, l = %.3f\n", M, m, l);
print_system(A, B, C, D);


%% find eigenvectors 
[eivA, eigA] = eig(A);
fprintf("Eigenvectors of A:\n");
print_matrix(eivA, 2);
eigA = eig(A);
fprintf("Eigenvalues of A:\n");
fprintf("\\sigma(A) = ")
print_matrix(eigA, 2);

%% Controllability and observability
U = TAU.get_controllability_matrix(A, B);
fprintf("Controllability matrix:\nU = ");
print_matrix(U, 4);
W = TAU.get_observability_matrix(A, C);
fprintf("Observability matrix:\nW = ");
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
    plotter({{t, x, "$x$"}, {t, ang, "$\theta$"}}, sprintf("%s/nonlin_%d.png", path, i), "t (s)", "position (m) / angle (rad)", "");
    plotter({{t, xlin, "$x$"}, {t, anglin, "$\theta$"}}, sprintf("%s/lin_%d.png", path, i), "t (s)", "position (m) / angle (rad)", "");

    pos cmp 
    plotter({{t, x, "$x$"}, {t, xlin, "$x_l$"}}, sprintf("%s/pos_cmp_%d.png", path, i), "t (s)", "position (m)", "");
    % ang cmp
    plotter({{t, ang, "$\theta$"}, {t, anglin, "$\theta_l$"}}, sprintf("%s/ang_cmp_%d.png", path, i), "t (s)", "angle (rad)", "");

    % err 
    plotter({{t, x - xlin, "$x - x_l$"}, {t, ang - anglin, "$\theta - \theta_l$"}}, sprintf("%s/err_%d.png", path, i), "t (s)", "error (m) / angle (rad)", "");
end

%% Simulation of free motion with different initial conditions
theta0 = 0.1; 
res = sim("free_motion.slx", 5);
path = "Report/media/plots/free_motion";
t = res.tout;
x = res.x;
ang = res.ang;
xlin = res.xlin;
anglin = res.anglin;

% pos + ang 
plotter({{t, x, "$x$"}, {t, ang, "$\theta$"}}, sprintf("%s/long.png", path), "t (s)", "position (m) / angle (rad)", "");
plotter({{t, xlin, "$x$"}, {t, anglin, "$\theta$"}}, sprintf("%s/long_linear.png", path), "t (s)", "position (m) / angle (rad)", "");
% pos cmp 
plotter({{t, x, "$x$"}, {t, xlin, "$x_l$"}}, sprintf("%s/long_pos_cmp.png", path), "t (s)", "position (m)", "");
% ang cmp
plotter({{t, ang, "$\theta$"}, {t, anglin, "$\theta_l$"}}, sprintf("%s/long_ang_cmp.png", path), "t (s)", "angle (rad)", "");



