rng(30, "philox");
M = randi([100000 1000000]) / 1000 / sqrt(2);
m = randi([1000 10000]) / 1000 * sqrt(3);
l = randi([100 1000]) / 100 / sqrt(5);
g = 9.81;

fprintf("M = %.3f, m = %.3f, l = %.3f\n", M, m, l);

l = l / 2;
A = [0, 1, 0, 0;
    0, 0, -m * g / M, 0;
    0, 0, 0, 1;
    0, 0, g * (M + m) / (l * M), 0];
B = [0; 0; 1 / M; -1 / (l * M)];
D = [0; -1/(l * M); 0; (M + m) / (m * l * l * M)];
C = [1, 0, 0, 0; 0, 0, 1, 0];

fprintf("A = ");
print_matrix(A, 2);
fprintf("B = ");
print_matrix(B, 2);
fprintf("D = ");
print_matrix(D, 2);
fprintf("C = ");
print_matrix(C, 0);

%%  find eigenvectors 
[eivA, eigA] = eig(A);
fprintf("Eigenvectors of A = ");
print_matrix(eivA, 2);
fprintf("Eigenvalues of A = ");
print_matrix(eigA, 2);

%% Controllability and Observability
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

U = get_controllability_matrix(A, B);
fprintf("Controllability matrix U = ");
print_matrix(U, 2);
W = get_observability_matrix(A, C);
fprintf("Observability matrix W = ");
print_matrix(W, 2);
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
    plotter({{t, x - xlin, "cart pos err"}, {t, ang - anglin, "angle err"}}, sprintf("%s/err_%d.png", path, i), "t (s)", "error (m) / angle (rad)", "");
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

