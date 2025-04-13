%% 
cvx_setup;
cvx_quiet true;  
%% 

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

function K2 = FindFeedforwardController(A, B, Gamma, Cz, D, Bf, K1) 
    [Gm, Gn] = size(Gamma);
    [Am, An] = size(A);
    [Bm, Bn] = size(B);
    [Bfm, Bfn] = size(Bf);
    cvx_begin sdp
        variable P(An, Gm)
        variable Y(Bn, Bfn)
        P * Gamma - A * P == B * Y + Bf; 
        Cz * P + D == 0;
    cvx_end
    K2 = Y - K1 * P;
end

function [Aj, Bj] = getJordanForm(A, B)
    [V, J] = jordan(A);
    [V, J] = cdf2rdf(V, J);
    Aj = J;
    Bj = V \ B;
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
    % W = [C; C * A; C * A^2];
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

%% TASK 1 
A = [8, 1, 11; 4, 0, 4; -4, -3, -7]; 
B = [-1; -3; 3];
Bf = [0, 1, -1, -1; 0, 0, 0, 0; 0, -1, 0, 0];
Cz = [-2, -3, -1];
Gamma = [-40, 16, 9, 7; -64, 25, 14, 12; -26, 11, 7, 3; -48, 18, 14, 8];

%% 
% eigenvalues of external signal 
Gamma_eig = eig(Gamma); 

fprintf('\sigma(\Gamma) = ');
print_matrix(Gamma_eig, 2);

wf_func = @(t) expm(Gamma * t) * [1; 1; 1; 1];

t_arr = linspace(0, 10, 100);
wf = arrayfun(@(ti) wf_func(ti), t_arr, 'UniformOutput', false);

wf1 = cellfun(@(x) x(1), wf);
wf2 = cellfun(@(x) x(2), wf);
wf3 = cellfun(@(x) x(3), wf);
wf4 = cellfun(@(x) x(4), wf);

plotter({{t_arr, wf1, 'wf_1'}, {t_arr, wf2, 'wf_2'}, {t_arr, wf3, 'wf_3'}, {t_arr, wf4, 'wf_4'}}, 'media/plots/wf.png', 't', 'w_f(t)', '');

%% plot external and system without controller
wfen = 1; % enable external signal  
x0 = [0; 0; 0];
K2 = [0, 0, 0, 0];
K1 = [0, 0, 0];
res = sim("scheme1", 5);
open_t_arr = res.tout;
wf_open = res.wf;
open_u_arr = res.u;
open_x_arr = res.x;
open_z_arr = res.z;

plotter({{open_t_arr, open_x_arr(:, 1), 'x_1'}, {open_t_arr, open_x_arr(:, 2), 'x_2'}, {open_t_arr, open_x_arr(:, 3), 'x_3'}}, 'media/plots/open_x.png', 't', 'x(t)', '');
plotter({{open_t_arr, open_z_arr, 'z'}}, 'media/plots/open_z.png', 't', 'z(t)', '');
plotter({{open_t_arr, open_u_arr(:, 1), 'u'}}, 'media/plots/open_u.png', 't', 'u(t)', '');

%% Find feedback controller
A_eig = eig(A);
fprintf('Eigenvalues of A: \n');
print_matrix(A_eig, 2);

[Aj, Bj] = getJordanForm(A, B);
fprintf('A_j =  ');
print_matrix(Aj, 2);
fprintf('B_j = ');
print_matrix(Bj, 2);

alpha = 3; 
x0 = [0; 0; 0];
[K1, mu] = FindControllerLyapMin(A, B, x0, alpha);
fprintf('K_1 = ');
print_matrix(K1, 2);

sys_eig = eig(A + B * K1);
fprintf('\sigma(A + BK_1) = ');
print_matrix(sys_eig, 2);

%% plot output with K1 
wfen = 0; % disable external signal  
x0 = [1; 1; 1];
K2 = [0, 0, 0, 0];
res = sim("scheme1", 5);
K1_t_arr = res.tout;
K1_u_arr = res.u;
K1_x_arr = res.x;
K1_z_arr = res.z;
plotter({{K1_t_arr, K1_x_arr(:, 1), 'x_1'}, {K1_t_arr, K1_x_arr(:, 2), 'x_2'}, {K1_t_arr, K1_x_arr(:, 3), 'x_3'}}, 'media/plots/K1_free_x.png', 't', 'x(t)', '');
plotter({{K1_t_arr, K1_z_arr, 'z'}}, 'media/plots/K1_free_z.png', 't', 'z(t)', '');
plotter({{K1_t_arr, K1_u_arr(:, 1), 'u'}}, 'media/plots/K1_free_u.png', 't', 'u(t)', '');

wfen = 1; % enable external signal  
x0 = [1; 1; 1];
K2 = [0, 0, 0, 0];
res = sim("scheme1", 5);
K1wf_t_arr = res.tout;
K1wf_u_arr = res.u;
K1wf_x_arr = res.x;
K1wf_z_arr = res.z;
plotter({{K1wf_t_arr, K1wf_x_arr(:, 1), 'x_1'}, {K1wf_t_arr, K1wf_x_arr(:, 2), 'x_2'}, {K1wf_t_arr, K1wf_x_arr(:, 3), 'x_3'}}, 'media/plots/K1_wf_x.png', 't', 'x(t)', '');
plotter({{K1wf_t_arr, K1wf_z_arr, 'z'}}, 'media/plots/K1_wf_z.png', 't', 'z(t)', '');
plotter({{K1wf_t_arr, K1wf_u_arr(:, 1), 'u'}}, 'media/plots/K1_wf_u.png', 't', 'u(t)', '');

%% Find feedforward controller 
K2 = FindFeedforwardController(A, B, Gamma, Cz, [0, 0, 0, 0], Bf, K1);
fprintf('K_2 = ');
print_matrix(K2, 2);

%% plot output with K1 and K2
wfen = 1; % enable external signal  
x0 = [0; 0; 0];
res = sim("scheme1", 5);
full_t_arr = res.tout;
full_u_arr = res.u;
full_x_arr = res.x;
full_z_arr = res.z;

plotter({{full_t_arr, full_x_arr(:, 1), 'x_1'}, {full_t_arr, full_x_arr(:, 2), 'x_2'}, {full_t_arr, full_x_arr(:, 3), 'x_3'}}, 'media/plots/full_x.png', 't', 'x(t)', '');
plotter({{full_t_arr, full_z_arr, 'z'}}, 'media/plots/full_z.png', 't', 'z(t)', '');
plotter({{full_t_arr, full_u_arr(:, 1), 'u'}}, 'media/plots/full_u.png', 't', 'u(t)', '');

%% comparing 

plotter({{K1_t_arr, K1_u_arr(:, 1), 'u_1'}, {K1wf_t_arr, K1wf_u_arr(:, 1), 'u_2'}, {full_t_arr, full_u_arr(:, 1), 'u_3'}}, 'media/plots/u_cmp.png', 't', 'u(t)', '');
plotter({{K1wf_t_arr, K1wf_z_arr, 'z_1'}, {full_t_arr, full_z_arr, 'z_2'}}, 'media/plots/z_cmp.png', 't', 'z(t)', '');

%% TASK 2
Dz = [8, -8, 12, -3];
wgen  = 1;
x0 = [1; 1; 1];
K1 = [0, 0, 0];
K2 = [0, 0, 0, 0];

res = sim("scheme2", 5);
t_arr = res.tout;
wg_arr = res.wg;
z_arr = res.z;
u_arr = res.u;
x_arr = res.x;

plotter({{t_arr, x_arr(:, 1), 'x_1'}, {t_arr, x_arr(:, 2), 'x_2'}, {t_arr, x_arr(:, 3), 'x_3'}}, 'media/plots/task2_open_x.png', 't', 'x(t)', '');
plotter({{t_arr, z_arr, 'z'}}, 'media/plots/task2_open_z.png', 't', 'z(t)', '');

[K1, mu] = FindControllerLyapMin(A, B, x0, alpha);

res = sim("scheme2", 5);
K1_t_arr = res.tout;
K1_wg_arr = res.wg;
K1_z_arr = res.z;
K1_u_arr = res.u;
K1_x_arr = res.x;

plotter({{K1_t_arr, K1_x_arr(:, 1), 'x_1'}, {K1_t_arr, K1_x_arr(:, 2), 'x_2'}, {K1_t_arr, K1_x_arr(:, 3), 'x_3'}}, 'media/plots/task2_K1_x.png', 't', 'x(t)', '');
plotter({{K1_t_arr, K1_z_arr, 'z'}}, 'media/plots/task2_K1_z.png', 't', 'z(t)', '');
plotter({{K1_t_arr, K1_u_arr(:, 1), 'u'}}, 'media/plots/task2_K1_u.png', 't', 'u(t)', '');

%% 
K2 = FindFeedforwardController(A, B, Gamma, Cz, Dz, zeros(3, 4), K1);
fprintf('K_2 = ');
print_matrix(K2, 2);

res = sim("scheme2", 5);
full_t_arr = res.tout;
full_wg_arr = res.wg;
full_z_arr = res.z;
full_u_arr = res.u;
full_x_arr = res.x;

plotter({{full_t_arr, full_x_arr(:, 1), 'x_1'}, {full_t_arr, full_x_arr(:, 2), 'x_2'}, {full_t_arr, full_x_arr(:, 3), 'x_3'}}, 'media/plots/task2_full_x.png', 't', 'x(t)', '');
plotter({{full_t_arr, full_z_arr, 'z'}}, 'media/plots/task2_full_z.png', 't', 'z(t)', '');
plotter({{full_t_arr, full_u_arr(:, 1), 'u'}}, 'media/plots/task2_full_u.png', 't', 'u(t)', '');

%% comparing
plotter({{K1_t_arr, K1_u_arr(:, 1), 'u_1'}, {full_t_arr, full_u_arr(:, 1), 'u_2'}}, 'media/plots/task2_u_cmp.png', 't', 'u(t)', '');
plotter({{K1_t_arr, K1_z_arr, 'z_1'}, {full_t_arr, full_z_arr, 'z_2'}}, 'media/plots/task2_z_cmp.png', 't', 'z(t)', '');

%% TASK 3 

A = [8, 1, 11; 4, 0, 4; -4, -3, -7]; 
B = [-1; -3; 3];
C = [-2, -3, -1];
Bf = [0, 1, -1, -1; 0, 0, 0, 0; 0, -1, 0, 0];
Cz = [-2, -3, -1];
Gamma = [-40, 16, 9, 7; -64, 25, 14, 12; -26, 11, 7, 3; -48, 18, 14, 8];
D = [-12, 2, 2, 6];
Dz = [8, -8, 12, -3];

x0 = [0; 0; 0];
alpha = 3;
[K1, mu] = FindControllerLyapMin(A, B, x0, alpha);

ABFG = [A, Bf; zeros(4, 3), Gamma];
CD = [C, D];
W = get_observability_matrix(CD, ABFG);
fprintf('W = ');
print_matrix(W, 2);
fprintf('rank(W) = %d\n', rank(W));

%% 
e0 = [0; 0; 0; 0; 0; 0; 0];
L = FindObserverMin(ABFG, CD, e0, 3);
fprintf('L = ');
print_matrix(L, 2);

L1 = L(1:3);
L2 = L(4:7);
fprintf('L1 = ');
print_matrix(L1, 2);
fprintf('L2 = ');
print_matrix(L2, 2);


K21 = FindFeedforwardController(A, B, Gamma, Cz, Dz, Bf, K1);
fprintf('K_{21} = ');
print_matrix(K21, 2);

K22 = FindFeedforwardController(A, B, Gamma, C, D, Bf, K1);
fprintf('K_{22} = ');
print_matrix(K22, 2);

%% 
wfen = 1;
K2 = K21; 
Abebra = [A + B * K1 + L1 * C, B * K2 + Bf + L1 * D; L2 * C, Gamma + L2 * D];
fprintf('\\overline{A} = ');
print_matrix(Abebra, 2);
fprintf('\\sigma(\\overline{A}) = ')
abebra_eig = eig(Abebra);
print_matrix(abebra_eig, 2);

%% 
res = sim("scheme3", 5);
t_arr = res.tout;
z1_arr = res.z1;
z2_arr = res.z2;
u_arr = res.u;
x_arr = res.x;
w_arr = res.w;
xhat_arr = res.xhat;
what_arr = res.what;

plotter({{t_arr, u_arr, 'u'}}, 'media/plots/task3_z1_u.png', 't', 'u(t)', '');
plotter({{t_arr, x_arr(:, 1), 'x_1', "style", "-", "color", "#0072BD"}, {t_arr, xhat_arr(:, 1), 'hx_1', "style", "--", "color", "#0072BD"}, {t_arr, x_arr(:, 2), 'x_2', "style", "-", "color", "#D95319"}, {t_arr, xhat_arr(:, 2), 'hx_2', "style", "--", "color", "#D95319"}, {t_arr, x_arr(:, 3), 'x_3', "style", "-", "color", "#EDB120"}, {t_arr, xhat_arr(:, 3), 'hx_3', "style", "--", "color", "#EDB120"}}, 'media/plots/task3_z1_x_cmp.png', 't', 'x(t)', '');
plotter({{t_arr, w_arr(:, 1), 'w_1', "style", "-", "color", "#0072BD"}, {t_arr, what_arr(:, 1), 'hw_1', "style", "--", "color", "#0072BD"}, {t_arr, w_arr(:, 2), 'w_2', "style", "-", "color", "#D95319"}, {t_arr, what_arr(:, 2), 'hw_2', "style", "--", "color", "#D95319"}, {t_arr, w_arr(:, 3), 'w_3', "style", "-", "color", "#EDB120"}, {t_arr, what_arr(:, 3), 'hw_3', "style", "--", "color", "#EDB120"}}, 'media/plots/task3_z1_w_cmp.png', 't', 'w(t)', '');

xerr = x_arr - xhat_arr;
werr = w_arr - what_arr;
plotter({{t_arr, xerr(:, 1), 'xe_1', "style", "-", "color", "#0072BD"}, {t_arr, xerr(:, 2), 'xe_2', "style", "-", "color", "#D95319"}, {t_arr, xerr(:, 3), 'xe_3', "style", "-", "color", "#EDB120"}}, 'media/plots/task3_z1_xerr_cmp.png', 't', 'x(t)', '');
plotter({{t_arr, werr(:, 1), 'we_1', "style", "-", "color", "#0072BD"}, {t_arr, werr(:, 2), 'we_2', "style", "-", "color", "#D95319"}, {t_arr, werr(:, 3), 'we_3', "style", "-", "color", "#EDB120"}}, 'media/plots/task3_z1_werr_cmp.png', 't', 'w(t)', '');

plotter({{t_arr, z1_arr, 'z_1'}}, 'media/plots/task3_z1_z1.png', 't', 'z_1(t)', '');
plotter({{t_arr, z2_arr, 'z_2'}}, 'media/plots/task3_z1_z2.png', 't', 'z_2(t)', '');

plotter({{t_arr, z1_arr, 'z_1'}, {t_arr, z2_arr, 'y'}}, 'media/plots/task3_z1_cmp.png', 't', 'z(t)', '');

%% 
wfen = 1;
K2 = K22; 
Abebra = [A + B * K1 + L1 * C, B * K2 + Bf + L1 * D; L2 * C, Gamma + L2 * D];
fprintf('\\overline{A} = ');
print_matrix(Abebra, 2);
fprintf('\\sigma(\\overline{A}) = ')
abebra_eig = eig(Abebra);
print_matrix(abebra_eig, 2);
%% 

res = sim("scheme3", 5);
t_arr = res.tout;
z1_arr = res.z1;
z2_arr = res.z2;
u_arr = res.u;
x_arr = res.x;
w_arr = res.w;
xhat_arr = res.xhat;
what_arr = res.what;

plotter({{t_arr, u_arr, 'u'}}, 'media/plots/task3_z2_u.png', 't', 'u(t)', '');
plotter({{t_arr, x_arr(:, 1), 'x_1', "style", "-", "color", "#0072BD"}, {t_arr, xhat_arr(:, 1), 'hx_1', "style", "--", "color", "#0072BD"}, {t_arr, x_arr(:, 2), 'x_2', "style", "-", "color", "#D95319"}, {t_arr, xhat_arr(:, 2), 'hx_2', "style", "--", "color", "#D95319"}, {t_arr, x_arr(:, 3), 'x_3', "style", "-", "color", "#EDB120"}, {t_arr, xhat_arr(:, 3), 'hx_3', "style", "--", "color", "#EDB120"}}, 'media/plots/task3_z2_x_cmp.png', 't', 'x(t)', '');
plotter({{t_arr, w_arr(:, 1), 'w_1', "style", "-", "color", "#0072BD"}, {t_arr, what_arr(:, 1), 'hw_1', "style", "--", "color", "#0072BD"}, {t_arr, w_arr(:, 2), 'w_2', "style", "-", "color", "#D95319"}, {t_arr, what_arr(:, 2), 'hw_2', "style", "--", "color", "#D95319"}, {t_arr, w_arr(:, 3), 'w_3', "style", "-", "color", "#EDB120"}, {t_arr, what_arr(:, 3), 'hw_3', "style", "--", "color", "#EDB120"}}, 'media/plots/task3_z2_w_cmp.png', 't', 'w(t)', '');

xerr = x_arr - xhat_arr;
werr = w_arr - what_arr;
plotter({{t_arr, xerr(:, 1), 'xe_1', "style", "-", "color", "#0072BD"}, {t_arr, xerr(:, 2), 'xe_2', "style", "-", "color", "#D95319"}, {t_arr, xerr(:, 3), 'xe_3', "style", "-", "color", "#EDB120"}}, 'media/plots/task3_z2_xerr_cmp.png', 't', 'x(t)', '');
plotter({{t_arr, werr(:, 1), 'we_1', "style", "-", "color", "#0072BD"}, {t_arr, werr(:, 2), 'we_2', "style", "-", "color", "#D95319"}, {t_arr, werr(:, 3), 'we_3', "style", "-", "color", "#EDB120"}}, 'media/plots/task3_z2_werr_cmp.png', 't', 'w(t)', '');

plotter({{t_arr, z1_arr, 'z_1'}}, 'media/plots/task3_z2_z1.png', 't', 'z_1(t)', '');
plotter({{t_arr, z2_arr, 'z_2'}}, 'media/plots/task3_z2_z2.png', 't', 'z_2(t)', '');

plotter({{t_arr, z2_arr, 'z_2'}, {t_arr, z2_arr, 'y'}}, 'media/plots/task3_z2_cmp.png', 't', 'z(t)', '');
