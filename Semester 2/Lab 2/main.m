
 
function U = get_control_matrix(A, B) 
    U = [B, A * B, A^2 * B];
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

function H = get_hautus_matrix(A, B, C, lambda, mode)
    if mode == 'c'
        H = [A - lambda * eye(size(A)), B];
    end
    if mode == 'o'
        H = [A - lambda * eye(size(A)); C];
    end
end

function [A, P] = get_jordan_form(A) 
    [Pj, Aj] = jordan(A);
    [P, A] = cdf2rdf(Pj, Aj);
end


function K = find_controller(A, B, sigma)
    [Aj, P] = get_jordan_form(A);
    % fprintf("Jordan form of A: \n");
    % print_matrix(Aj, 2);
    % fprintf("P matrix: \n");
    % print_matrix(P, 2);
    Bj = inv(P) * B;
    % fprintf("Bj matrix: \n");
    % print_matrix(Bj, 2);

    % crop matrix 
    removed = [];
    dim = size(A, 1);
    for i = 1:dim
        eigval = Aj(i, i);
        if rank(get_hautus_matrix(A, B, [], eigval, 'c')) < dim
            % fprintf("Uncontrollable eigenvalue: %d, %d\n", eigval, i);
            % fill line with zeros
            Aj(i, :) = 0;
            Bj(i, :) = 0;
            % fill column with zeros
            Aj(:, i) = 0;
            removed = [removed, i];
        end
    end
    % clear zero rows and columns
    Aj = Aj(any(Aj, 2), :);
    Aj = Aj(:, any(Aj, 1));
    Bj = Bj(any(Bj, 2), :);
    % fprintf("Cropped Jordan form of A: \n");
    % print_matrix(Aj, 2);
    % fprintf("Cropped Bj matrix: \n");
    % print_matrix(Bj, 2);

    Kj = acker(Aj, Bj, sigma);
    % fprintf("Kj matrix: \n");
    % print_matrix(Kj, 2);

    % insert zeros to recover original size
    i = 1;
    Kr = [];
    for j = 1:dim
        if ismember(j, removed)
            Kr = [Kr, 0];
        else
            Kr = [Kr, Kj(i)]; 
            i = i + 1;
        end
    end

    % fprintf("Kr matrix: \n");
    % print_matrix(Kr, 2);

    K = Kr * inv(P);
end


%% 
A = [8, 1, 11; 4, 0, 4; -4, -3, -7];
B = [-1; -3; 3];
%% 
fprintf("TASK 1\n");
sigma = [-3, -3]; 
K = find_controller(A, B, sigma);
fprintf("K matrix: \n");
print_matrix(K, 2);
res = sim("task1", 5);
u_arr = res.u;
x_arr = res.x;
t_arr = res.tout;

system_matrix = A - B * K; 
fprintf("System matrix: \n");
print_matrix(system_matrix, 2);

plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, x_arr(:, 2), "x_2"}, {t_arr, x_arr(:, 3), "x_3"}}, "media/plots/task1_x_2.png", "t", "x", "");
plotter({{t_arr, u_arr(:, 1), "u_1"}, {t_arr, u_arr(:, 2), "u_2"}, {t_arr, u_arr(:, 3), "u_3"}}, "media/plots/task1_u_2.png", "t", "u", "");

%% 
fprintf("TASK 2\n");
sigma = [-30, -300]; 
K = find_controller(A, B, sigma);
fprintf("K matrix: \n");
print_matrix(K, 2);
res = sim("task1", 5);
u_arr = res.u;
x_arr = res.x;
t_arr = res.tout;

system_matrix = A - B * K; 
fprintf("System matrix: \n");
print_matrix(system_matrix, 2);

plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, x_arr(:, 2), "x_2"}, {t_arr, x_arr(:, 3), "x_3"}}, "media/plots/task1_x_4.png", "t", "x", "");
plotter({{t_arr, u_arr(:, 1), "u_1"}, {t_arr, u_arr(:, 2), "u_2"}, {t_arr, u_arr(:, 3), "u_3"}}, "media/plots/task1_u_4.png", "t", "u", "");

%%
fprintf("TASK 3\n");
sigma = [-3+9j, -3-9j]; 
K = find_controller(A, B, sigma);
fprintf("K matrix: \n");
print_matrix(K, 2);
res = sim("task1", 5);
u_arr = res.u;
x_arr = res.x;
t_arr = res.tout;

system_matrix = A - B * K; 
fprintf("System matrix: \n");
print_matrix(system_matrix, 2);

plotter({{t_arr, x_arr(:, 1), "x_1"}, {t_arr, x_arr(:, 2), "x_2"}, {t_arr, x_arr(:, 3), "x_3"}}, "media/plots/task1_x_6.png", "t", "x", "");
plotter({{t_arr, u_arr(:, 1), "u_1"}, {t_arr, u_arr(:, 2), "u_2"}, {t_arr, u_arr(:, 3), "u_3"}}, "media/plots/task1_u_6.png", "t", "u", "");

%% TASK 2 
A = [-40, 16, 9, 7; -64, 25, 14 ,12; -26, 11, 7, 3; -48, 18, 14, 8];
C = [-3, 2, -2, 1]; 

[Aj, P] = get_jordan_form(A);
fprintf("Jordan form of A: \n");
print_matrix(Aj, 2);
fprintf("P matrix: \n");
print_matrix(P, 2);
Cj = C * P; 
fprintf("Cj matrix: \n");
print_matrix(Cj, 2);

