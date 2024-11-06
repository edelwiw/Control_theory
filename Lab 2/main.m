
% [lambda1, lambda2, y0, dy0, a0, a1, c1, c2,] 

case1 = [-5.5, -6, 1, 0, 0, 0, 0, 0];
case2 = [-2.8 + 6j, -2.8 - 6j, 1, 0, 0, 0, 0, 0];
case3 = [18j, -18j, 1, 0, 0, 0, 0, 0];
case4 = [0.8 + 6j, 0.8 - 6j, 0.05, 0, 0, 0, 0, 0];
case5 = [5.5, 6, 0.05, 0, 0, 0, 0, 0];
case6 = [-1.7, 1.7, 0, 0.1, 0, 0, 0, 0];

% калясик 
% case1 = [-1, -2, 1, 0, 0, 0, 0, 0];
% case2 = [-0.7 + 5j, -0.7 - 5j, 1, 0, 0, 0, 0, 0];
% case3 = [5j, -5j, 1, 0, 0, 0, 0, 0];
% case4 = [0.7 + 5j, 0.7 - 5j, 0.05, 0, 0, 0, 0, 0];
% case5 = [1, 2, 0.05, 0, 0, 0, 0, 0];
% case6 = [-0.3, 0.3, 0, 0.1, 0, 0, 0, 0];


cases = [case1; case2; case3; case4; case5; case6];

function createAndSavePlot(plots_arr, path, filename, xlabelText, ylabelText, area_color, lim)
    line_styles = ["-", "--", "-.", ":"];
    fig = figure('Position', [10 10 900 600]);
    set(gca, 'LooseInset', get(gca, 'TightInset') * 2.0);
    hold on;
    for i = 1:size(plots_arr, 2)
        plot(plots_arr{i}{1}, plots_arr{i}{2}, 'LineWidth', 1.3, 'LineStyle', line_styles(mod(i - 1, 4) + 1));
        if area_color ~= ""
            area(plots_arr{i}{1}, plots_arr{i}{2}, 'FaceAlpha', 0.1, 'LineStyle', 'none', 'FaceColor', area_color);
        end
    end
    grid("on");
    grid("minor");
    legendText_arr = {};
    for i = 1:size(plots_arr, 2)
        legendText_arr{end + 1} = plots_arr{i}{3};
        if area_color ~= ""
            legendText_arr{end + 1} = plots_arr{i}{4};
        end
    end
    leg = legend(legendText_arr);
    fontsize(leg, 18, 'points');
    fontsize(gca, 14, 'points');
    xlabel(xlabelText);
    ylabel(ylabelText);
    % set ylim if lim is set
    if lim ~= 0
        ylim([0, lim]);
    end
    saveas(fig, fullfile(path, filename));
    close(fig);
end


function C = calculate_real(lambda1, lambda2, y0, dy0)
    A = [1, 1; lambda1, lambda2];
    B = [y0; dy0];
    C = A^-1 * B;
end

function C = calculate_imag(alpha, beta, y0, dy0)
    A = [1, 0; alpha, beta];
    B = [y0; dy0];
    C = A^-1 * B;
end

function C = calculate_multyp(lambda, y0, dy0)
    A = [1, 0; lambda, 1];
    B = [y0; dy0];
    C = A^-1 * B;
end

for i = 1:size(cases, 1)
    a0 = cases(i, 1) * cases(i, 2);
    a1 = -(cases(i, 1) + cases(i, 2));
    cases(i, 5) = a0;
    cases(i, 6) = a1;
    if imag(cases(i, 1)) == 0 && imag(cases(i, 2)) == 0 && cases(i, 1) ~= cases(i, 2)
        C = calculate_real(cases(i, 1), cases(i, 2), cases(i, 3), cases(i, 4));
        cases(i, 7) = C(1);
        cases(i, 8) = C(2);
    elseif cases(i, 1) == cases(i, 2)
        C = calculate_multyp(cases(i, 1), cases(i, 3), cases(i, 4));
        cases(i, 7) = C(1);
        cases(i, 8) = C(2);
    else
        C = calculate_imag(real(cases(i, 1)), imag(cases(i, 1)), cases(i, 3), cases(i, 4));
        cases(i, 7) = C(1);
        cases(i, 8) = C(2);
    end

    lambda1 = cases(i, 1);
    lambda2 = cases(i, 2);
    y0 = cases(i, 3);
    dy0 = cases(i, 4);
    c1 = cases(i, 7);
    c2 = cases(i, 8);

    % res = sim("scheme1.slx", 10);
    % createAndSavePlot({{res.y.Time, res.y.Data, 'Model'}, {res.g.Time, res.g.Data, 'Analytycal'}}, 'media', strcat('case', num2str(i), '.png'), 't', 'y(t)', "", 0);
end
T = table([1;2;3;4;5;6],  cases(:, 1), cases(:, 2), cases(:, 3), cases(:, 4), cases(:, 5), cases(:, 6), cases(:, 7), cases(:, 8), 'VariableNames', {'num', 'lambda1', 'lambda2', 'y0', 'dy0', 'a0', 'a1', 'c1', 'c2'});
disp(T);



% TASK 2
% T1 = 2/11;
% T2 = 1/6;

% first = @(T) (T + T2) ./ (T * T2);
% second = @(T) (T + T1) ./ (T * T1);

% x = 0:0.01:10;
% createAndSavePlot({{x, first(x), 'Edje of stability', 'Asympt. stability'}}, 'media', 'T1.png', 'T_1', 'K(T_1)', "blue", 30);
% createAndSavePlot({{x, second(x), 'Edje of stability', 'Asympt. stability'}}, 'media', 'T2.png', 'T_2', 'K(T_2)', "blue", 30);

% TTK = {{1/6, 1, 5}, {1, 2/11, 13/2}, {1/6, 2, 10}};
% g = 1;
% for i = 1:3
%     T1 = TTK{i}{1};
%     T2 = TTK{i}{2};
%     K = TTK{i}{3};
%     res = sim("scheme2.slx", 50);
%     createAndSavePlot({{res.y.Time, res.y.Data, 'Model'}}, 'media', strcat('case', num2str(6 + i), '.png'), 't', 'y(t)', "", 0);
% end


% TASK 2
A = [0, 4, 0, 0; -4, 0, 0, 0; 0, 0, -8, 5; 0, 0, -5, -8];
C = [0, 1, 0, 1];
x0 = [0, 1, 0, 1]';

res = sim("scheme3.slx", 10);
x = 0:0.01:10;
func = @(t) cos(4 .* t) + exp(-8 .* t) .* cos(5 .* t);
createAndSavePlot({{res.y.Time, res.y.Data, 'Model'}, {x, func(x), 'Equation'}}, 'media', 'generator.png', 't', 'y(t)', "", 0);



% to simulink 
% function y = f(t, lambda1, lambda2, c1, c2)
%     if imag(lambda1) == 0 && imag(lambda2) == 0 && lambda1 ~= lambda2
%         y = c1 * exp(lambda1 * t) + c2 * exp(lambda2 * t);
%     elseif lambda1 == lambda2
%         y = c1 * exp(lambda1 * t) + c2 .* t * exp(lambda2 * t)
%     else
%         alpha = real(lambda1);
%         beta = imag(lambda1);
%         y = exp(alpha * t) * (c1 * cos(beta * t) + c2 * sin(beta * t));
%     end
