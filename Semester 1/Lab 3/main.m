const_node = @(a) @(t) a;
linear_node = @(a, b) @(t) (a * t + b);
sin_node = @(a, b) @(t) (a * sin(b * t));
cos_node = @(a, b) @(t) (a * cos(b * t));

%% 

% y(0), dy(0)
initials = [[-1, 0]; [0, 0]; [1, 0]];
% u1(t), u2(t), u3(t)
inputs = {const_node(0), linear_node(0.5, 0), cos_node(1, 2)};
% a0 a1 
cases = [[43.84, 5.6]; [324 0]; [36.64 -1.6]];
%% 
% 
for casen = 1:size(cases, 1)
    for inputn = 1:size(inputs, 2)
        to_plot = {{}; {}; {}};
        for initial = 1:size(initials, 1)
            a0 = cases(casen, 1);
            a1 = cases(casen, 2);
            y0 = initials(initial, 1);
            dy0 = initials(initial, 2);
            func = inputs{inputn};
            legend = sprintf("y(0) = %d, dy(0) = %d", y0, dy0);
            res = sim("scheme1.slx", 3);
            to_plot{initial} = {res.y.Time, res.y.Data, legend};
        end
        plotter(to_plot, sprintf("media/plots/case%d_input%d.png", casen, inputn), "t", "y(t)");
    end
end

%%
function plot_points_on_complex_plane(points, filename, angle, plot_angle, plot_min)
    fig = figure('Position', [10 10 900 900]);
    hold on;
    grid("on");
    grid("minor");
    xlabel("Re");
    ylabel("Im");
    set(gca, 'LooseInset', get(gca, 'TightInset'));
    axis equal;
    set(gca, 'XAxisLocation', 'origin');
    set(gca, 'YAxisLocation', 'origin');
    min_real = min(real(points));
    min_imag = min(imag(points));  
    gmin = abs(min(min_real, min_imag));
    axis([-gmin * 1.2, gmin * 1.2, -gmin * 1.2, gmin * 1.2]);
    % vertical line at min real
    if plot_min
        plot([min_real, min_real], [-gmin * 1.2, gmin * 1.2], 'LineStyle', '--', 'Color', '#EDB120', 'LineWidth', 1.8);
    end
    if plot_angle
        plot([0, -cos(angle) * gmin * 10], [0, sin(angle) * gmin * 10], 'LineStyle', '--', 'Color', '#EDB120', 'LineWidth', 1.8);
        plot([0, -cos(angle) * gmin * 10], [0, -sin(angle) * gmin * 10], 'LineStyle', '--', 'Color', '#EDB120', 'LineWidth', 1.8);
    end
    for i = 1:size(points, 2)
        plot(real(points(i)), imag(points(i)), '.', 'MarkerSize', 40);
    end 
    leg = legend('', '', '', '$\lambda_1$', '$\lambda_2$', '$\lambda_3$');
    set(leg,'Interpreter','Latex');
    fontsize(leg, 18, 'points');
    fontsize(gca, 14, 'points');
    saveas(fig, filename);
    % close(fig);
end

%% 

target = 5;
cor_delta = 0.1;

%  lambda1, lambda2, lambda3
case1 = [-3, -3, -3];
case2 = [-5, -3, -3];
case3 = [-5, -5, -3];
case4 = [-5, -5, -1];
case5 = [-5, -1, -1];

case6 = [-3, -3 - 5j, -3 + 5j];
case7 = [-3, -3 - 15j, -3 + 15j];
case8 = [-3, -1 - 15j, -1 + 15j];
case9 = [-10, -1 - 15j, -1 + 15j];
case10 = [-10, -5 - 15j, -5 + 15j]; 

cases = [case1; case2; case3; case4; case5; case6; case7; case8; case9; case10];

for casen = 1:size(cases, 1) 
    lambda1 = cases(casen, 1);
    lambda2 = cases(casen, 2);
    lambda3 = cases(casen, 3);

    % find point with greatest imaginary part
    mx = 0;
    for i = 1:size(cases(casen, :), 2)
        if abs(imag(cases(casen, i))) > mx
            mx = cases(casen, i);
        end
    end
    % calc angle
    angle = atan(imag(mx) / real(mx));
    fprintf("Case %d:\nangle %f\n", casen, angle);
    reals = real(cases(casen, :));
    max_real = max(reals);
    fprintf("max real %f\n", max_real);
    min_real = min(reals);
    fprintf("min real %f\n", min_real);

    plot_points_on_complex_plane(cases(casen, :), sprintf("media/plots/task2_points%d.png", casen), angle, true, true);
    

    k3 = 1;
    k2 = -(lambda1 + lambda2 + lambda3);
    k1 = (lambda1 * lambda2 + lambda1 * lambda3 + lambda2 * lambda3);
    k0 = -lambda1 * lambda2 * lambda3;

    res = sim("scheme2.slx", 7);

    % Calculate Transient response time as last time when y(t) joins corridor
    transient_time = 0;
    transient_val = 0;
    for t = size(res.y.Time, 1):-1:1
        if ~((res.y.Data(t) >= target - cor_delta) && (res.y.Data(t) <= target + cor_delta))
            transient_time = res.y.Time(t);
            transient_val = res.y.Data(t);
            break;
        end
    end
   
    fprintf("transient time %f\n", transient_time);
    fprintf("overcontrol %f\n\n", (max(res.y.Data) - target) / target);
    func_line = {res.y.Time, res.y.Data, "y(t)"};
    target_line = {res.y.Time, arrayfun(@(t) target, res.y.Time), "target"};
    corridor_line1 = {res.y.Time, arrayfun(@(t) target - cor_delta, res.y.Time), "", "style", ":", "color", "#EDB120"};
    corridor_line2 = {res.y.Time, arrayfun(@(t) target + cor_delta, res.y.Time), "", "style", ":", "color", "#EDB120"};
    transient_time_line = {arrayfun(@(t) transient_time, res.y.Time), arrayfun(@(t) transient_val * (t / res.y.Time(end)), res.y.Time), "", "style", "-.", "color", "#7E2F8E"};
    plotter({func_line, target_line, corridor_line1, corridor_line2, transient_time_line}, sprintf("media/plots/task2_case%d.png", casen), "t", "y(t)");
    
end 

%% 