sin_node = @(a, b) @(t) (a * sin(2 * 3.14 * b * t));
cos_node = @(a, b) @(t) (a * cos(2 * 3.14 * b * t));


function bode_plotter(transfer_function, freq_domain_power, filename)
    figure('Position', [10 10 900 600]);
    set(gca, 'LooseInset', get(gca, 'TightInset') * 2.0);
    
    grid("on");
    grid("minor");
    hold on;
    w = logspace(-2, freq_domain_power, 1000);
    h = bodeplot(transfer_function, w);
    grid("on");
    grid("minor");
    set(findall(gcf,'type','line'),'linewidth',2);
    
    h.showCharacteristic('AllStabilityMargins')
    [Gm, Pm, Wcg, Wcp] = margin(transfer_function);
    fprintf("Gm = %f dB, Pm = %f degrees, Wcg = %f rad/s, Wcp = %f rad/s\n", Gm, Pm, Wcg, Wcp);
    
    saveas(gcf, filename);
    % close(gcf);
end

function plot_points_on_complex_plane(points, filename)
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
    max_real = max(real(points));
    min_imag = min(imag(points));
    max_imag = max(imag(points));

    gmax_real = max([abs(min_real), abs(max_real)]);
    gmax_imag = max([abs(min_imag), abs(max_imag)]);
    gmax = max([gmax_real, gmax_imag]);
    axis([-gmax * 1.2, gmax * 1.2, -gmax * 1.2, gmax * 1.2]);
    for i = 1:size(points, 2)
        if real(points(i)) < 0 
            color = 'b';
        else
            color = 'r';
        end

        plot(real(points(i)), imag(points(i)), '*', 'MarkerSize', 20, 'LineWidth', 2, 'Color', color);
    end 

    leg = legend('$\lambda_1$', '$\lambda_2$', '$\lambda_3$', '$\lambda_4$', '$\lambda_5$');
    set(leg,'Interpreter','Latex');

    fontsize(leg, 18, 'points');
    fontsize(gca, 14, 'points');

    saveas(fig, filename);
    close(fig);
end

function plot_nyquist(transfer_function, filename)
    figure('Position', [10 10 900 900]);

    nyquist(transfer_function, logspace(-10, 10, 10000)); 
    set(findall(gcf,'type','line'),'linewidth',2);
    hold on;
    plot(-1, 0, '+', 'MarkerSize', 10, 'LineWidth', 2, 'Color', 'r');

    xlabel('Re');
    ylabel('Im');   
    set(gca, 'XAxisLocation', 'origin');
    set(gca, 'YAxisLocation', 'origin');

    title("");
    set(gca, 'box', 'off');

    saveas(gcf, filename);
    close(gcf);
end

function A = get_polynom_coefficients(X)
    x1 = X(1); x2 = X(2); x3 = X(3); x4 = X(4); x5 = X(5);
    a_4 = -x1 - x2 - x3 - x4 - x5;
    a_3 = x1 * x2 + x1 * x3 + x1 * x4 + x1 * x5 + x2 * x3 + x2 * x4 + x2 * x5 + x3 * x4 + x3 * x5 + x4 * x5;
    a_2 = -(x1 * x2 * x3 + x1 * x2 * x4 + x1 * x2 * x5 + x1 * x3 * x4 + x1 * x3 * x5 + x1 * x4 * x5 + x2 * x3 * x4 + x2 * x3 * x5 + x2 * x4 * x5 + x3 * x4 * x5);
    a_1 = x1 * x2 * x3 * x4 + x1 * x2 * x3 * x5 + x1 * x2 * x4 * x5 + x1 * x3 * x4 * x5 + x2 * x3 * x4 * x5;
    a_0 = -x1 * x2 * x3 * x4 * x5;
    A = [a_0, a_1, a_2, a_3, a_4];
end

function solve(poles_open, poles_closed, n, time_domain_time, freq_domain_power) 
    A = get_polynom_coefficients(poles_open);
    B = get_polynom_coefficients(poles_closed) - A;
    
    % fprintf("x^5 + %dx^4 + %dx^3 + %dx^2 + %dx + %d = 0\n", A(5), A(4), A(3), A(2), A(1));
    % fprintf("x^5 + %dx^4 + %dx^3 + %dx^2 + %dx + %d = 0\n", B(5) + A(5), B(4) + A(4), B(3) + A(3), B(2) + A(2), B(1) + A(1));
    
    fprintf("TASK %d\n", n);
    fprintf("a_4 = %f, a_3 = %f, a_2 = %f, a_1 = %f, a_0 = %f\n", A(5), A(4), A(3), A(2), A(1));
    fprintf("b_4 = %f, b_3 = %f, b_2 = %f, b_1 = %f, b_0 = %f\n", B(5), B(4), B(3), B(2), B(1));
    fprintf("Transfer functions\n");
    fprintf("W_{\\text{o}} = \\frac{s^5 %+.2fs^4 %+.2fs^3 %+.2fs^2 %+.2fs %+.2f}{s^5 %+.2fs^4 %+.2fs^3 %+.2fs^2 %+.2fs %+.2f}\n", B(5), B(4), B(3), B(2), B(1), A(5), A(4), A(3), A(2), A(1));
    fprintf("W_{\\text{c}} = \\frac{s^5 %+.2fs^4 %+.2fs^3 %+.2fs^2 %+.2fs %+.2f}{2s^5 %+.2fs^4 %+.2fs^3 %+.2fs^2 %+.2fs %+.2f}\n", B(5), B(4), B(3), B(2), B(1), A(5) + B(5), A(4) + B(4), A(3) + B(3), A(2) + B(2), A(1) + B(1));
    
    plot_points_on_complex_plane(poles_open, sprintf("media/plots/task%d_poles_open.png", n));
    plot_points_on_complex_plane(poles_closed, sprintf("media/plots/task%d_poles_closed.png", n));
    
    transfer_function_open = tf(cat(2, 1, flip(B, 2)), cat(2, 1, flip(A, 2)));
    transfer_function_closed = tf(cat(2, 1, flip(B, 2)), cat(2, 1, flip(A, 2)) + cat(2, 1, flip(B, 2)));

    plot_nyquist(transfer_function_open, sprintf("media/plots/task%d_nyquist_open.png", n));

    time = linspace(0, time_domain_time, 1000);
    impulse_response_open = impulse(transfer_function_open, time);
    step_response_open = step(transfer_function_open, time);

    plotter({{time, impulse_response_open, "Impulse response"}}, sprintf("media/plots/task%d_impulse_response_open.png", n), "t", "y", "");
    plotter({{time, step_response_open, "Step response"}}, sprintf("media/plots/task%d_step_response_open.png", n), "t", "y", "");

    impulse_response_closed = impulse(transfer_function_closed, time);
    step_response_closed = step(transfer_function_closed, time);

    plotter({{time, impulse_response_closed, "Impulse response"}}, sprintf("media/plots/task%d_impulse_response_closed.png", n), "t", "y", "");
    plotter({{time, step_response_closed, "Step response"}}, sprintf("media/plots/task%d_step_response_closed.png", n), "t", "y", "");

    bode_plotter(transfer_function_open, freq_domain_power,  sprintf("media/plots/task%d_bode_open.png", n));
    bode_plotter(transfer_function_closed, freq_domain_power, sprintf("media/plots/task%d_bode_closed.png", n));

end


function solve2(num_open, denom_open, denom_closed, Karr, time_domain_time, freq_domain_power, n) 
    for i = 1:size(Karr, 2)
        K = Karr(i);
        num = num_open(K);
        denom = denom_open(K);
        transfer_function = tf(num, denom);
        plot_nyquist(transfer_function, sprintf("media/plots/task%d_nyquist_%d.png", n, i));
    end

    transfer_function = tf(num_open(1), denom_open(1));

    omega = logspace(-2, freq_domain_power, 1000);
    h = freqs(num_open(1), denom_open(1), omega);
    mag_exp = abs(h);
    phase_exp = angle(h);

    % plot_freq_response({{mag_exp, phase_exp, omega}},  sprintf("media/plots/%s_freq_resp_exp_loglog.png", path), "loglog");
    plot_freq_response({{mag_exp, phase_exp, omega, "K=1"}},  sprintf("media/plots/task%d_freqs.png", n), "lin");

    % 
    for i = 1:size(Karr, 2)
        K = Karr(i);
        num = num_open(K);
        denom = denom_open(K);
        denom_cl = denom_closed(K);
    
        tf_open = tf(num, denom);
        tf_closed = tf(num * K, denom_cl);

        time = linspace(0, time_domain_time, 1000);
        % impulse_response_open = impulse(tf_open, time);
        % step_response_open = step(tf_open, time);
        impulse_response_closed = impulse(tf_closed, time);
        step_response_closed = step(tf_closed, time);

        % plotter({{time, impulse_response_open, "Impulse response"}}, sprintf("media/plots/task%d_impulse_response_open_%d.png", n, i), "t", "y", "");
        % plotter({{time, step_response_open, "Step response"}}, sprintf("media/plots/task%d_step_response_open_%d.png", n, i), "t", "y", "");
        plotter({{time, impulse_response_closed, "Impulse response"}}, sprintf("media/plots/task%d_impulse_response_closed_%d.png", n, i), "t", "y", "");
        plotter({{time, step_response_closed, "Step response"}}, sprintf("media/plots/task%d_step_response_closed_%d.png", n, i), "t", "y", "");
    end
end 

function solve3(num, denom_open, tf_closed, tarr, time_domain_time, freq_domain_power, n)
    for i = 1:size(tarr, 2)
        t = tarr(i);
        transfer_function_open = tf(num, denom_open, 'InputDelay', t);
        transfer_function_closed = tf_closed(t);
        time = linspace(0, time_domain_time, 1000);

        % step_response_open = step(transfer_function_open, time);
        step_response_closed = step(transfer_function_closed, time);

        % plotter({{time, step_response_open, "Step response"}}, sprintf("media/plots/task%d_step_response_open_%d.png", n, i), "t", "y", "");
        % plotter({{time, impulse_response_closed, "Impulse response"}}, sprintf("media/plots/task%d_impulse_response_closed_%d.png", n, i), "t", "y", "");
        plotter({{time, step_response_closed, "Step response"}}, sprintf("media/plots/task%d_step_response_closed_%d.png", n, i), "t", "y", "");

        plot_nyquist(transfer_function_open, sprintf("media/plots/task%d_nyquist_open_%d.png", n, i));
        bode_plotter(transfer_function_open, freq_domain_power,  sprintf("media/plots/task%d_bode_open_%d.png", n, i));

    end

end




%% task 1 
%% system 1
poles_open = [1, 1.5, 2, -1, -2];
poles_closed = [1, 1.5, -2.5, -1, -2];
solve(poles_open, poles_closed, 1, 10, 3);

%% system 2
poles_open = [-1, -1.5, -2.5, -3, -3.5]
poles_closed = [-1, -1.5, -2.5, 3, 3.5]
solve(poles_open, poles_closed, 2, 10, 3);

%% system 3
poles_open = [-1, -1.5, 2.5, 3, 3.5]
poles_closed = [-1, -1.5, -2.5, -3, -3.5]
solve(poles_open, poles_closed, 3, 10, 3);

%% task 2
%% system 1
Karr = [1, 0.5, 2, 0.1, 8/9];
num = @(k) [k, -9 * k];
denom_open = @(k) [1, 1, 8];
denom_closed = @(k) [1, 1 + k, 8 - 9 * k];
solve2(num, denom_open, denom_closed, Karr, 10, 1, 4);

%% system 2
Karr = [1, 0.5, 2, 0.1, 0.75, 1.25, 7.5];
num = @(k) [-80 * k, 80 * k, 3 * k, -0.04 * k];
denom_open = @(k) [100, -20, -2, 0.3];
denom_closed = @(k) [100 - 80 * k, -20 + 80 * k, -2 + 3 * k, + 0.3 - 0.04 * k];
solve2(num, denom_open, denom_closed, Karr, 2, 1, 5);

%% task 3
tarr = [0, 0.1, 0.5, 1, 5]; 
num = [9, 2];
denom_open = [1, 6, 1];
stf = tf("s"); 
tf_closed = @(t) (9 * stf + 2) / (stf^2 + (6 + 9 * exp(-t * stf)) * stf + 1 + 2 * exp(-t * stf)) * exp(-t * stf);
solve3(num, denom_open, tf_closed, tarr, 10, 3, 6);

%% system 2
tarr = [0, 0.1, 0.5, 1, 0.3, 1.2, 1.5]; 
num = [8, 4, 2.4];
denom_open = [10, -5, 11];
stf = tf("s");
tf_closed = @(t) ((8 * stf^2 + 4 * stf + 2.4) / ((10 + 8 * exp(-t * stf)) * stf^2 + (-5 + 4 * exp(-t * stf)) * stf + 11 + 2.4 * exp(-t * stf))) * exp(-t * stf);
solve3(num, denom_open, tf_closed, tarr, 200, 3, 7);

%%