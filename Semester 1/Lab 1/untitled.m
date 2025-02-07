% run model simulation

a0 = 12;
a1 = 19;
a2 = 8;
b0 = 42;
b1 = 24;
b2 = 12;

path = "/Users/alexivanov/Desktop/UNI/Control theory/Lab 1/media";

function createAndSavePlot(plots_arr, path, filename, xlabelText, ylabelText)
    line_styles = ["-", "--", "-.", ":"];
    fig = figure('Position', [10 10 900 600]);
    set(gca, 'LooseInset', get(gca, 'TightInset') * 2.0);
    for i = 1:size(plots_arr, 2)
        plot(plots_arr{i}{1}, plots_arr{i}{2}, 'LineWidth', 1.3, 'LineStyle', line_styles(mod(i - 1, 4) + 1));
        hold on;
    end
    grid("on");
    grid("minor");
    legendText_arr = {};
    for i = 1:size(plots_arr, 2)
        legendText_arr{end + 1} = plots_arr{i}{3};
    end
    leg = legend(legendText_arr);
    fontsize(leg, 18, 'points');
    fontsize(gca, 14, 'points');
    xlabel(xlabelText);
    ylabel(ylabelText);
    saveas(fig, fullfile(path, filename));
    close(fig);
end


% res1 = sim("lab1_system", 10);

% createAndSavePlot({{res1.y.time, res1.y.data, "y"}}, path, "sys1_y(t).png", "t", "y");
% createAndSavePlot({{res1.u.time, res1.u.data, "u"}}, path, "sys1_u(t).png", "t", "u");

% res2 = sim("lab1_system2", 10);
% createAndSavePlot({{res2.y.time, res2.y.data, "y"}}, path, "sys2_y(t).png", "t", "y");
% createAndSavePlot({{res2.u.time, res2.u.data, "u"}}, path, "sys2_u(t).png", "t", "u");

% createAndSavePlot({{res1.y.time, res1.y.data, "y_1"}, {res2.y.time, res2.y.data, "y_2"}}, path, "cmp_sys1_sys2.png", "t", "y");
% createAndSavePlot({{res1.y.time, 42 * res1.y.data, "42 \cdot y_1"}, {res2.y.time, res2.y.data, "y_2"}}, path, "cmp_sys1_sys2_scaled.png", "t", "y");

% res3 = sim("lab1_system3", 10);

% createAndSavePlot({{res3.y.time, res3.y.data, "y"}}, path, "sys3_y(t).png", "t", "y");
% createAndSavePlot({{res3.u.time, res3.u.data, "u"}}, path, "sys3_u(t).png", "t", "u");

% createAndSavePlot({{res1.y.time, res1.y.data, "y_1"}, {res3.y.time, res3.y.data, "y_3"}}, path, "cmp_sys1_sys3.png", "t", "y");

% res4 = sim("lab1_system4", 10);

% createAndSavePlot({{res4.y.time, res4.y.data, "y"}}, path, "sys4_y(t).png", "t", "y");
% createAndSavePlot({{res4.u.time, res4.u.data, "u"}}, path, "sys4_u(t).png", "t", "u");

% createAndSavePlot({{res1.y.time, res1.y.data, "y_1"}, {res4.y.time, res4.y.data, "y_4"}}, path, "cmp_sys1_sys4.png", "t", "y");


% Multy channel 

res5 = sim("lab1_system5", 10);

createAndSavePlot({{res5.y1.time, res5.y1.data, "y_1"}, {res5.y2.time, res5.y2.data, "y_2"}}, path, "sys5_y(t).png", "t", "y");
createAndSavePlot({{res5.u1.time, res5.u1.data, "u_1"}, {res5.u2.time, res5.u2.data, "u_2"}}, path, "sys5_u(t).png", "t", "u");


res6 = sim("lab1_system6", 10);

createAndSavePlot({{res6.y1.time, res6.y1.data, "y_1"}, {res6.y2.time, res6.y2.data, "y_2"}}, path, "sys6_y(t).png", "t", "y");
createAndSavePlot({{res6.u1.time, res6.u1.data, "u_1"}, {res6.u2.time, res6.u2.data, "u_2"}}, path, "sys6_u(t).png", "t", "u");
