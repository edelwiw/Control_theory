% parameters:
%   plots_arr: cell, array of plots to plot (x, y, legend, ["style", style], ["color", color])
%   path: string, path to save the plot
%   x_label: string, x-axis label
%   y_label: string, y-axis label
function plotter(plots, path, x_label, y_label, label)
    line_styles = ["-", "--", "-.", ":"];
    colors = ["#0072BD", "#D95319", "#EDB120", "#7E2F8E", "#77AC30"];
    
    fig = figure('Position', [10 10 900 600]);
    set(gca, 'LooseInset', get(gca, 'TightInset') * 2.0);
    hold on;
    for i = 1:length(plots)
        style = line_styles(mod(i - 1, 4) + 1);
        color = colors(mod(i - 1, 5) + 1);
        if size(plots{i}, 2) >= 5 
            if strcmp(plots{i}{4}, "style")
                style = plots{i}{5};
            end
            if strcmp(plots{i}{4}, "color")
                color = plots{i}{5};
            end
        end
        if size(plots{i}, 2) >= 7 
            if strcmp(plots{i}{6}, "style")
                style = plots{i}{7};
            end
            if strcmp(plots{i}{6}, "color")
                color = plots{i}{7};
            end
        end
        plot(plots{i}{1}, plots{i}{2}, 'LineWidth', 1.8, 'LineStyle', style, 'Color', color);
    end
    grid("on");
    grid("minor");

    legendText_arr = {};
    for i = 1:length(plots)
        legendText_arr{end + 1} = plots{i}{3};
    end
    leg = legend(legendText_arr, 'Interpreter', 'latex');

    fontsize(leg, 18, 'points');
    fontsize(gca, 14, 'points');
    xlabel(x_label);
    ylabel(y_label);
    label = strrep(label, "_", "\_");
    title(label);

    saveas(fig, path);
    close(fig); 
end
