% plots = array of {mag, phase, omega, legend}
function plot_freq_response(plots, path, mode)
    line_styles = ["-", "--", "-.", ":"];
    colors = ["#0072BD", "#D95319", "#EDB120", "#7E2F8E", "#77AC30"];
    
    fig = figure('Position', [10 10 900 600]);
    set(gca, 'LooseInset', get(gca, 'TightInset') * 2.0);

    legendText_arr = {};
    for i = 1:length(plots)
        legendText_arr{end + 1} = plots{i}{4};
    end

    % magnitudes
    subplot(2, 1, 1);
    for i = 1:length(plots)
        style = line_styles(mod(i - 1, 4) + 1);
        color = colors(mod(i - 1, 5) + 1);
        mag = plots{i}{1};
        omega = plots{i}{3};

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

        if mode == "loglog"
            loglog(omega, mag, 'LineWidth', 1.8, 'LineStyle', style, 'Color', color);
        elseif mode == "log"
            semilogx(omega, mag, 'LineWidth', 1.8, 'LineStyle', style, 'Color', color);
        elseif mode == "lin"
            plot(omega, mag, 'LineWidth', 1.8, 'LineStyle', style, 'Color', color);
        end    
        hold on;
    end
    xlabel('\omega (rad/s)')
    ylabel('Magnitude')
    grid("on");
    grid("minor");
    leg = legend(legendText_arr);
    fontsize(leg, 18, 'points');
    fontsize(gca, 14, 'points');

    % phases 
    subplot(2, 1, 2);
    for i = 1:length(plots)
        style = line_styles(mod(i - 1, 4) + 1);
        color = colors(mod(i - 1, 5) + 1);
        phase = plots{i}{2};
        omega = plots{i}{3};

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

        if mode == "loglog" || mode == "log"
            semilogx(omega, phase, 'LineWidth', 1.8, 'LineStyle', style, 'Color', color);
        elseif mode == "lin"
            plot(omega, phase, 'LineWidth', 1.8, 'LineStyle', style, 'Color', color);
        end   
        hold on; 
    end

    grid("on");
    grid("minor");
    xlabel('\omega (rad/s)')
    ylabel('Phase')
    leg = legend(legendText_arr);
    fontsize(leg, 18, 'points');
    fontsize(gca, 14, 'points');


    saveas(fig, path);
    % close(fig); 
end
