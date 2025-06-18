function [t, x, ang] = test_controller(K, time, theta0, path, n)
    if ~exist(path, "dir")
        mkdir(path);
    end
    simIn = Simulink.SimulationInput('modal_control');
    simIn = simIn.setVariable('K', K);
    simIn = simIn.setVariable('theta0', theta0);
    res = sim(simIn.setModelParameter('StopTime', num2str(time)));
    
    t = res.tout;
    x = res.x;
    ang = res.ang;
    u = res.u;
    xlin = res.xlin;
    anglin = res.anglin;

    plotter({{t, x, "$x$"}, {t, ang, "$\theta$"}}, sprintf("%s/out_%d.png", path, n), "t (s)", "position (m) / angle (rad)", "");
    plotter({{t, u, "$u$"}}, sprintf("%s/u_%d.png", path, n), "t (s)", "control", "");
    plotter({{t, x - xlin, "$x - x_l$"}, {t, ang - anglin, "$\theta - \theta_l$"}}, sprintf("%s/cmp_%d.png", path, n), "t (s)", "position (m)", "");
    plotter({{t, xlin, "$x$"}, {t, anglin, "$\theta$"}}, sprintf("%s/linear_out_%d.png", path, n), "t (s)", "position (m) / angle (rad)", "");
end