function [t, x, ang] = test_controller(K, time, x0, dx0, theta0, dtheta0, path, n)
    if ~exist(path, "dir")
        mkdir(path);
    end
    simIn = Simulink.SimulationInput('modal_control_initials');
    simIn = simIn.setVariable('K', K);
    simIn = simIn.setVariable('x0', x0);
    simIn = simIn.setVariable('dx0', dx0);
    simIn = simIn.setVariable('theta0', theta0);
    simIn = simIn.setVariable('dtheta0', dtheta0);
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

    state = res.state;
    stalelin = res.statelin;
    x = state(:, 1);
    dotx = state(:, 2);
    theta = state(:, 3);
    dottheta = state(:, 4);

    xlin = stalelin(:, 1);
    dotxlin = stalelin(:, 2);
    thetalin = stalelin(:, 3);
    dotthetalin = stalelin(:, 4);

    plotter({{t, x, "$x$"}, {t, dotx, "$\dot{x}$"}, {t, theta, "$\theta$"}, {t, dottheta, "$\dot{\theta}$"}}, sprintf("%s/state_%d.png", path, n), "t (s)", "state", "");
    plotter({{t, xlin, "$x_l$"}, {t, dotxlin, "$\dot{x}_l$"}, {t, thetalin, "$\theta_l$"}, {t, dotthetalin, "$\dot{\theta}_l$ "}}, sprintf("%s/state_lin_%d.png", path, n), "t (s)", "state", "");
    plotter({{t, dotx, "$\dot{x}$"}, {t, dottheta, "$\dot{\theta}$"}}, sprintf("%s/state_dot_%d.png", path, n), "t (s)", "state", "");
    plotter({{t, dotxlin, "$\dot{x}$"}, {t, dotthetalin, "$\dot{\theta}$"}}, sprintf("%s/state_dot_lin_%d.png", path, n), "t (s)", "state", "");
end