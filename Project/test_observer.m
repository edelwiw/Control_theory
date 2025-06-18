function test_observer(K, L, time, theta, path, n)
    if ~exist(path, "dir")
        mkdir(path);
    end
    theta0 = theta;
    simIn = Simulink.SimulationInput('observer');
    simIn = simIn.setVariable('K', K);
    simIn = simIn.setVariable('L', L);
    simIn = simIn.setVariable('theta0', theta0);
    res = sim(simIn.setModelParameter('StopTime', num2str(time)));

    t = res.tout;
    state = res.x;
    statehat = res.xhat;

    x = state(:, 1);
    dotx = state(:, 2);
    theta = state(:, 3);
    dottheta = state(:, 4);

    xhat = statehat(:, 1);
    dotxhat = statehat(:, 2);
    thetahat = statehat(:, 3);
    dotthetahat = statehat(:, 4);

    % cmp 
    plotter({{t, x, "$x$"}, {t, xhat, "$\hat{x}$"}}, sprintf("%s/observer_x_cmp_%d.png", path, n), "t (s)", "position (m)", "");
    plotter({{t, dotx, "$\dot{x}$"}, {t, dotxhat, "$\hat{\dot{x}}$"}}, sprintf("%s/observer_dotx_cmp_%d.png", path, n), "t (s)", "velocity (m/s)", "");
    plotter({{t, theta, "$\theta$"}, {t, thetahat, "$\hat{\theta}$"}}, sprintf("%s/observer_theta_cmp_%d.png", path, n), "t (s)", "angle (rad)", "");
    plotter({{t, dottheta, "$\dot{\theta}$"}, {t, dotthetahat, "$\hat{\dot{\theta}}$"}}, sprintf("%s/observer_dottheta_cmp_%d.png", path, n), "t (s)", "angular velocity (rad/s)", "");

    plotter({{t, x, "$x$", "style", "-", "color", "#0072BD"}, {t, xhat, "$\hat{x}$", "style", "--", "color", "#0072BD"}, {t, dotx, "$\dot{x}$", "style", "-", "color", "#D95319"}, {t, dotxhat, "$\hat{\dot{x}}$", "style", "--", "color", "#D95319"}, {t, theta, "$\theta$", "style", "-", "color", "#EDB120"}, {t, thetahat, "$\hat{\theta}$", "style", "--", "color", "#EDB120"}, {t, dottheta, "$\dot{\theta}$", "style", "-", "color", "#7E2F8E"}, {t, dotthetahat, "$\hat{\dot{\theta}}$", "style", "--", "color", "#7E2F8E"}}, sprintf("%s/observer_cmp_%d.png", path, n), "t (s)", "state", "");

    % errors 
    plotter({{t, x - xhat, "$x - \hat{x}$"}, {t, dotx - dotxhat, "$\dot{x} - \hat{\dot{x}}$"}, {t, theta - thetahat, "$\theta - \hat{\theta}$"}, {t, dottheta - dotthetahat, "$\dot{\theta} - \hat{\dot{\theta}}$"}}, sprintf("%s/observer_err_%d.png", path, n), "t (s)", "state", "");
end