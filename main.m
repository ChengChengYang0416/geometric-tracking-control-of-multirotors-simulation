% A simulation for geometric tracking control of multirotors
close all;

% simulation time
dt = 0.001;
sim_t = 20;

% initialize parameters
multirotor = multirotor_dynamics;
multirotor.dt = dt;
multirotor.sim_t = sim_t;
multirotor.t = 0:dt:sim_t;
multirotor.m = 1.15;
multirotor.J = [0.0131, 0, 0;
                0, 0.0131, 0;
                0, 0, 0.0244];
multirotor.d = 0.225;
multirotor.c_tau = 1.347e-2;
multirotor.allocation_matrix = cal_allocation_matrix(multirotor.d, multirotor.c_tau);
multirotor.allocation_matrix_inv = cal_allocation_matrix_inv(multirotor.allocation_matrix);
multirotor.x = zeros(3, length(multirotor.t));
multirotor.v = zeros(3, length(multirotor.t));
multirotor.R = zeros(9, length(multirotor.t));
multirotor.W = zeros(3, length(multirotor.t));
multirotor.ex = zeros(3, length(multirotor.t));
multirotor.ev = zeros(3, length(multirotor.t));
multirotor.eR = zeros(3, length(multirotor.t));
multirotor.eW = zeros(3, length(multirotor.t));
multirotor.force_moment = zeros(4, length(multirotor.t));
multirotor.rotor_thrust = zeros(4, length(multirotor.t));

% initialize states
multirotor.x(:, 1) = [0; 0; 0];
multirotor.v(:, 1) = [0; 0; 0];
multirotor.R(:, 1) = [1; 0; 0; 0; 1; 0; 0; 0; 1];
multirotor.W(:, 1) = [0; 0; 0];

% initialize controller
ctrl = controller;

% initialize trajectory
tra = zeros(12, length(multirotor.t));
traj = trajectory;

for i = 2:length(multirotor.t)
    t_now = multirotor.t(i);

    % desired trajectory
    tra(:, i) = traj.traj_generate(t_now);
    Xd_enu = tra(1:9, i-1);
    b1d = tra(10:12, i);

    % control input and error
    [control, error] = ctrl.geometric_tracking_ctrl(i, multirotor, Xd_enu, b1d);

    % dynamics
    X0 = [vec_enu_to_ned(multirotor.x(:, i-1));
        vec_enu_to_ned(multirotor.v(:, i-1));
        reshape(reshape(multirotor.R(:, i-1), 3, 3), 9, 1);
        multirotor.W(:, i-1)];
    [T, X_new] = ode45(@(t, x) multirotor.dynamics(t, x, control), [0, dt], X0, control);
    multirotor.x(:, i) = vec_ned_to_enu(X_new(end, 1:3));
    multirotor.v(:, i) = vec_ned_to_enu(X_new(end, 4:6));
    multirotor.R(:, i) = X_new(end, 7:15);
    multirotor.W(:, i) = X_new(end, 16:18);

    % save the error
    multirotor.ex(:, i) = error(1:3);
    multirotor.ev(:, i) = error(4:6);
    multirotor.eR(:, i) = error(7:9);
    multirotor.eW(:, i) = error(10:12);
    
    % save rotor thrust
    multirotor.force_moment(:, i) = control(1:4);
    multirotor.rotor_thrust(:, i) = multirotor.allocation_matrix_inv*control(1:4);
end

% plot trajectory and desired trajectory
figure(1)
subplot(3, 1, 1)
plot(multirotor.t, multirotor.x(1, :))
hold on
plot(multirotor.t, tra(1, :))
y = ylabel('$X(m)$', 'rotation', 0, 'Interpreter', 'latex');
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
legend('$X$', '$X_{d}$' , 'Interpreter', 'latex')
title('$Trajectory$ $and$ $Desired$ $Trajectory$ $(m)$', 'Interpreter', 'latex')
subplot(3, 1, 2)
plot(multirotor.t, multirotor.x(2, :))
hold on
plot(multirotor.t, tra(2, :))
y = ylabel('$Y(m)$', 'rotation', 0, 'Interpreter', 'latex');
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
legend('$Y$', '$Y_{d}$' , 'Interpreter', 'latex')
subplot(3, 1, 3)
plot(multirotor.t, multirotor.x(3, :))
hold on
plot(multirotor.t, tra(3, :))
y = ylabel('$Z(m)$', 'rotation', 0, 'Interpreter', 'latex');
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
legend('$Z$', '$Z_{d}$' , 'Interpreter', 'latex')
xlabel('$Time(sec)$', 'Interpreter', 'latex')

% plot position error
figure(2)
subplot(3, 1, 1)
plot(multirotor.t, multirotor.ex(1, :))
y = ylabel('$e_{p_{x}}$', 'rotation', 0, 'Interpreter', 'latex');
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
title('$Position$ $Error$ $(m)$', 'Interpreter', 'latex')
subplot(3, 1, 2)
plot(multirotor.t, multirotor.ex(2, :))
y = ylabel('$e_{p_{y}}$', 'rotation', 0, 'Interpreter', 'latex');
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
subplot(3, 1, 3)
plot(multirotor.t, multirotor.ex(3, :))
y = ylabel('$e_{p_{z}}$', 'rotation', 0, 'Interpreter', 'latex');
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
xlabel('$Time(sec)$', 'Interpreter', 'latex')

% plot velocity error
figure(3)
subplot(3, 1, 1)
plot(multirotor.t, multirotor.ev(1, :))
y = ylabel('$e_{v_{x}}$', 'rotation', 0, 'Interpreter', 'latex');
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
title('$Velocity$ $Error$ $(m/s)$', 'Interpreter', 'latex')
subplot(3, 1, 2)
plot(multirotor.t, multirotor.ev(2, :))
y = ylabel('$e_{v_{y}}$', 'rotation', 0, 'Interpreter', 'latex');
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
subplot(3, 1, 3)
plot(multirotor.t, multirotor.ev(3, :))
y = ylabel('$e_{v_{z}}$', 'rotation', 0, 'Interpreter', 'latex');
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
xlabel('$Time(sec)$', 'Interpreter', 'latex')

% plot attitude error
figure(4)
subplot(3, 1, 1)
plot(multirotor.t, multirotor.eR(1, :))
y = ylabel('$e_{R_{x}}$', 'rotation', 0, 'Interpreter', 'latex');
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
title('$Attitude$ $Error$ $(rad)$', 'Interpreter', 'latex')
subplot(3, 1, 2)
plot(multirotor.t, multirotor.eR(2, :))
y = ylabel('$e_{R_{y}}$', 'rotation', 0, 'Interpreter', 'latex');
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
subplot(3, 1, 3)
plot(multirotor.t, multirotor.eR(3, :))
y = ylabel('$e_{R_{z}}$', 'rotation', 0, 'Interpreter', 'latex');
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
xlabel('$Time(sec)$', 'Interpreter', 'latex')

% plot angular velocity error
figure(5)
subplot(3, 1, 1)
plot(multirotor.t, multirotor.eW(1, :))
y = ylabel('$e_{\Omega_{x}}$', 'rotation', 0, 'Interpreter', 'latex');
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
title('$Angular$ $Velocity$ $Error$ $(rad/s)$', 'Interpreter', 'latex')
subplot(3, 1, 2)
plot(multirotor.t, multirotor.eW(2, :))
y = ylabel('$e_{\Omega_{y}}$', 'rotation', 0, 'Interpreter', 'latex');
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
subplot(3, 1, 3)
plot(multirotor.t, multirotor.eW(3, :))
y = ylabel('$e_{\Omega_{z}}$', 'rotation', 0, 'Interpreter', 'latex');
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
xlabel('$Time(sec)$', 'Interpreter', 'latex')

figure(6)
plot(multirotor.t, multirotor.rotor_thrust(1, :))
hold on
plot(multirotor.t, multirotor.rotor_thrust(2, :))
hold on
plot(multirotor.t, multirotor.rotor_thrust(3, :))
hold on
plot(multirotor.t, multirotor.rotor_thrust(4, :))
y = ylabel('$f$', 'rotation', 0, 'Interpreter', 'latex');
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
xlabel('$Time(sec)$', 'Interpreter', 'latex')
legend('$f_{1}$', '$f_{2}$', '$f_{3}$', '$f_{4}$', 'Interpreter', 'latex')
title('$Rotor$ $Thrust$ $(N)$', 'Interpreter', 'latex')

figure(7)
subplot(211)
plot(multirotor.t, multirotor.force_moment(1, :))
y = ylabel('$f$', 'rotation', 0, 'Interpreter', 'latex');
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
title('$Total$ $thrust$ $(N)$', 'Interpreter', 'latex')

subplot(212)
plot(multirotor.t, multirotor.force_moment(2, :))
hold on
plot(multirotor.t, multirotor.force_moment(3, :))
hold on
plot(multirotor.t, multirotor.force_moment(4, :))
y = ylabel('$M$', 'rotation', 0, 'Interpreter', 'latex');
set(y, 'Units', 'Normalized', 'Position', [-0.09, 0.41])
xlabel('$Time(sec)$', 'Interpreter', 'latex')
legend('$M_{x}$', '$M_{y}$', '$M_{z}$', 'Interpreter', 'latex')
title('$Moment$ $Control$ $input$ $(N\cdot m)$', 'Interpreter', 'latex')
