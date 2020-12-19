% A simulation for geometric tracking control of multirotors
close all;

% simulation time
dt = 0.01;
sim_t = 10;
t = 0:dt:sim_t;
t = t';

% initialize parameters
multirotor = multirotor_dynamics;
multirotor.m = 4.34;
multirotor.J = [0.0820, 0, 0;
                0, 0.0845, 0;
                0, 0, 0.1377];
multirotor.d = 0.315;
multirotor.c_tau = 8.004e-4;

% initialize trajectory
tra = zeros(9, length(t));
traj = trajectory;
