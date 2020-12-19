% A simulation for geometric tracking control of multirotors
close all;

% simulation time
dt = 0.01;
sim_t = 10;

% initialize parameters
multirotor = multirotor_dynamics;
multirotor.dt = dt;
multirotor.sim_t = sim_t;
multirotor.t = 0:dt:sim_t;
multirotor.m = 4.34;
multirotor.J = [0.0820, 0, 0;
                0, 0.0845, 0;
                0, 0, 0.1377];
multirotor.d = 0.315;
multirotor.c_tau = 8.004e-4;
multirotor.x = zeros(3, length(multirotor.t));
multirotor.v = zeros(3, length(multirotor.t));
multirotor.R = zeros(9, length(multirotor.t));
multirotor.W = zeros(3, length(multirotor.t));
multirotor.ex = zeros(3, length(multirotor.t));
multirotor.ev = zeros(3, length(multirotor.t));
multirotor.eR = zeros(3, length(multirotor.t));
multirotor.eW = zeros(3, length(multirotor.t));
multirotor.R(1:9) = [1; 0; 0; 0; 1; 0 ;0 ; 0; 1];

% initialize trajectory
tra = zeros(9, length(multirotor.t));
traj = trajectory;
