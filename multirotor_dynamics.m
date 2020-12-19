classdef multirotor_dynamics
    properties
        % simulation time
        dt = 0.01;
        sim_t = 10;
        t
        iter
        % parameters
        m
        J
        d
        c_tau
        g = 9.81;
        % unit vector
        e1 = [1; 0; 0];
        e2 = [0; 1; 0];
        e3 = [0; 0; 1];
        % states
        x
        v
        R
        W
        % errors
        ex
        ev
        eR
        eW
    end
    methods
        function Dynamics = dynamics(obj)
            Dynamics = obj.m;
        end
    end
end
