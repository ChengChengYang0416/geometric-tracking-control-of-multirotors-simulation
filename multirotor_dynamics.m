classdef multirotor_dynamics
    properties
        m
        J
        d
        c_tau
        g = 9.81;
        e1 = [1; 0; 0];
        e2 = [0; 1; 0];
        e3 = [0; 0; 1];
    end
    methods
        function Dynamics = dynamics(obj)
            Dynamics = obj.m;
        end
    end
end
