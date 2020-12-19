classdef trajectory
   methods
       function out = traj_generate(~, t)
           % xd, vd, b1d
           out = zeros(9, 1);
           % xd
           out(1) = 0.4*t;
           out(2) = 0.4*sin(pi*t);
           out(3) = 0.6*cos(pi*t);
           % vd
           out(4) = 0.4;
           out(5) = 0.4*pi*cos(pi*t);
           out(6) = -0.6*pi*sin(pi*t);
           % b1d
           out(7) = 1;
           out(8) = 0;
           out(9) = 0;
       end
   end
end
