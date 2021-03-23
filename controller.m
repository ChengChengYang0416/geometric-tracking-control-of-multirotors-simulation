classdef controller
   methods
       function [control, error] = geometric_tracking_ctrl(obj, iter, multirotor, tra)
           % f, M
           control = zeros(16, 1);
           
           % xd, vd, ad, b1d, Wd
           xd = tra(1:3, iter-1);
           vd = tra(4:6, iter-1);
           ad = tra(7:9, iter-1);
           b1d = tra(10:12, iter);
           Wd = [0; 0; 0];
           
           % now states
           x = multirotor.x(:, iter-1);
           v = multirotor.v(:, iter-1);
           R = reshape(multirotor.R(:, iter-1), 3, 3);
           W = multirotor.W(:, iter-1);
           e3 = multirotor.e3;
           
           % control gains
           kx = diag([16*multirotor.m; 16*multirotor.m; 16*multirotor.m]);
           kv = diag([5.6*multirotor.m; 5.6*multirotor.m; 5.6*multirotor.m]);
           kR = 150;
           kW = 2.54;
           
           % error
           ex = x - xd;
           ev = v - vd;
           
           % f
           A = (-kx*ex - kv*ev + multirotor.m*ad + multirotor.m*multirotor.g*e3);
           b3 = R*e3;
           f = vec_dot(A, b3);
           
           % Rd
           norm_A = norm(A);
           b3d = A/norm_A;
           b2d = vec_cross(b3d, b1d);
           norm_b2d = norm(b2d);
           b2d = b2d/norm_b2d;
           b1d_proj = vec_cross(b2d, b3d);
           Rd = [b1d_proj b2d b3d];

           % eR and eW
           eR = 1/2*vee_map(Rd'*R - R'*Rd);
           eW = W - R'*Rd*Wd;

           % M
           M = -kR*eR - kW*eW + vec_cross(W, multirotor.J*W);

           % f, M
           control(1) = f;
           control(2) = M(1);
           control(3) = M(2);
           control(4) = M(3);
           
           % ex, ev, eR, eW
           error(1:3) = ex;
           error(4:6) = ev;
           error(7:9) = eR;
           error(10:12) = eW;
       end
   end
end
