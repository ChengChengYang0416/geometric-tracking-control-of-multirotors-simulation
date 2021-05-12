function M = cal_allocation_matrix(d, c_tau)
    M = [     1,     1,      1,     1;
              0,    -d,      0,     d;
              d,     0,     -d,     0;
         -c_tau, c_tau, -c_tau, c_tau];
end
