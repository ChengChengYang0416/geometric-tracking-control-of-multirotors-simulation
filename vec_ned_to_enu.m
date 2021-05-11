function enu = vec_ned_to_enu(ned)
    enu = zeros(3, 1);
    enu(1) = ned(2);
    enu(2) = ned(1);
    enu(3) = -ned(3);
end
