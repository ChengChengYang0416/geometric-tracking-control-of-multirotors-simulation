function ned = vec_enu_to_ned(enu)
    ned = zeros(3, 1);
    ned(1) = enu(2);
    ned(2) = enu(1);
    ned(3) = -enu(3);
end
