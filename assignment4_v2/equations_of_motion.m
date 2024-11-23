% equations_of_motion.m

function dy = equations_of_motion(t, y, params)
    % Unpack parameters
    L1 = params.L1; L2 = params.L2; L3 = params.L3;
    m1 = params.m1; m2 = params.m2; m3 = params.m3;
    g = params.g;
    
    % Unpack state variables
    q1 = y(1); q2 = y(2); q3 = y(3);
    dq1 = y(4); dq2 = y(5); dq3 = y(6);
    
    % Compute accelerations
    ddq1 = compute_ddq1(q1, q2, q3, dq1, dq2, dq3, L1, L2, L3, m1, m2, m3, g);
    ddq2 = compute_ddq2(q1, q2, q3, dq1, dq2, dq3, L1, L2, L3, m1, m2, m3, g);
    ddq3 = compute_ddq3(q1, q2, q3, dq1, dq2, dq3, L1, L2, L3, m1, m2, m3, g);
    
    % Return derivatives
    dy = zeros(6,1);
    dy(1) = dq1;
    dy(2) = dq2;
    dy(3) = dq3;
    dy(4) = ddq1;
    dy(5) = ddq2;
    dy(6) = ddq3;
end
