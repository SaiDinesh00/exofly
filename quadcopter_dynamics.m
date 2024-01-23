function state_dot = quadcopter_dynamics(t, state, mass, gravity,inertia, omega, k, b, l)
%% Unpacking variables
    x = state(1);
    y = state(2);
    z = state(3);
    u = state(4);
    v = state(5);
    w = state(6);
    phi = state(7);
    theta = state(8);
    psi = state(9);
    p = state(10);
    q = state(11);
    r = state(12);

    thrust = k * (omega(1)^2 + omega(2)^2 + omega(3)^2 + omega(4)^2);
    tau = [l * k * (omega(1)^2 - omega(3)^2);
            l * k * (omega(2)^2 - omega(4)^2);
            b * ((omega(1)^2)-(omega(2)^2)+(omega(3)^2)-(omega(4)^2))];
    

    u_dot = gravity*sin(theta) - w*q + v*r;
    v_dot = -gravity*cos(theta) * sin(phi) - u*r + w*p;
    w_dot = -gravity*cos(theta) * cos(phi) - v*p + u*q + (1/mass) * thrust;
    p_dot = (1 / inertia(1)) * (tau(1) - (inertia(3) - inertia(2)) * q * r);
    q_dot = (1 / inertia(2)) * (tau(2) - (inertia(1) - inertia(3)) * p * r);
    r_dot = (1 / inertia(3)) * (tau(3) - (inertia(2) - inertia(1)) * q * p);

    phi_dot = p + (q * sin(phi) + r * cos(phi)) * tan(theta);
    theta_dot = q * cos(phi) - r * sin(phi);
    % Check for division by zero in psi_dot
    if abs(cos(theta)) > 1e-6
        psi_dot = (q * sin(phi) + r * cos(phi)) / cos(theta);
    else
        psi_dot = 0;  % Set to zero or handle it appropriately
    end
    

    state_dot = [u; v; w; u_dot; v_dot; w_dot; phi_dot; theta_dot; psi_dot;p_dot; q_dot; r_dot];

end