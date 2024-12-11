function Joint_Space = joint_trajectory(q0, qf, qdot0, qdotf, Tf, Ts)
    % Define time vector
    t = 0:Ts:Tf;

    % Number of joints
    nJoints = length(q0);

    % Preallocate for joint positions
    Joint_Space = zeros(length(t), nJoints);

    % Coefficients for cubic polynomial interpolation
    for i = 1:nJoints
        A = [1, 0,    0,       0;       % q(0) = q0
             1, Tf,   Tf^2,    Tf^3;    % q(Tf) = qf
             0, 1,    0,       0;       % qdot(0) = qdot0
             0, 1,    2*Tf,    3*Tf^2]; % qdot(Tf) = qdotf
        b = [q0(i); qf(i); qdot0(i); qdotf(i)];
        coeffs = A\b;                   % Solve for coefficients [a0; a1; a2; a3]

        for j = 1:length(t)
            Joint_Space(j, i) = coeffs(1) + coeffs(2)*t(j) + coeffs(3)*t(j)^2 + coeffs(4)*t(j)^3;
        end
    end
end
