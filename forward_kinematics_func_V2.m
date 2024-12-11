function T = forward_kinematics_func_V2()
syms q1 q2 q3 q4 real % Joint angles (in radians)
syms d1 d2 d3 d4 real % Link offsets (if any)
syms a1 a2 a3 a4 real % Link lengths
syms alpha1 alpha2 alpha3 alpha4 real % Link twists

DH_Table = [
    -1*q1, d1, a1, alpha1; 
    q2, d2, a2, alpha2; 
    q3, d3, a3, alpha3; 
    q4, d4, a4, alpha4
];

   % A = deg2rad(60);  % Joint 1 angle (degrees to radians)
   % B = deg2rad(60);  % Joint 2 angle (degrees to radians)
   % C = deg2rad(90);  % Joint 3 angle (degrees to radians)
   % D = deg2rad(40);  % Joint 4 angle (degrees to radians)

    L1 = 0.03388;  % Length of link 1
    L2 = 0.140;    % Length of link 2
    L3 = 0.1315;    % Length of link 3
    L4 = 0.068;     % Length of link 4

%% Compute Transformation Matrices for Each Link
T = eye(4); % Initialize the total transformation matrix as identity

for i = 1:4
    theta = DH_Table(i, 1); % Joint angle
    d = DH_Table(i, 2);     % Link offset
    a = DH_Table(i, 3);     % Link length
    alpha = DH_Table(i, 4); % Link twist

    % Transformation matrix for the current link
    Ti = [
        cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
        sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
        0,           sin(alpha),             cos(alpha),            d;
        0,           0,                      0,                     1
    ];

    % Update the total transformation matrix
    T = T * Ti;
end

%% Display the Total Transformation Matrix
%fprintf('Total Transformation Matrix T:\n');
%T = simplify(T); % Simplify the expression for clarity
%disp(T);

%% Example Usage
% Assign specific values to the DH parameters to compute numerical results.
% For example:
 %a1 = 0; a2 = -L2; a3 = -L3; a4 = L4;
 %alpha1 = pi/2; alpha2 = 0; alpha3 = 0; alpha4 = -pi;
 %d1 = L1; d2 = 0; d3 = 0; d4 = 0;
 %q1 = A; q2 = B; q3 = C; q4 = D;
 
    T = subs(T, d1, L1);
    T = subs(T, d2, 0);
    T = subs(T, d3, 0);
    T = subs(T, d4, 0);
    T = subs(T, a1, 0);
    T = subs(T, a2, -L2);
    T = subs(T, a3, -L3);
    T = subs(T, a4, L4);
    T = subs(T, alpha1, pi/2);
    T = subs(T, alpha2, 0);
    T = subs(T, alpha3, 0);
    T = subs(T, alpha4, -pi);
    %T = subs(T, q1, A);
    %T = subs(T, q2, B);
    %T = subs(T, q3, C);
    %T = subs(T, q4, D);
    %T = vpa(T);
    T = vpa(simplify(T));
    %disp(T);
 %{
% Example Usage
% Assign specific values to the DH parameters to compute numerical results.
% For example:
 a1 = 0; a2 = -L2; a3 = -L3; a4 = L4;
 alpha1 = pi/2; alpha2 = 0; alpha3 = 0; alpha4 = -pi;
 d1 = L1; d2 = 0; d3 = 0; d4 = 0;
 q1 = A; q2 = B; q3 = C; q4 = D;

 %Substitute these values into the matrix:
 T_numeric = double(subs(T));
 disp(T_numeric);
%}
end