clc;
clear;

q0_initial = deg2rad([0;0;0;0]);
q0_final = deg2rad([90;25;45;60]);

%X_initial = [0.27; 0.0035; 0.093]; %Trajectory Initial Position
%X_final = [-0.0035; 0.2504; 0.1708]; %Trajectory Final Position
X_initial = [0.2035;0;0.0339];
X_final = [0; 0.2156; -0.0968]; 

disp('Initial qs at starting position:')
q_0 = inverse_kinematics_func_final(q0_initial, X_initial); 
disp('Final qs at end position:')
q_f = inverse_kinematics_func_final(q0_final, X_final);


q_dot_initial = [0; 0; 0; 0];
q_dot_final = [0; 0; 0; 0];

Ts = 0.2; % Sampling Time 
Tf =  7 ; %Duration of trajectory

trajectory_dropoff = joint_trajectory(q_0, q_f, q_dot_initial, q_dot_final, Tf, Ts);
disp(['------------' ...
    '-----------------------------']);
disp('            Trajectory Table             ');
disp('-----------------------------------------');
disp('      q1        q2       q3        q4'    );
disp('-----------------------------------------');
disp(trajectory_dropoff);

syms q1 q2 q3 q4 


T = forward_kinematics_func_V2();
end_effector_position = T(1:3, 4);
X = end_effector_position;
pos_values = zeros(length(trajectory_dropoff), 3); 

for j = 1:length(trajectory_dropoff)
    q_i = trajectory_dropoff(j, :);  
    q_i = q_i * (pi/180);  
    pos_values(j, :) = double(subs(X, {q1, q2, q3, q4}, q_i));  
end

disp('-----------------------------------------');
disp('     Position Table For Validation');
disp('-----------------------------------------');
disp('       X        Y        Z'    );
disp('-----------------------------------------');
disp(pos_values);

% Time vector for the trajectory
time_vector = (0:Ts:Tf)';  % Column vector for time

% Ensure the number of time steps matches the trajectory length
if length(time_vector) ~= size(trajectory_dropoff, 1)
    error('Time vector length and trajectory points mismatch. Check Ts and Tf values.');
end

% Separate each joint angle with time
q1_over_time = [time_vector, trajectory_dropoff(:, 1)];
q2_over_time = [time_vector, trajectory_dropoff(:, 2)];
q3_over_time = [time_vector, trajectory_dropoff(:, 3)];
q4_over_time = [time_vector, trajectory_dropoff(:, 4)];

% Save the joint angles as variables
assignin('base', 'q1_over_time', q1_over_time);
assignin('base', 'q2_over_time', q2_over_time);
assignin('base', 'q3_over_time', q3_over_time);
assignin('base', 'q4_over_time', q4_over_time);

% Display confirmation
disp('Joint angles over time have been prepared for Simulink:');
disp('Variables q1_over_time, q2_over_time, q3_over_time, and q4_over_time are in the base workspace.');
