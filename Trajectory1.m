clc;
clear;

%Initial Guesses for Inverse Position Kinematics 
%q0_initial = deg2rad([90;25;45;60]);
%q0_final = deg2rad([150;45;80;60]);
q0_initial = deg2rad([100;50;60;90]);
q0_final = deg2rad([90;10;20;20]);

%X_initial = [-0.0035; 0.2504; 0.1078]; %Trajectory Initial Position
%X_final = [-0.1807; 0.1003; 0.1172]; %Trajectory Final Position

X_initial = [0; 0.2156; -0.0968]; %Trajectory Initial Position
X_final = [0.0791; 0.0457; -0.1788]; %Trajectory Final Position


disp('Initial qs at starting position:')
q_0 = inverse_kinematics_func_final(q0_initial, X_initial); 
disp('Final qs at end position:')
q_f = inverse_kinematics_func_final(q0_final, X_final); 

q_dot_initial = [0; 0; 0; 0];
q_dot_final = [0; 0; 0; 0];

Ts = 0.2; 
Tf = 7 ; 

trajectory_pickup = joint_trajectory(q_0, q_f, q_dot_initial, q_dot_final, Tf, Ts);
disp(['------------' ...
    '-----------------------------']);
disp('            Trajectory Table             ');
disp('-----------------------------------------');
disp('      q1        q2       q3        q4'    );
disp('-----------------------------------------');
disp(trajectory_pickup);

syms q1 q2 q3 q4 

T = forward_kinematics_func_V2();
end_effector_position = T(1:3, 4);
X_pos = end_effector_position;
pos_values = zeros(length(trajectory_pickup), 3); 

for j = 1:length(trajectory_pickup)
    q_i = trajectory_pickup(j, :);  
    q_i = q_i * (pi/180);  
    pos_values(j, :) = double(subs(X_pos, {q1, q2, q3, q4}, q_i));  
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
if length(time_vector) ~= size(trajectory_pickup, 1)
   error('Time vector length and trajectory points mismatch. Check Ts and Tf values.');
end

% Separate each joint angle with time
q1_over_time = [time_vector, trajectory_pickup(:, 1)];
q2_over_time = [time_vector, trajectory_pickup(:, 2)];
q3_over_time = [time_vector, trajectory_pickup(:, 3)];
q4_over_time = [time_vector, trajectory_pickup(:, 4)];

% Save the joint angles as variables
assignin('base', 'q1_over_time', q1_over_time);
assignin('base', 'q2_over_time', q2_over_time);
assignin('base', 'q3_over_time', q3_over_time);
assignin('base', 'q4_over_time', q4_over_time);

% Display confirmation
disp('Joint angles over time have been prepared for Simulink:');
disp('Variables q1_over_time, q2_over_time, q3_over_time, and q4_over_time are in the base workspace.');