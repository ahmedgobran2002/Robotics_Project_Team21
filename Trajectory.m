clear;
clc;

q0_initial = deg2rad([0;25;45;60]);
q0_final = deg2rad([0;45;80;60]);

X_initial = [0.011; 0.19; 0.0339]; %Trajectory Initial Position
X_final = [0.9; 0.457; -0.1788]; %Trajectory Final Position

disp('Initial qs at starting position:')
q_0 = inverse_kinematics_func_final(q0_initial, X_initial); % Actual Q's
disp('Final qs at end position:')
q_f = inverse_kinematics_func_final(q0_final, X_final); % Actual Q's

q_dot_initial = [0; 0; 0; 0];
q_dot_final = [0; 0; 0; 0];

Ts = 0.2; % Sampling Time 
Tf = 7; % Duration of trajectory

% Generate Pickup Trajectory
trajectory_pickup = joint_trajectory(q_0, q_f, q_dot_initial, q_dot_final, Tf, Ts);
disp('-----------------------------------------');
disp('            Pickup Trajectory            ');
disp('-----------------------------------------');
disp('      q1        q2       q3        q4');
disp('-----------------------------------------');
disp(trajectory_pickup);


q0_initial_dropoff = deg2rad([90;60;90;40]);
q0_final_dropoff = deg2rad([90;25;45;60]);
 
X_initial_dropoff = [0.2035; 0; 0.0339]; 
X_final_dropoff = [0; 0.2156; -0.0968]; 

disp('Initial qs at starting position (Dropoff):')
q_0_dropoff = inverse_kinematics_func_final(q0_initial_dropoff, X_initial_dropoff); % Actual Q's
disp('Final qs at end position (Dropoff):')
q_f_dropoff = inverse_kinematics_func_final(q0_final_dropoff, X_final_dropoff); % Actual Q's

% Generate Dropoff Trajectory
trajectory_dropoff = joint_trajectory(q_0_dropoff, q_f_dropoff, q_dot_initial, q_dot_final, Tf, Ts);
disp('-----------------------------------------');
disp('            Dropoff Trajectory             ');
disp('-----------------------------------------');
disp('      q1        q2       q3        q4');
disp('-----------------------------------------');
disp(trajectory_dropoff);

% Combine Pickup and Dropoff Trajectories
full_trajectory = [trajectory_pickup; trajectory_dropoff];

disp('-----------------------------------------');
disp('            Combined Trajectory (Total)    ');
disp('-----------------------------------------');
disp('      q1        q2       q3        q4');
disp('-----------------------------------------');
disp(full_trajectory);

% Check the total length of the combined trajectory
num_points_combined = size(full_trajectory, 1);

% Time vector for the combined trajectory, ensuring it matches the length
time_vector_combined = linspace(0, Tf, num_points_combined)';  % Generate time vector matching the number of trajectory points

% Ensure the number of time steps matches the combined trajectory length
if length(time_vector_combined) ~= num_points_combined
   error('Time vector length and combined trajectory points mismatch. Check Ts and Tf values.');
end

% Separate each joint angle with time for the combined trajectory
q1_over_time_combined = [time_vector_combined, full_trajectory(:, 1)];
q2_over_time_combined = [time_vector_combined, full_trajectory(:, 2)];
q3_over_time_combined = [time_vector_combined, full_trajectory(:, 3)];
q4_over_time_combined = [time_vector_combined, full_trajectory(:, 4)];

% Save the joint angles as variables for the combined trajectory
assignin('base', 'q1_over_time_combined', q1_over_time_combined);
assignin('base', 'q2_over_time_combined', q2_over_time_combined);
assignin('base', 'q3_over_time_combined', q3_over_time_combined);
assignin('base', 'q4_over_time_combined', q4_over_time_combined);

% Display confirmation
disp('Combined joint angles over time have been prepared for Simulink:');
disp('Variables q1_over_time_combined, q2_over_time_combined, q3_over_time_combined, and q4_over_time_combined are in the base workspace.');