% Ensure any existing Arduino connection is cleared
if exist('a', 'var') && isa(a, 'arduino')
    delete(a);  % Explicitly delete the Arduino object
    clear a;    % Clear the variable from the workspace
end

a = arduino();  % Connect to Arduino
servos = [servo(a, 'D9'), servo(a, 'D10'), servo(a, 'D11'), servo(a, 'D12')]; % Define servo objects for each joint
servo_5 = servo(a, 'D13');  % Define the 5th servo on pin D5

writePosition(servo_5, 1);  % EE is initially open

% Perform trajectory_pickup
for i = 1:size(trajectory_pickup, 1)  % Iterate through rows of the trajectory_pickup array
    joint_angles = trajectory_pickup(i, :);  % Get joint angles for this step (in degrees)
    
    %joint_angles(4) = joint_angles(4) + 20; %Compensate q4's offset in real life
     joint_angles(2) = joint_angles(2) -90; 
     joint_angles(3) = joint_angles(3) +90; 


    % Scale joint angles from degrees to servo-compatible range [0, 1]
    %servo_positions = (joint_angles + 90) / 180;  % Map [-180°, 180°] to [0, 1]
    servo_positions = max(0, min(1, (joint_angles + 90) / 180));


    % Send positions to servos
    for j = 1:length(servos)
        writePosition(servos(j), servo_positions(j));
    end
    
    % Debugging: Print joint angles and servo positions
    fprintf('Step %d: Joint Angles [%.2f, %.2f, %.2f, %.2f] (degrees)\n', i, joint_angles);
    fprintf('Servo Positions: [%.2f, %.2f, %.2f, %.2f] (normalized)\n', servo_positions);
    
    pause(0.05);  % Pause to match your sampling time
end


writePosition(servo_5, 0);  
pause(2);  
% Perform trajectory_dropoff
for i = 1:size(trajectory_dropoff, 1)  % Iterate through rows of the trajectory_dropoff array
    joint_angles = trajectory_dropoff(i, :);  % Get joint angles for this step (in degrees)

    %joint_angles(4) = joint_angles(4) + 20; %Compensate q4's offset in real life
    joint_angles(2) = joint_angles(2) -90; 
    joint_angles(3) = joint_angles(3) +90; 

    % Scale joint angles from degrees to servo-compatible range [0, 1]
    %servo_positions = (joint_angles + 90) / 180;  % Map [-180°, 180°] to [0, 1]
    servo_positions = max(0, min(1, (joint_angles + 90) / 180));


    % Send positions to servos
    for j = 1:length(servos)
        writePosition(servos(j), servo_positions(j));
    end
    
    fprintf('Step %d: Joint Angles [%.2f, %.2f, %.2f, %.2f] (degrees)\n', i, joint_angles);
    fprintf('Servo Positions: [%.2f, %.2f, %.2f, %.2f] (normalized)\n', servo_positions);
    
    pause(0.05);  % Pause to match your sampling time
end

% Set the 5th servo to 180 degrees and wait for 2 seconds
writePosition(servo_5, 1);  % Map 180° to normalized value 1
pause(2);  % Wait for 2 seconds

% Clean up resources
clear a s