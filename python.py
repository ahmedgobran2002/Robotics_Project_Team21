def sysCall_init():
    sim = require('sim')
    
    # Get handles for the joints of links 1, 2, and 3
    self.q1_handle = sim.getObject("../1")
    self.q2_handle = sim.getObject("../2")
    self.q3_handle = sim.getObject("../3")
    self.q4_handle = sim.getObject("../4")
    # Set the target positions for each joint in radians
    self.target_position1 = 0 * (3.14 / 180)  # 90 degrees for link 1
    self.target_position2 = 30 * (3.14 / 180)  # 45 degrees for link 2 (you can modify as needed)
    self.target_position3 = 60 * (3.14 / 180)  # 30 degrees for link 3 (you can modify as needed)
    self.target_position4 = 45 * (3.14 / 180)
    self.required_duration = 5  # Duration to complete the rotation

def sysCall_actuation():
    c_t = sim.getSimulationTime()  # Get current simulation time

    # Calculate the new position for each joint
    if c_t < self.required_duration:
        new_position1 = (c_t / self.required_duration) * self.target_position1
        new_position2 = (c_t / self.required_duration) * self.target_position2
        new_position3 = (c_t / self.required_duration) * self.target_position3
        new_position4 = (c_t / self.required_duration) * self.target_position4
        # Set the new position for each joint
        sim.setJointTargetPosition(self.q1_handle, new_position1)
        sim.setJointTargetPosition(self.q2_handle, new_position2)
        sim.setJointTargetPosition(self.q3_handle, new_position3)
        sim.setJointTargetPosition(self.q4_handle, new_position4)
    else:
        # Set joints to final target positions
        sim.setJointTargetPosition(self.q1_handle, self.target_position1)
        sim.setJointTargetPosition(self.q2_handle, self.target_position2)
        sim.setJointTargetPosition(self.q3_handle, self.target_position3)
        sim.setJointTargetPosition(self.q4_handle, self.target_position4)
def sysCall_sensing():
    pass

def sysCall_cleanup():
    # Get position of end effector at cleanup
    EHandle = sim.getObject("../Grip_respondable")
    base_handle = sim.getObject("../base_link_visual/ReferenceFrame")
    Eposition = sim.getObjectPosition(EHandle, base_handle)
    
    # Print the end effector position
    print(Eposition)
    pass
    
import numpy as np
import math
import matplotlib.pyplot as plt

# Function to calculate inverse kinematics for a 4-DOF robot
def inverse_kinematics(q0, X):
    L1 = 100  # Link 1 length
    L2 = 100  # Link 2 length
    L3 = 50   # Link 3 length
    
    # Initialize joint angles with the guess
    q = np.array(q0)

    # Extract desired position
    x_des, y_des, z_des = X

    # Calculate base joint (q1)
    q[0] = math.atan2(y_des, x_des)

    r = math.sqrt(x_des**2 + y_des**2)  # Planar distance

    # Calculate D for elbow joint (q3)
    D = (r**2 + (z_des - L1)**2 - L2**2 - L3**2) / (2 * L2 * L3)

    # Ensure D is in the range [-1, 1] for valid atan2 computation
    D = max(min(D, 1), -1)

    # Elbow joint (q3)
    q[2] = math.atan2(math.sqrt(1 - D**2), D)

    # Shoulder joint (q2)
    q[1] = math.atan2(z_des - L1, r) - math.atan2(L3 * math.sin(q[2]), L2 + L3 * math.cos(q[2]))

    # Wrist joint (q4) - assuming aligned with end-effector frame
    q[3] = 0

    return q

# Function to generate trajectory in task space
def task_trajectory(X0, Xf, Tf, Ts):
    # Time vector
    timeSteps = np.arange(0, Tf + Ts, Ts)
    
    joint_angles = []
    trajectory = []

    # Generate trajectory and calculate joint angles using inverse kinematics
    for t in timeSteps:
        t_norm = t / Tf

        # Calculate end-effector position at time t
        X = (1 - t_norm) * X0 + t_norm * Xf
        trajectory.append(X)

        # Get joint angles using inverse kinematics
        q = inverse_kinematics([90, 45, 60, 90], X)
        joint_angles.append(q)

    return trajectory, joint_angles, timeSteps

# Main function to simulate the trajectory and plot results
def main():
    # Initial and final end-effector positions
    X0 = np.array([180, 0, 45])   # Initial position
    Xf = np.array([0, 25, 2])     # Final position
    Tf = 5                        # Total time in seconds
    Ts = 0.1                      # Sampling time in seconds

    # Generate trajectory and compute joint angles
    trajectory, joint_angles, timeSteps = task_trajectory(X0, Xf, Tf, Ts)

    # Display the results
    print("Trajectory (X, Y, Z) at each time step:")
    for t, traj in zip(timeSteps, trajectory):
        print(f"Time {t:.2f} s: X = {traj}")

    print("Joint Angles (q1, q2, q3, q4) at each time step:")
    for t, q in zip(timeSteps, joint_angles):
        print(f"Time {t:.2f} s: q = {np.rad2deg(q)}")

    # Plotting the trajectory
    plt.figure()
    plt.plot(timeSteps, [traj[0] for traj in trajectory], 'r', label='X')
    plt.plot(timeSteps, [traj[1] for traj in trajectory], 'g', label='Y')
    plt.plot(timeSteps, [traj[2] for traj in trajectory], 'b', label='Z')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (mm)')
    plt.title('End-Effector Trajectory (X, Y, Z)')
    plt.legend()
    plt.grid(True)
    plt.show()

    # Plotting joint angles
    plt.figure()
    joint_angles_deg = np.rad2deg(joint_angles)
    for i in range(4):
        plt.plot(timeSteps, joint_angles_deg[:, i], label=f'q{i+1}')
    plt.xlabel('Time (s)')
    plt.ylabel('Joint Angles (degrees)')
    plt.title('Joint Angles Trajectory')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()

