import math
def sysCall_init():
    sim = require('sim')
    
    # Get handles for the joints of links 1, 2, and 3
    self.q1_handle = sim.getObject("../1")
    self.q2_handle = sim.getObject("../2")
    self.q3_handle = sim.getObject("../3")
    self.q4_handle = sim.getObject("../4")
    # Set the target positions for each joint in radians
    self.target_position1 = 90 * (3.14 / 180)  # 90 degrees for link 1
    self.target_position2 = 45 * (3.14 / 180)  # 45 degrees for link 2 (you can modify as needed)
    self.target_position3 = 60 * (3.14 / 180)  # 30 degrees for link 3 (you can modify as needed)
    self.target_position4 = 90 * (3.14 / 180)
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
    inverse_kinematics_func(np.array([1.6, 0.8, 1.1, 1.6]), np.array([178.6628, 0, 49.051]))
    velocity_kinematics()
    # Print the end effector position
    print(Eposition)
    pass
    
import numpy as np

def inverse_kinematics_func(q0, X):
    """
    Calculates inverse kinematics for a 4-DOF manipulator.
    
    Args:
    - q0 (numpy array): Initial guess for joint angles [q1, q2, q3, q4]
    - X (numpy array): Desired end-effector position [x, y, z]

    Returns:
    - theta_1, theta_2, theta_3, theta_4 (float): Calculated joint angles to achieve desired position
    """
    
    # Link lengths
    L1 = 33.88
    L2 = 115
    L3 = 119
    L4 = 68

    # Define initial joint values
    q_old = np.array(q0)

    # Parameters for the iterative solver
    err_toler = 1e-4  # Convergence tolerance
    max_iters = 100   # Maximum number of iterations
    error = 1         # Initial error
    i = 0             # Iteration counter

    # Iterative inverse kinematics solver
    while error > err_toler and i < max_iters:
        q1, q2, q3, q4 = q_old

        # Calculate forward kinematics (numerically)
        T = forward_kinematics(q1, q2, q3, q4, L1, L2, L3, L4)
        W = T[:3, 3]  # Extract end-effector position [x; y; z]

        # Calculate the error in position
        F_Qn = W - X
        error = np.linalg.norm(F_Qn)

        # Calculate the Jacobian matrix (numerically)
        J = calculate_jacobian(q1, q2, q3, q4, L1, L2, L3, L4)

        # Calculate the pseudo-inverse of the Jacobian matrix
        J_pseudo_inv = np.dot(J.T, np.linalg.inv(np.dot(J, J.T)))

        # Update joint angles
        q_new = q_old - np.dot(J_pseudo_inv, F_Qn)
        q_old = q_new  # Set the new angles as the old for the next iteration

        i += 1
    
    # Return the final joint angles
    #theta_1, theta_2, theta_3, theta_4 = q_old
    #return theta_1, theta_2, theta_3, theta_4
    print("Inverse Kinematics", q_old)
    return q_old
    

def forward_kinematics(q1, q2, q3, q4, L1, L2, L3, L4):
    """
    Forward kinematics of the manipulator.
    
    Args:
    - q1, q2, q3, q4 (float): Joint angles in radians
    - L1, L2, L3, L4 (float): Link lengths
    
    Returns:
    - T (numpy array): The transformation matrix representing the position of the end-effector
    """
    
    # Define the transformation matrices for each joint
    A = np.array([[np.cos(q1 + np.pi/2), 0, -np.sin(q1 + np.pi/2), 0],
                  [np.sin(q1 + np.pi/2), 0, np.cos(q1 + np.pi/2), 0],
                  [0, -1, 0, L1],
                  [0, 0, 0, 1]])

    B = np.array([[np.cos(q2 - np.pi/2), -np.sin(q2 - np.pi/2), 0, -L2 * np.cos(q2 - np.pi/2)],
                  [np.sin(q2 - np.pi/2), np.cos(q2 - np.pi/2), 0, -L2 * np.sin(q2 - np.pi/2)],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

    C = np.array([[np.cos(q3), -np.sin(q3), 0, -L3 * np.cos(q3)],
                  [np.sin(q3), np.cos(q3), 0, -L3 * np.sin(q3)],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

    D = np.array([[np.cos(q4), -np.sin(q4), 0, -L4 * np.cos(q4)],
                  [np.sin(q4), np.cos(q4), 0, -L4 * np.sin(q4)],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

    # Compute the transformation matrix
    T = np.dot(np.dot(np.dot(A, B), C), D)
    return T


def calculate_jacobian(q1, q2, q3, q4, L1, L2, L3, L4):
    """
    Calculate the Jacobian matrix using finite differences.
    
    Args:
    - q1, q2, q3, q4 (float): Joint angles in radians
    - L1, L2, L3, L4 (float): Link lengths
    
    Returns:
    - J (numpy array): The Jacobian matrix
    """
    
    delta = 1e-6  # Small change for numerical differentiation

    # Initial transformation matrix and end-effector position
    T = forward_kinematics(q1, q2, q3, q4, L1, L2, L3, L4)
    W0 = T[:3, 3]

    # Preallocate Jacobian matrix
    J = np.zeros((3, 4))

    # Differentiate with respect to q1
    T1 = forward_kinematics(q1 + delta, q2, q3, q4, L1, L2, L3, L4)
    J[:, 0] = (T1[:3, 3] - W0) / delta

    # Differentiate with respect to q2
    T2 = forward_kinematics(q1, q2 + delta, q3, q4, L1, L2, L3, L4)
    J[:, 1] = (T2[:3, 3] - W0) / delta

    # Differentiate with respect to q3
    T3 = forward_kinematics(q1, q2, q3 + delta, q4, L1, L2, L3, L4)
    J[:, 2] = (T3[:3, 3] - W0) / delta

    # Differentiate with respect to q4
    T4 = forward_kinematics(q1, q2, q3, q4 + delta, L1, L2, L3, L4)
    J[:, 3] = (T4[:3, 3] - W0) / delta

    return J
    
    import math

def velocity_kinematics():
    # Define symbolic variables (substitute with numeric values)
    q1_dot = 0.1
    q2_dot = 0.3
    q3_dot = 0.4
    q4_dot = 0.2

    q_dot = [q1_dot, q2_dot, q3_dot, q4_dot]

    # Define time variable
    tn = 0

    # Assume the Differentiation function gives the Jacobian matrix (J_matrix)
    # In your case, replace this with the actual Jacobian function or matrix
    J_matrix = differentiation()

    # Loop for velocity kinematics calculation over time
    while tn < 10:
        # Calculate p1, p2, p3, p4 at time tn
        p1 = 0.1 * tn
        p2 = 0.3 * tn
        p3 = 0.4 * tn
        p4 = 0.2 * tn

        # Substitute values into the Jacobian matrix (numerically)
        # Here we assume Differentiation returns a matrix that is evaluated at p1, p2, p3, p4
        J_matrix_sub = subs(J_matrix, [p1, p2, p3, p4])

        # Calculate the velocity vector V
        V = matrix_multiply(J_matrix_sub, q_dot)

        # Print the velocity
        print("Velocity at time", tn, ":", V)

        # Increment time step
        tn += 0.1

def differentiation():
    # This function should return the Jacobian matrix (numerically computed)
    # You need to define the Jacobian matrix for your system here.
    # For example, using placeholder values:
    J_matrix = [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0]
    ]
    return J_matrix

def subs(J_matrix, values):
    # Substitute the values into the Jacobian matrix
    # Since this is a numerical example, no symbolic substitution is needed.
    # In a real system, substitute the variables with actual values.
    return J_matrix  # For this example, no actual substitution occurs.

def matrix_multiply(A, B):
    # Matrix multiplication function
    # Multiply a 3x4 matrix by a 4x1 vector
    result = [sum(A[i][j] * B[j] for j in range(len(B))) for i in range(len(A))]
    return result

# Call the velocity_kinematics function to execute the kinematics calculation


