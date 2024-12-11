function q = inverse_kinematics_func_final(q0, X)
   
    %J_pseudo_inv = J_inv();
    T = forward_kinematics_func_V2();
    J_Matrix = Differentiation();
    %end_effector_position = T(1:3, 4);
    f1 = T(1, 4); %X.E.E
    f2 = T(2, 4); %Y.E.E
    f3 = T(3, 4); %Z.E.E

    % Define symbolic variables
    syms L1 L2 L3 L4 q1 q2 q3 q4

    % Define constants for joints angles
    %q1_num = deg2rad(90);  % Joint 1 angle (degrees to radians)
    %q2_num = deg2rad(45);  % Joint 2 angle (degrees to radians)
    %q3_num = deg2rad(60);  % Joint 3 angle (degrees to radians)
    %q4_num = deg2rad(90);  % Joint 4 angle (degrees to radians)

    % End-effector position matrix in matrix form
    %x_desired = [178.6628; 0; 49.051];
    %x_desired = [0; -240; -50];
    %x_desired = [0; 0.2156; -0.0968];
    x_desired = X;
    %x_desired = [-0.1807;0.1003;0.1172];
    %disp(x_desired);
    
    %q0 = q;
    q0 = deg2rad([90;25;45;60]);
    % 1.57 - 0.43 - 0.78 - 1.04

    % Define F(Qn) 
    W = [f1; f2; f3];
    %disp(W);
    F_Qn = W - x_desired;

    % Define the joint variables as a vector
    % q_new = [q1; q2; q3; q4];
    
    %random_matrix1 = pi *rand(4, 1);
    %q_old = random_matrix1;
    q_old = [1; 0.8; 1.2; 1]; 
    %q_old = deg2rad([95;30;50;70]);

    % Define parameters:
    i = 0;
    error = [1;1;1;1];
    err_toler = 1e-8;   % Convergence tolerance
    %err_toler = 0.5;
    %err_toler = [1e-6;1e-6;1e-6;1e-6];   % Convergence tolerance
    max_iters = 100;    % Maximum number of iteration
   
    while((error(1) > err_toler) || (error(2) > err_toler) || (error(3) > err_toler) || (error(4) > err_toler))
        q1_num = q_old(1,1);
        q2_num = q_old(2,1);
        q3_num = q_old(3,1);
        q4_num = q_old(4,1);
        
        W_sub = subs(W,[q1,q2,q3,q4],[q1_num,q2_num,q3_num,q4_num]);
        F_Qn = W_sub - x_desired;
        
        p_inv_sub = vpa(subs(pinv(J_Matrix),[q1,q2,q3,q4],[q1_num,q2_num,q3_num,q4_num]));
        
        q_new = vpa(q_old - (p_inv_sub * F_Qn));
        disp('Qnew =');
        disp (q_new);
        %error = vpa(W_sub - x_desired)
        error(1) = vpa(abs(q_new(1) - q_old(1)));
        error(2) = vpa(abs(q_new(2) - q_old(2)));
        error(3) = vpa(abs(q_new(3) - q_old(3)));
        error(4) = vpa(abs(q_new(4) - q_old(4)));
        q_old = q_new;
        i=i+1;
        q = q_old;
            if i>max_iters
                disp('Warning: Maximum iterations reached without convergence.');
                random_matrix = pi * rand(4, 1);
                q_old = random_matrix;
                i = 0;
            %break;
        end
    end
end 