function J_Matrix = calculate_partial_derivatives()
    % Calculate the partial derivatives of the end-effector position with respect to each joint angle
    
    % Get the transformation matrix T from the forward kinematics function
      T = forward_kinematics_func_V2();

    % Extract the fourth column elements of the end-effector position
    f1 = T(1, 4); %X.E.E
    f2 = T(2, 4); %Y.E.E
    f3 = T(3, 4); %Z.E.E

    %disp(f1);
    %disp(f2);
    %disp(f3);
    

    % Define symbolic variables for joint angles to use for differentiation
    syms q1 q2 q3 q4 real

    % Calculate the partial derivatives with respect to each joint angle
      partial_q1_f1 = diff(f1, q1);
      partial_q2_f1 = diff(f1, q2);
      partial_q3_f1 = diff(f1, q3);
      partial_q4_f1 = diff(f1, q4);

      partial_q1_f2 = diff(f2, q1);
      partial_q2_f2 = diff(f2, q2);
      partial_q3_f2 = diff(f2, q3);
      partial_q4_f2 = diff(f2, q4);

      partial_q1_f3 = diff(f3, q1);
      partial_q2_f3 = diff(f3, q2);
      partial_q3_f3 = diff(f3, q3);
      partial_q4_f3 = diff(f3, q4);


    % Store results in a matrix for easy interpretation
     J_Matrix = [partial_q1_f1, partial_q2_f1, partial_q3_f1, partial_q4_f1;
                 partial_q1_f2, partial_q2_f2, partial_q3_f2, partial_q4_f2;   
                 partial_q1_f3, partial_q2_f3, partial_q3_f3, partial_q4_f3];

     %disp(J_Matrix);

   % Display results
   % disp('Partial derivatives of the end-effector first row, first column with respect to q1, q2, q3, and q4:');
   % disp(partial_derivatives);

end
