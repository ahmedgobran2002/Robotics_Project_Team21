function J_pseudo_inv = inverse_jacobian_matrix(q)
   
    % Get the Jacobian matrix by calling Differentiation
    J_Matrix = Differentiation();
    %disp(Diff);

    % Calculate the inverse of the Jacobian matrix
    J_transpose = J_Matrix';

    % Pseudo inverse since the robot is over actuated DOF < DOJ
    J_pseudo_inv = simplify((J_transpose)*(inv(J_Matrix*J_transpose)));
 
    % Display the inverse of the Jacobian matrix
    %disp('Pseudo inverse of the Jacobian matrix (J_inv):');
    %disp(J_pseudo_inv);

end
