function joint_positions = Inverse_Kin_frog(plot,desired_position)
    % Joint Init
    a1 = 0.7;
    initial_theta1=deg2rad(195); %-195 init. pos.
    initial_theta2=deg2rad(165);%+165 init. pos.
    initial_theta3=deg2rad(195); %-195 init. pos.

    joint_positions = [initial_theta1, initial_theta2, initial_theta3];

    %Iteration set-up
    step_size = -1;
    max_iterations = 40;
    tolerance = 0.001;

    for iter = 1:max_iterations
        % Evaluate the current_position by using the forward kinematics
        % associated function.
        
        current_position = Direct_Kin_frog(plot,rad2deg(joint_positions(1)), rad2deg(joint_positions(2)), rad2deg(joint_positions(3)));
        
        % Position error 
        error_position = desired_position - current_position;

            % Transpose Jacobian Matrix (transposed just for page-space issues)
            J_transpose = [
                -(a1*sin(joint_positions(1)) + a1*sin(joint_positions(1) + joint_positions(2)) + a1*sin(sum(joint_positions))), a1*cos(joint_positions(1)) + a1*cos(joint_positions(1) + joint_positions(2)) + a1*cos(sum(joint_positions)), 0;
                -(a1*sin(joint_positions(1) + joint_positions(2)) + a1*sin(sum(joint_positions))), a1*cos(joint_positions(1) + joint_positions(2)) + a1*cos(sum(joint_positions)), 0;
                -a1*sin(sum(joint_positions)), a1*cos(sum(joint_positions)), 0
            ];
            J_transpose = transpose(J_transpose);

            % Jacobian Matrix Pseudoinvers J+
            J_pseudo_inverse = pinv(J_transpose);

        % Aungular Variation of Joints
        delta_theta = J_pseudo_inverse * error_position;

        % Update joint positions 
        joint_positions = joint_positions + step_size .* delta_theta';

        % CONSTRAINT 
        joint_positions(1) = max(min(joint_positions(1), deg2rad(360)), deg2rad(195));
        joint_positions(2) = max(min(joint_positions(2), deg2rad(165)), deg2rad(0));
        joint_positions(3) = max(min(joint_positions(3), deg2rad(360)), deg2rad(195));

        % Verifica la convergenza
        if norm(error_position) < tolerance
            break;
        end
    end
end
