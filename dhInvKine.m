


% myFunctionName: Short description of what this function does.
%
% [output1, output2] = myFunctionName(input1, input2)
% A multiline description of the function would be appropriate.
%
% output1 = description of the first output (include units if appropriate)
% output2 = description of the second output (include units if appropriate)
%
% linkList =  list of the joint parameters created by createLink
% desTransform = desired homogeneous transformed initial guess at the 
% parameters, according to the robot's encoders
%
% Your Name
% Your Student Number
% Course Number
% Date
function [paramList, error] = dhInvKine(linkList, desTransform, paramListGuess)
    opt_steps = 1000;
    tolerance = 1e-12;
    paramList = paramListGuess;
    T_des = desTransform;
    T_curr = dhFwdKine(linkList, paramList);
    curr_error = compute_pose_error( T_des, T_curr);

    iter = 0;

    while norm(curr_error(1:3)) > tolerance && iter < opt_steps
        % Compute Jacobian
        [J, ~] = velocityJacobian(linkList, paramList);
        J_pos = J(1:3, :);
        err_pos = curr_error(1:3);
        delta_x = pinv(J_pos) * err_pos;
        
        paramList = paramList + delta_x;
        %paramList(1:3) = wrapToPi(paramList(1:3));
        if paramList(4) < 0
             paramList(4) = 0.1;
        end

        % Recompute my errors for the next step
        T_curr = dhFwdKine(linkList, paramList);
        curr_error = compute_pose_error( T_des, T_curr);
        iter = iter + 1;
    end
    error = curr_error;
end