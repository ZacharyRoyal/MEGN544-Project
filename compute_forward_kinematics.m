% compute_forward_kinematics: computes pose of a joint given thetas
% 
% [T] = myFunctionName(t1, t2, t3, joint_num) 
% Calculates the homogeneous transformation matrix for a given joint number
% (1,2,3) with provided arm configuration as the three joint angles
% 
% T = homogeneous transformation matrix describing pose of given joint
% 
% t1 = joint 1's angle 
% t2 = joint 2's angle
% t3 = joint 3's angle 
% joint_num = desired joint to give pose for 
function [T] = compute_forward_kinematics(theta, joint_num)
    T = eye(4);
    
    for i = 1:1:joint_num
        a_i = dhTable(i, 'a');
        d_i = dhTable(i, 'd');
        alpha_i = dhTable(i, 'alpha');
        current_joint_T = dhTransform(a_i ...
                                    , d_i, ...
                                    alpha_i, theta(i));
        T = T * current_joint_T;

    end

end