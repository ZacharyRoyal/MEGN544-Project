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
function [T] = compute_forward_kinematics(t1, t2, t3, joint_num)
    a = [0; 0; 0];
    d = [0.1; 0.1; 0];
    alpha = [pi/2; -pi/2; 0];
    theta = [t1; t2; t3];
    
    T = zeros(4);
    T(1,1) = 1;
    T(2,2) = 1;
    T(3,3) = 1;
    T(4,4) = 1;
    
    for i = 1:1:joint_num
       
        current_joint_T = dhTransform(a(i), d(i), alpha(i), theta(i));
        T = T * current_joint_T;

    end

end