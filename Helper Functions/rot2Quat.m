% rot2Quat: converts rotation matrix to quaternion
% 
% [Q] = rot2Quat(R) 
% Takes in a given rotation matrix and converts it to a unit quaternion
% that encodes the same rotation
% 
% Q = unit quaternion that encodes the same rotation as the given rotation
% matrix
% 
% R = given rotation matrix
% 
% Zachary Royal
% 10891021
% MEGN 544
% 2025-09-26
function [Q] = rot2Quat(R) 

    candidate_q0 = sqrt(0.25*(1+R(1,1)+R(2,2)+R(3,3)));
    candidate_q1 = sqrt(0.25*(1+R(1,1)-R(2,2)-R(3,3)));
    candidate_q2 = sqrt(0.25*(1-R(1,1)+R(2,2)-R(3,3)));
    candidate_q3 = sqrt(0.25*(1-R(1,1)-R(2,2)+R(3,3)));

    candidate_qs = [candidate_q0; candidate_q1; candidate_q2; candidate_q3];

    max_candidate_q = max(candidate_qs);

    if candidate_q0 == max_candidate_q
        fprintf("q_0 max case");
        q_0 = candidate_q0;

        q_1 = (R(3,2)-R(2,3))/(4*q_0);
        q_2 = (R(1,3)-R(3,1))/(4*q_0);
        q_3 = (R(2,1)-R(1,2))/(4*q_0);
        
        Q = [q_0; q_1; q_2; q_3];
        return
    end

    if candidate_q1 == max_candidate_q
        fprintf("q_1 max case");
        q_1 = candidate_q1;

        q_0 = (R(3,2)-R(2,3))/(4*q_1);
        q_2 = (R(1,2)+R(2,1))/(4*q_1);
        q_3 = (R(1,3)+R(3,1))/(4*q_1);

        Q = [q_0; q_1; q_2; q_3];
        return
    end

    if candidate_q2 == max_candidate_q
        fprintf("q_2 max case");
        q_2 = candidate_q2;
        
        q_0 = (R(1,3)-R(3,1))/(4*q_2);
        q_1 = (R(1,2)+R(2,1))/(4*q_2);
        q_3 = (R(2,3)+R(3,2))/(4*q_2);

        Q = [q_0; q_1; q_2; q_3];
        return
    end

    if candidate_q3 == max_candidate_q
        fprintf("q_3 max case");
        q_3 = candidate_q3;

        q_0 = (R(2,1)-R(1,2))/(4*q_3);
        q_1 = (R(1,3)+R(3,1))/(4*q_3);
        q_2 = (R(2,3)+R(3,2))/(4*q_3);
        
        Q = [q_0; q_1; q_2; q_3];
        return
    end

end