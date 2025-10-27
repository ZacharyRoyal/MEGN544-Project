% rot2rpy: extract roll, pitch, and yaw representation from rotation matrix
% 
% function [roll, pitch, yaw] = rot2RPY(R) 
% Takes a rotation matrix and extracts necessary roll, pitch, and yaw
% angles to perform the same rotation
% 
% roll = angle to rotate about x axis (in radians)
% ptich = angle to rotate about y axis (in radians)
% yaw = angle to rotate about z axis (in radians)
% 
% R = provided rotation matrix
% 
% Zachary Royal
% 10891021
% MEGN 544
% 2025-09-26
function [roll, pitch, yaw] = rot2RPY(R) 
    cos_pitch_pos = sqrt(R(1,1)^2+R(2,1)^2);
    cos_pitch_neg = -sqrt(R(1,1)^2+R(2,1)^2);
    
    pitch_pos = atan2(-R(3,1), cos_pitch_pos);
    pitch_neg = atan2(-R(3,1), cos_pitch_neg);

    pi = 3.141592;

   % degenerate case
    if abs(abs(pitch_pos)-pi/2) < 0.01
        %fprintf("Degenerate case")
        total_z = atan2(-R(1,2), R(2,2)) * R(3,1);
        
        roll = [total_z; total_z];
        pitch = [pitch_pos; pitch_neg];
        yaw = [0; 0];
        return
    end
    %fprintf("Non-Degenerate case")

    roll_pos = atan2(R(3,2)/cos_pitch_pos, R(3,3)/cos_pitch_pos);
    roll_neg = atan2(R(3,2)/cos_pitch_neg, R(3,3)/cos_pitch_neg);

    yaw_pos = atan2(R(2,1)/cos_pitch_pos, R(1,1)/cos_pitch_pos);
    yaw_neg = atan2(R(2,1)/cos_pitch_neg, R(1,1)/cos_pitch_neg);

    roll = [roll_pos; roll_neg];
    pitch = [pitch_pos; pitch_neg];
    yaw = [yaw_pos; yaw_neg];

end