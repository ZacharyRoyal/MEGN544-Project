% rpy2Rot: create roll pitch yaw rotation matrix
% 
% function [rot] = rpy2Rot(roll, pitch, yaw) 
% Converts a given angle theta to rotate about the Y axis into a rotation
% matrix
% 
% rot = rotation matrix for rotation about x by roll, then about y by
% pitch, and then about z by yaw
% 
% roll = angle to rotate about x axis (in radians)
% pitch = angle to rotate about y axis (in radians)
% yaw = angle to rotate about z axis (in radians)
% 
% Zachary Royal
% 10891021
% MEGN 544
% 2025-09-26
function [rot] = rpy2Rot(roll, pitch, yaw) 
    x_rot = rotX(roll);
    y_rot = rotY(pitch);
    z_rot = rotZ(yaw);
    rot = z_rot*y_rot*x_rot;
end