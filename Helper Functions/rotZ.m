% rotZ: create Z rotation matrix
% 
% function [rot] = rotZ(theta) 
% Converts a given angle theta to rotate about the Z axis into a rotation
% matrix
% 
% rot = rotation matrix for given angle to turn about Z axis 
% 
% theta = angle of rotation about Z axis (in radians) 
% 
% Zachary Royal
% 10891021
% MEGN 544
% 2025-09-26
function [rot] = rotZ(theta) 
    s = sin(theta);
    c = cos(theta);
    rot = [
            [c -s 0];
            [s c 0];
            [0 0  1]
          ];
end