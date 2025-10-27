% rotX: create X rotation matrix
% 
% function [rot] = rotX(theta) 
% Converts a given angle theta to rotate about the x axis into a rotation
% matrix
% 
% rot = rotation matrix for given angle to turn about x axis 
% 
% theta = angle of rotation about x axis (in radians) 
% 
% Zachary Royal
% 10891021
% MEGN 544
% 2025-09-26
function [rot] = rotX(theta) 
    s = sin(theta);
    c = cos(theta);
    rot = [
            [1 0 0];
            [0 c -s];
            [0 s c]
          ];
end