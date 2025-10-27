% rotY: create Y rotation matrix
% 
% function [rot] = rotY(theta) 
% Converts a given angle theta to rotate about the Y axis into a rotation
% matrix
% 
% rot = rotation matrix for given angle to turn about Y axis 
% 
% theta = angle of rotation about Y axis (in radians) 
% 
% Zachary Royal
% 10891021
% MEGN 544
% 2025-09-26
function [rot] = rotY(theta) 
    s = sin(theta);
    c = cos(theta);
    rot = [
            [c 0 s];
            [0 1 0];
            [-s 0 c]
          ];
end