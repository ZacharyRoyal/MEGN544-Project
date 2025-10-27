% angleAxis2Rot: create rotation matrix for angle-axis represenation
% 
% function [rot] = angleAxis2Rot(Omega) 
% Converts a given Omega vector for a angle-axis rotation into the
% corresponding rotation matrix
% 
% rot = rotation matrix for rotation about axis by theta
% 
% Omega = k vector times theta encoding desired rotation
% 
% Zachary Royal
% 10891021
% MEGN 544
% 2025-09-26
function [rot] = angleAxis2Rot(Omega) 
    theta = norm(Omega, 2);

    % catch edge case to prevent divide by zero on next step
    if(theta == 0)
        rot = eye(3);
        return
    end

    k = Omega/theta;
    term1 = eye(3)*cos(theta);
    term2 = (1-cos(theta))*(k*k');
    term3 = sin(theta)*cpMap(k);
    rot = term1 + term2 + term3;
end