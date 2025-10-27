% rot2AngleAxis: extract angle axis representation from a rotation matrix
% 
% function [Omega] = rot2angleAxis(R) 
% Converts a given rotation matrix into the angle axis representation for
% the same rotation
% 
% Omega = k vector times theta encoding desired rotation
%
% R = given rotation matrix
%
% Zachary Royal
% 10891021
% MEGN 544
% 2025-09-26
function [Omega] = rot2AngleAxis(R) 
    cos_theta = (trace(R)-1)/2;

    R_diff_32 = R(3,2)-R(2,3);
    R_diff_13 = R(1,3)-R(3,1);
    R_diff_21 = R(2,1)-R(1,2);
    
    R_diff_vec = [
                  R_diff_32; 
                  R_diff_13; 
                  R_diff_21;
                 ];

    sin_theta = 0.5*norm(R_diff_vec, 2);

    theta = atan2(sin_theta, cos_theta);

    pi = 3.141592;

    % special case, we must compute the elements of k 
    % from the values of the main diagonal
    if abs(theta - pi) < 0.01
        % in this case we also know that cos(theta) = -1
        % so we have:
        k = [
             sqrt((R(1,1)+1)/2);
             sqrt((R(2,2)+1)/2);
             sqrt((R(3,3)+1)/2);
            ];
    else % otherwise we can do it the normal way
        k = 1/(2*sin_theta)*R_diff_vec;
    end

    Omega = theta*k;

end