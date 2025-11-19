% dhTransform: Returns the homogeneous transform corresponding to
% the provided DH parameters for a link.
%
% H = dhTransform(a, d, alpha, theta )
%  H = TransZ(d) * RotZ(theta) * TransX(a) * RotX(alpha)
%
% H = Homogenous transformation matrix
%
%
% a = dispalcement along the X axis
% d = displacement along the Z axis
% alpha = Rotation along the X axis
% theta = Rotation along the Z axis
%
% Oluwatosin Oseni
% CWID: 10937682
% ROBO 554
% 13-Oct-2025
function H = dhTransform(a, d, alpha, theta)
     H_a = [cos(theta), -sin(theta), 0, 0;
             sin(theta), cos(theta), 0, 0;
             0,  0, 1, d;
             0,  0, 0, 1];
     H_b = [1, 0, 0, a;
            0, cos(alpha), -sin(alpha), 0;
            0, sin(alpha), cos(alpha), 0;
            0,0,0,1];
     H = H_a * H_b;
 end