% twist2Transform: takes in a given twist transform and creates the
% corresponding transformation matrix 
% 
% [H] = twist2Transform(t) 
% Converts a given twist transform vector into the homogeneous transformation
% matrix that encodes the same rotation and translation 
% 
% H = transformation matrix encoding the given twist vector
% 
% t = given twist vector
% 
% Zachary Royal
% 10891021
% MEGN 544
% 2025-10-02
function [H] = twist2Transform(t) 

    v = t(1:3);
    Omega = t(4:6);

    theta = norm(Omega, 2);

    k = Omega/theta;

    I = eye(3);

    R = eye(3) + sin(theta)*cpMap(k) + ((1-cos(theta))*(cpMap(k)^2));

    d = ((I-R)*cpMap(k)+Omega*k')*v;

    if abs(theta) < 0.01
        R = I;
        d = v;
    end

    H = [
         [R(1,1), R(1,2), R(1,3), d(1)];
         [R(2,1), R(2,2), R(2,3), d(2)];
         [R(3,1), R(3,2), R(3,3), d(3)];
         [0, 0, 0, 1];
        ];

end