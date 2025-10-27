% transform2Twist: converts a transformation matrix into the corresponding
% twist vector
% 
% t = transform2Twist(H)
% Takes in a given homogeneous transformation matrix and returns the twist
% vector that encodes the same rotation and translation
% 
% t = twist vector that encodes the same transformation as the given
% transformation matrix
% 
% H = given homogeneous transformation matrix
% 
% Zachary Royal
% 10891021
% MEGN 544
% 2025-09-26
function t = transform2Twist(H)

    Omega = rot2AngleAxis(H(1:3,1:3));
    theta = norm(Omega, 2);
    k = Omega/theta;

    d = H(1:3,4);

    I = eye(3);

    term1 = (sin(theta)/(2*(1-cos(theta))))*I;

    term2_top = 2*(1-cos(theta))-theta*sin(theta);
    term2_bot = 2*theta*(1-cos(theta));
    term2 = (term2_top/term2_bot)*(k*k');

    term3 = -0.5*cpMap(k);

    v = (term1+term2+term3)*d;

    if H(1:3,1:3) == eye(3)
        Omega = [0; 0; 0];
        v = d;
    end

    t = [v(1); v(2); v(3); Omega(1); Omega(2); Omega(3)];

end