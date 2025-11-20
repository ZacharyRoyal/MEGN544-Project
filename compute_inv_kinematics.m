% function [thetas]  = compute_inv_kinematics(R)
%     r13 = R(1, 3);
%     r23 = R(2, 3);
%     r33 = R(3, 3);
%     r31 = R(3, 1);
% 
%     theta_y = atan2(sqrt(r13^2 + r23^2), r33);
%     sin_theta = sin(theta_1);
%     theta_x_last = 0;
%     theta_z_last = 0;
%     if abs(sin(theta_y)) < 1e-5
%        if abs(r_33- 1) < 1e-12
%            sgn_ = 1;
%        else
%            sgn_ = -1;
%        end
%        %theta x+z
%        theta_xz = atan2( -r_12, r_22);
%        A = [2, 0, 1;
%            0, 2, sgn_;
%            1, sgn_, 0];
%        b = [2* theta_x_last;
%            2*theta_z_last;
%            theta_xz];
%        x = A \ b;
%         theta_x = x(1);
%         theta_z = x(2);
%     else
%         theta_x = atan2(r23/sin_theta, r13 / sin_theta);
%         theta_z = atan2(r32 / sin_theta, -r31/sin_theta);
%     end
% 
%     thetas = [theta_x, theta_y, theta_z ];
% 
% end
function [thetas]  = compute_inv_kinematics(px, py, pz)

    d2 = dhTable(2, 'd');
    d1 = dhTable(1, 'd');
    a3 = dhTable(3, 'a');

    r = sqrt(px^2 + py^2)
    psi = 2 * atan(sqrt((r-d2)/(r+d2)))
    phi = atan2(py, px)
    theta_1 = phi + (pi/2) - psi

    theta_2 = asin((pz - d1) /a3);
    theta_3 = 0;
    thetas = [theta_1, theta_2, theta_3]';
end

