function inv_kinHW5(Fx, Fy, Fz, Froll, Fpitch, Fyaw)
    pi = 3.141592;

    a = [
         0;
         1;
         1;
         0;
         0;
         0;
        ];

    d = [
         1;
         0;
         0;
         1;
         0;
         1;
        ];

    alpha = [
             -pi/2;
             0;
             -pi/2;
             pi/2;
             -pi/2;
             0;
            ];

    % backtrack from final position to wrist center
    final_ori = rpy2Rot(Froll, Fpitch, Fyaw);
    O04 = [Fx; Fy; Fz] + (final_ori * [0; 0; -d(6)]);

    theta_1_a = atan2(O04(2), O04(1))
    theta_1_b = theta_1_a + pi;

    % now we find theta 2 and 3
    T01 = dhTransform(a(1), d(1), alpha(1), theta_1_a)
    O14 = inv(T01) * [O04; 1];

    % solve "2" link parallel manipulator

    q = sqrt(a(3)^2+d(4)^2);
    phi = atan2(d(4), a(3));

    delta_frac_top = ((a(2)+q)^2)-(O14(1)^2+O14(2)^2);
    delta_frac_bot = (O14(1)^2+O14(2)^2)-((a(2)-q)^2);
    delta_a = 2*atan(sqrt(delta_frac_top/delta_frac_bot));
    delta_b = -delta_a;
    
    theta_2_a = atan2(O14(2),O14(1))-atan2(q*sin(delta_a), a(2)+q*cos(delta_a))
    %theta_2_b = atan2(h,r)-atan2(q*sin(delta_b), a(2)+q*cos(delta_b));

    theta_3_a = delta_a - phi
    %theta_3_b = delta_b - phi;

    % now we find thetas 4-6 based on final orientation and rotations
    % induced by getting the right position

    T12 = dhTransform(a(2), d(2), alpha(2), theta_2_a)
    T23 = dhTransform(a(3), d(3), alpha(3), theta_3_a)
    % need to do more multiple-solution stuff here

    T03 = T01 * T12 * T23;
    R03 = T03(1:3, 1:3);

    % since the wrist is reponsbile for orientation, it must undo the
    % rotation induced by getting the right position, and then get us to
    % the desired orientation
    R36 = (R03')*final_ori;

    % decompose that R36 as a zyz rotation matrix formed by 
    % ZYZ(theta 4, theta 5, theta 6)

    theta_5_a = atan2(sqrt(R36(1,3)^2+R36(2,3)^2), R36(3,3))
    %theta_5_b = atan2(-sqrt(R36(1,3)^2+R36(2,3)^2), R36(3,3)); 

    theta_4_a = atan2(R36(2,3)/sin(theta_5_a), R36(1,3)/sin(theta_5_a))
    %theta_4_b = atan2(R36(2,3)/sin(theta_5_b), R36(1,3)/sin(theta_5_b));

    theta_6_a = atan2(R36(3,2)/sin(theta_5_a), -R36(3,1)/sin(theta_5_a))
    %theta_6_b = atan2(R36(3,2)/sin(theta_5_b), -R36(3,1)/sin(theta_5_b));

    T34 = dhTransform(a(4), d(4), alpha(4), theta_4_a)
    T45 = dhTransform(a(5), d(5), alpha(5), theta_5_a)
    T56 = dhTransform(a(6), d(6), alpha(6), theta_6_a)

    T06 = T03 * T34 * T45 * T56

    F = [final_ori(1,1), final_ori(1,2), final_ori(1,3), Fx;
         final_ori(2,1), final_ori(2,2), final_ori(2,3), Fy;
         final_ori(3,1), final_ori(3,2), final_ori(3,3), Fz;
         0, 0, 0, 1;
        ]

    Err = F-T06

end