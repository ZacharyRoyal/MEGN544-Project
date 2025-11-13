function [laser_point] = drawArm(thetas, ax)
    % drawArm: draw the 3-axis arm and return laser pointer direction
    % thetas = [theta1; theta2; theta3]
    % ax = axis handle for 3D plot

    % Compute forward kinematics for each joint
    T1 = compute_forward_kinematics(thetas, 1);
    T2 = compute_forward_kinematics(thetas, 2);
    T3 = compute_forward_kinematics(thetas, 3);

    % Extract joint positions
    p0 = [0;0;0];
    p1 = get_disp(T1);
    p2 = get_disp(T2);
    p3 = get_disp(T3);

    % Laser direction: assume pointing along +x of end-effector frame
    laser_dir = T3(1:3,1); % x-axis of end-effector
    laser_point = p3 + 5*laser_dir; % extend a bit outward

    % Plot arm in 3D
    cla(ax);
    plot3(ax, [p0(1) p1(1) p2(1) p3(1)], ...
              [p0(2) p1(2) p2(2) p3(2)], ...
              [p0(3) p1(3) p2(3) p3(3)], 'bo-','LineWidth',2);
    hold(ax,'on');
    plot3(ax, [p3(1) laser_point(1)], ...
              [p3(2) laser_point(2)], ...
              [p3(3) laser_point(3)], 'r-','LineWidth',2);
    plot3(ax, laser_point(1), laser_point(2), laser_point(3), 'rx','MarkerSize',10);

    xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');
    axis(ax,[-1 10 -5 5 -1 6]);
    grid(ax,'on'); view(ax,3);
    title(ax,'3D Arm with Laser');

    hold(ax,'off');
end

