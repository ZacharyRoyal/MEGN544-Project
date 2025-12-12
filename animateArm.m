function animateArm(ikSolutions)
    % animateArm: Animate 4-DOF Laser Arm
    
    N = size(ikSolutions, 2);
    wall_x = 10;
    
    % Wall definition for plotting
    wall_pts = [ [wall_x; -5; -5] [wall_x; 5; -5] [wall_x; 5; 5] [wall_x; -5; 5] ];

    figure(1); clf;
    pause(5)
    arm_ax = subplot(1,2,1); 
    wall_ax = subplot(1,2,2); 
    hold(wall_ax, 'on');
    xlabel(wall_ax, 'Y'); ylabel(wall_ax, 'Z');
    title(wall_ax, 'Laser Trace on Wall');
    grid(wall_ax, 'on');
    axis(wall_ax, [-5 5 -5 5]); % Fix the scale so it doesn't jump
    axis(wall_ax, 'equal');

    laser_hits = zeros(2, N);    

    for k = 1:N
        % Get current params [theta1, theta2, theta3, d4]
        params = ikSolutions(:,k);
    
        % Draw arm and get exact tip position
        [p4, ~] = drawArm(params, arm_ax);
        
        % Draw the Wall
        hold(arm_ax, "on");
        fill3(arm_ax, wall_pts(1,:), wall_pts(2,:), wall_pts(3,:), "b", 'FaceAlpha', 0.1);
        
        % Mark the hit point on 3D plot
        plot3(arm_ax, p4(1), p4(2), p4(3), 'gx', 'MarkerSize', 10, 'LineWidth', 2);
        hold(arm_ax, "off");

        % Store Y,Z for 2D plot
        laser_hits(:,k) = [p4(2); p4(3)];
        

        % Update 2D Wall Plot
        plot(wall_ax, laser_hits(1,1:k), laser_hits(2,1:k), 'r.-');
        
        drawnow;
        % pause(0.01); 
    end
end