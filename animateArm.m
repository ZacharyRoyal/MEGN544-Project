function animateArm(ikSolutions)
    % animateArm: animate a 3-axis arm trajectory and plot laser trace on wall
    %
    % ikSolutions = 3 x N matrix of joint angles over time

    N = size(ikSolutions, 2);
    wall_x = 10;
    points = [ [wall_x; -5; -5] [wall_x; 5; -5] [wall_x; 5; 5] [wall_x; -5; 5]  ];

    % Prepare figure windows
    figure(1); clf;
    arm_ax = subplot(1,2,1); % 3D arm plot
    wall_ax = subplot(1,2,2); % 2D wall trajectory plot
    hold(wall_ax, 'on');
    xlabel(wall_ax, 'Y (wall axis)');
    ylabel(wall_ax, 'Z (wall axis)');
    title(wall_ax, 'Laser trajectory on wall');
    grid(wall_ax, 'on');

    % Store laser hits
    laser_hits = zeros(2, N);    

    % Loop through trajectory
    for k = 1:N
        thetas = ikSolutions(:,k);
    
        % Draw arm and get laser intersection point
        [p3, laser_dir] = drawArm(thetas, arm_ax);
        scale = wall_x / laser_dir(1); % line from origin to laser_point
        wall_hit = laser_dir * scale;
        hold(arm_ax, "on");
        fill3(arm_ax, points(1,:), points(2,:), points(3,:), "b", FaceAlpha=.3);
        plot3(arm_ax, [p3(1) wall_hit(1)], ...
              [p3(2) wall_hit(2)], ...
              [p3(3) wall_hit(3)], 'r-','LineWidth',2);
        plot3(arm_ax, wall_hit(1), wall_hit(2), wall_hit(3), 'rx','MarkerSize',10);
        hold(arm_ax, "off");

        % Save Y,Z coordinates for wall plot
        laser_hits(:,k) = [wall_hit(2); wall_hit(3)];

        % Update wall plot
        plot(wall_ax, laser_hits(1,1:k), laser_hits(2,1:k), 'r.-');

        % Pause for animation effect
        pause(0.01);
    end
end

