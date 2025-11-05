function animateArm(ikSolutions)
    % animateArm: animate a 3-axis arm trajectory and plot laser trace on wall
    %
    % ikSolutions = 3 x N matrix of joint angles over time

    N = size(ikSolutions, 2);

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
        [laser_point] = drawArm(thetas, arm_ax);

        % Project onto wall (assume wall at x = constant, e.g. x = 10)
        wall_x = 10;
        if abs(laser_point(1)) < 1e-6
            % Avoid division by zero
            continue;
        end
        scale = (wall_x - 0) / laser_point(1); % line from origin to laser_point
        wall_hit = laser_point * scale;

        % Save Y,Z coordinates for wall plot
        laser_hits(:,k) = [wall_hit(2); wall_hit(3)];

        % Update wall plot
        plot(wall_ax, laser_hits(1,1:k), laser_hits(2,1:k), 'r.-');

        % Pause for animation effect
        pause(0.05);
    end
end

