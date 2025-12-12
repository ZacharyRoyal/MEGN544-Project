% Constants
% pi = 3.1415;
time_between_points = 1;
granularity = 100;
shape = 'triangle';
turn_sharpness = 0.1;

% define vector of target poses, must have at least two items
target_poses = get_shape(shape);
% Create Links given DH Table
pose_count = size(target_poses, 2);
virtual_pose_count = pose_count + 2; % need two spares for the contstant vel/accel interpolation
linkList = create_linklist();
N_links = length(linkList);

start_time = 0;
end_time = time_between_points * (virtual_pose_count-1);

interpolation_times = linspace(start_time, end_time, granularity*pose_count)'; % Time vector for interpolation between each pair of points
waypoint_times = start_time:time_between_points:end_time;
% pad poses for const accel interp
target_poses = [target_poses(:,1), target_poses];
target_poses = [target_poses, target_poses(:,end)];
target_poses = [waypoint_times; target_poses];

joint_targets = zeros(5, size(target_poses, 2));
joint_targets(1, :) = waypoint_times;

ikSolutions = zeros(N_links , length(interpolation_times)); % Preallocate for inverse kinematics solutions
torques = zeros(3, length(interpolation_times));

boundary_conditions = struct(...
                            'base_angular_velocity', [0; 0; 0], ...
                            'base_angular_acceleration', [0; 0; 0], ...
                            'base_linear_acceleration', [0; 0; 9.8], ...
                            'distal_force', [0; 0; 0], ...
                            'distal_torque', [0; 0; 0] ...
                            );

initial_guess = [0; 0; 0; 10];
for i = 1:1:virtual_pose_count
    current_pos = target_poses(2:end, i);
    T_des = eye(4);
    T_des(1:3, 4) = current_pos; % set ik target based on position component
    [sol_params, err] = dhInvKine(linkList, T_des, initial_guess); % compute necessary joint positions
    initial_guess = sol_params; % set it as the initial guess for next loop's ik
    joint_targets(2:end, i) = sol_params;
end

for i = 1:1:length(interpolation_times)
    [q, qdot, qddot] = constAccelInterp(interpolation_times(i), joint_targets', turn_sharpness);
    ikSolutions(:, i) = q; % store that solution
    [Jv, Jvdot] = velocityJacobian(linkList(1:3), q, qdot);
    torques(:, i) = newtonEuler(linkList(1:3), q(1:3), qdot(1:3), qddot(1:3), boundary_conditions);
    error = norm(err(1:3));
end

animateArm(ikSolutions, torques);