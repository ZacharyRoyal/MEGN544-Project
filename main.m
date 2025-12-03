
% Constants
% pi = 3.1415;
<<<<<<< HEAD
time_between_points = 1;
granularity = 10;
=======
time_between_points = 2;
granularity = 5;
shape = 'cosine';
>>>>>>> fb3cd8781da18bac239163a01e713c31eb1f24f4

% define vector of target poses, must have at least two items
target_poses= get_shape(shape);
% Create Links given DH Table
pose_count = size(target_poses, 2);
virtual_pose_count = pose_count + 2;
linkList = create_linklist();
N_links = length(linkList);

start_time = 0;
end_time = time_between_points * (virtual_pose_count-1);

interpolation_times = linspace(start_time, end_time, granularity*virtual_pose_count)'; % Time vector for interpolation between each pair of points
waypoint_times = start_time:time_between_points:end_time;
% pad poses for const accel interp
target_poses = [target_poses(:,1), target_poses];
target_poses = [target_poses, target_poses(:,end)];
target_poses = [waypoint_times; target_poses];
interpolation_times = interpolation_times(1:end-1);

ikSolutions = zeros(N_links , length(interpolation_times)*(pose_count-1)); % Preallocate for inverse kinematics solutions

initial_guess = [0; 0; 0; 10];
for i = 1:1:length(interpolation_times)
    T_des = eye(4);
    [current_pos, ~, ~] = constAccelInterp(interpolation_times(i), target_poses', 0.33);
    T_des(1:3, 4) = current_pos;

    [sol_params, err] = dhInvKine(linkList, T_des, initial_guess);
    ikSolutions(:, i) = sol_params;
    initial_guess = sol_params;
    error = norm(err(1:3));
end

animateArm(ikSolutions);
return

% interpolate intermediary poses between each pair, and calculate
% inverse kinematics for each one
for i = 1:1:pose_count - 1

    current_start_pose = target_poses(:, i);

    current_next_pose = target_poses(:, i+1);

    [a0, a1, a2, a3] = calCubicCoeffs(0, 1, 0, time_between_points);
    current_intermediary_poses = calCubicInterp(current_start_pose, current_next_pose, a0, a1, a2, a3, 0, t);

    for j = 1:1:length(t)

        ik_index = j + (i-1)* granularity;
        current_x = current_intermediary_poses(1,j);
        current_y = current_intermediary_poses(2,j);
        current_z = current_intermediary_poses(3,j);

        T_des = eye(4);
        T_des(1:3, 4) = [current_x; current_y; current_z];

        [sol_params, err] = dhInvKine(linkList, T_des, initial_guess);
        ikSolutions(:, ik_index) = sol_params;
        initial_guess = sol_params;
        error = norm(err(1:3));
    end
end

animateArm(ikSolutions);