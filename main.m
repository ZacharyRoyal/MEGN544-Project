
% Constants
pi = 3.1415;
time_between_points = 2;
granularity = 100;

% define vector of target poses, must have at least two items
target_poses= get_shape('line');
% Create Links given DH Table
pose_count = 2;
linkList = create_linklist();
N_links = length(linkList);
%target_poses = [[4.3802; 2.4133; 0.1], [2.2518; 3.7; 2.5999], [2.2152; 1.1634; 4.43]];


t = linspace(0, time_between_points, granularity)'; % Time vector for interpolation between each pair of points

ikSolutions = zeros(N_links , length(t)*(pose_count-1)); % Preallocate for inverse kinematics solutions

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
        if i == 1
            %current_guess  = compute_inv_kinematics(current_x, current_y, current_z);
            current_guess = [0; 0; 0; 10];
        end
        T_des = eye(4);
        T_des(1:3, 4) = [current_x; current_y; current_z];
        %sol_params =0;
        %err =0;
        [sol_params, err] = dhInvKine(linkList, T_des, current_guess)
        ikSolutions(:, ik_index) = sol_params;
        current_guess = sol_params;
        error = norm(err)
    end
end
%ikSolutions
animateArm(ikSolutions);