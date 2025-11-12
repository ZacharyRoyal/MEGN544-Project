
% Constants
pi = 3.1415;
time_between_points = 2;
granularity = 50;

% define vector of target poses, must have at least two items
[target_thetas, target_poses, pose_count] = get_shape('square');

for i = 1:1:pose_count
    current_thetas_i = target_thetas(:, i);
    endpoint_pose_i = compute_forward_kinematics(current_thetas_i, 3);
    target_poses(:, i) = get_disp( endpoint_pose_i );
end

%target_poses = [[4.3802; 2.4133; 0.1], [2.2518; 3.7; 2.5999], [2.2152; 1.1634; 4.43]];


t = linspace(0, time_between_points, granularity)'; % Time vector for interpolation between each pair of points

ikSolutions = zeros(3, length(t)*(pose_count-1)); % Preallocate for inverse kinematics solutions

% interpolate intermediary poses between each pair, and calculate
% inverse kinematics for each one
for i = 1:1:pose_count-1
    current_start_pose = target_poses(:, i);
    current_next_pose = target_poses(:, i+1);

    [a0, a1, a2, a3] = calCubicCoeffs(0, 1, 0, time_between_points);
    current_intermediary_poses = calCubicInterp(current_start_pose, current_next_pose, a0, a1, a2, a3, 0, t);

    for j = 1:1:length(t)

        ik_index = j + (i-1)* granularity;
        current_x = current_intermediary_poses(1,j);
        current_y = current_intermediary_poses(2,j);
        current_z = current_intermediary_poses(3,j);

        ikSolutions(:, ik_index) = compute_inv_kinematics(current_x, current_y, current_z); % Compute IK for each interpolated pose
    
    end

end
%ikSolutions
animateArm(ikSolutions);