% define vector of target poses, must have at least two items
pi = 3.1415;

%triangle
%target_thetas = [[pi/12; 0; 0], [0; pi/12; 0], [-pi/12; 0; 0], [pi/12; 0; 0]];
%pose_count = 4;

%square
%target_thetas = [[0;0;0], [0; pi/6; 0], [pi/6; pi/6; 0], [pi/6; 0; 0], [0; 0; 0]];
%pose_count = 5;

target_poses = zeros(3, pose_count);
for i = 1:1:pose_count
    current_thetas = target_thetas(:, i);
    endpoint_pose = compute_forward_kinematics(current_thetas(1), current_thetas(2), current_thetas(3), 3);
    target_poses(:, i) = endpoint_pose(1:3,4)
end

%target_poses = [[4.3802; 2.4133; 0.1], [2.2518; 3.7; 2.5999], [2.2152; 1.1634; 4.43]];
time_between_points = 2;
granularity = 50;

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

        ik_index = j + (i-1)*granularity
        current_x = current_intermediary_poses(1,j);
        current_y = current_intermediary_poses(2,j);
        current_z = current_intermediary_poses(3,j);

        ikSolutions(:, ik_index) = compute_inv_kinematics(current_x, current_y, current_z); % Compute IK for each interpolated pose
    
    end

end
%ikSolutions
animateArm(ikSolutions);