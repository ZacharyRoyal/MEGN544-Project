% Define initial and final poses
q0 = [5; -.1; .1]; 
qf = [1.3367; 2.1151; 4.43];

t = linspace(0, 2, 50)'; % Time vector for interpolation

[a0, a1, a2, a3] = calCubicCoeffs(0, 1, 0, 2);
q = calCubicInterp(q0, qf, a0, a1, a2, a3, 0, t);

% Calculate the inverse kinematics at each time
ikSolutions = zeros(3, length(t)); % Preallocate for inverse kinematics solutions
for i = 1:length(t)
    ikSolutions(:, i) = compute_inv_kinematics(q(1,i), q(2,i), q(3,i)); % Compute IK for each interpolated pose
end

% ikSolutions
animateArm(ikSolutions);

