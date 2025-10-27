pi=3.141592;

min_angle = 0;
max_angle = pi;
rand_angles = (max_angle-min_angle).*rand(100,3) + min_angle;

min_position = 0.1;
max_position = 1.5;
rand_pos = (max_position-min_position).*rand(100,3) + min_position;

for i=1:1:2
    inv_kinHW5(rand_pos(i, 1), rand_pos(i, 2), rand_pos(i, 3), ...
        rand_angles(i, 1), rand_angles(i, 2), rand_angles(i, 3))
end