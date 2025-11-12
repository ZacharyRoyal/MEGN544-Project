function [thetas, poses, pose_count] = get_shape(shape)

     if strcmp(shape, 'triangle')
        thetas = [[pi/12; 0; 0], [0; pi/12; 0], [-pi/12; 0; 0], [pi/12; 0; 0]];
        pose_count = 4;
     elseif strcmp(shape, 'square')
        thetas = [[0;0;0], [0; pi/6; 0], [pi/6; pi/6; 0], [pi/6; 0; 0], [0; 0; 0]];
        pose_count = 5;
     end
     poses = zeros(3, pose_count);
end