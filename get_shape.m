function [thetas, poses, pose_count] = get_shape(shape)

     if strcmp(shape, 'triangle')
        thetas = [[pi/12; 0; 0], [0; pi/12; 0], [-pi/12; 0; 0], [pi/12; 0; 0]];
        pose_count = 4;
     elseif strcmp(shape, 'square')
        thetas = [[0;0;0], [0; pi/6; 0], [pi/6; pi/6; 0], [pi/6; 0; 0], [0; 0; 0]];
        pose_count = 5;
     elseif strcmp(shape, 'debug')
        thetas = [ [0; pi/6; 0], [pi/6; pi/5; 0] ];
        pose_count = 2;
     end
     poses = zeros(3, pose_count);
end