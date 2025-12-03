function poses = get_shape(shape)


     if strcmp(shape, 'line')
         poses =  [ [9.8; -3; 0], [9.8; 5; 0] ];
         return
     elseif strcmp(shape, 'square')
        poses =  [ [9.8; -3; 0], ... % 1. Start (Bottom-Left)
                    [9.8;  3; 0], ... % 2. Bottom-Right
                    [9.8;  3; 3], ... % 3. Top-Right
                    [9.8; -3; 3], ... % 4. Top-Left 
                    [9.8; -3; 0] ];   
         return
      elseif strcmp(shape, 'circle')
         % Define Circle Parameters
         center = [9.8; 0; 1.5]; % Center on the wall at (X=9.8, Y=0, Z=0)
         radius = 3.0;
         num_points = 20; % More points = smoother circle
         
         % Generate angles from 0 to 2*pi
         theta = linspace(0, 2*pi, num_points);
         
         % Calculate Y and Z coordinates
         y = center(2) + radius * cos(theta);
         z = center(3) + radius * sin(theta);
         x = zeros(1, num_points) + center(1); % X is constant (on the wall)
         
         % Combine into a 3xN matrix
         poses = [x; y; z];
         return
     end
     poses = zeros(3, pose_count);
end