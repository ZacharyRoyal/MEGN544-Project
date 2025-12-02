function poses = get_shape(shape)

    wall_x = 9.8;
    center_y = 0;
    center_z = 2.0;
    num_points = 50;

    switch lower(shape)
        case 'line'
           poses = [ [9.8; -3; 0], [9.8; 5; 0] ];
        case 'square'
            poses =  [ [9.8; -3; 0], ... % 1. Start (Bottom-Left)
                    [9.8;  3; 0], ... % 2. Bottom-Right
                    [9.8;  3; 3], ... % 3. Top-Right
                    [9.8; -3; 3], ... % 4. Top-Left 
                    [9.8; -3; 0] ];
        case 'circle'
            radius = 3.0;
            theta = linspace(0, 2 * pi, num_points);
            y = center_y + radius * cos(theta);
            z = 1.5 + radius * sin(theta);
            x = repmat(wall_x, 1, num_points);
            poses = [x; y; z];
         case 'cosine'
            % Cosine Wave
            t = linspace(-pi, pi, num_points);
            scale = 2.0;
            
            y = center_y + (t * 1.5);
            z = center_z + scale * cos(t);
            x = repmat(wall_x, 1, num_points);
            
            poses = [x; y; z];

        case 'heart'
            % Parametric Heart Equation
            % Note: Raw equations output large numbers (~16), so we scale by 0.15
            t = linspace(0, 2*pi, num_points);
            scale = 0.15; 
            
            h_x = 16 * sin(t).^3;
            h_y = 13 * cos(t) - 5 * cos(2*t) - 2 * cos(3*t) - cos(4*t);
            
            y = center_y + (h_x * scale); 
            z = center_z + (h_y * scale);
            x = repmat(wall_x, 1, num_points);
            
            poses = [x; y; z];

        case 'spiral'
            % Archimedean Spiral: Radius grows as angle grows
            rotations = 3;
            t = linspace(0, rotations*2*pi, num_points);
            
            r = 0.2 * t; % Growth factor
            
            y = center_y + r .* cos(t);
            z = center_z + r .* sin(t);
            x = repmat(wall_x, 1, num_points);
            
            poses = [x; y; z];

        otherwise
            error('Shape not recognized. Options: line, square, circle, sine, cosine, heart, spiral');
    end
    end

     %   if strcmp(shape, 'line')
     %     poses =  [ [9.8; -3; 0], [9.8; 5; 0] ];
     %     return
     % elseif strcmp(shape, 'square')
     %    poses =  [ [9.8; -3; 0], ... % 1. Start (Bottom-Left)
     %                [9.8;  3; 0], ... % 2. Bottom-Right
     %                [9.8;  3; 3], ... % 3. Top-Right
     %                [9.8; -3; 3], ... % 4. Top-Left 
     %                [9.8; -3; 0] ];   
     %     return
     %  elseif strcmp(shape, 'circle')
     %     % Define Circle Parameters
     %     center = [9.8; 0; 1.5]; % Center on the wall at (X=9.8, Y=0, Z=0)
     %     radius = 3.0;
     %     num_points = 50; % More points = smoother circle
     % 
     %     % Generate angles from 0 to 2*pi
     %     theta = linspace(0, 2*pi, num_points);
     % 
     %     % Calculate Y and Z coordinates
     %     y = center(2) + radius * cos(theta);
     %     z = center(3) + radius * sin(theta);
     %     x = zeros(1, num_points) + center(1); % X is constant (on the wall)
     % 
     %     % Combine into a 3xN matrix
     %     poses = [x; y; z];
     %     return
     % end
     % poses = zeros(3, pose_count);