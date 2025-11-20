function poses = get_shape(shape)


     if strcmp(shape, 'line')
         poses =  [ [9.8; -3; 0], [9.8; 5; 0] ];
         return
     end
     poses = zeros(3, pose_count);
end