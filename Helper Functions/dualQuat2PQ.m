% dualQuat2PQ: converts a dual quaternion into the corresponding position vector and unit
% quaternion
% 
% [pos,quat] = dualQuat2PQ(dual_quat)
% takes in a dual quaternion and extracts the position vector and unit
% quaternion that encode the same translation and rotation
% 
% pos = position vector extracted from the dual quaternion
% quat = unit quaternions extracted from the qual quaternion
% 
% dual_quat = given dual quaternion
% 
% Zachary Royal
% 10891021
% MEGN 544
% 2025-10-03
function [pos,quat] = dualQuat2PQ(dual_quat)

    quat = dual_quat.rot;

    conju = [quat(1); -quat(2); -quat(3); -quat(4)];

    pos_quat = 2 * multiplyQuat(dual_quat.disp, conju);

    pos = pos_quat(2:4);

end