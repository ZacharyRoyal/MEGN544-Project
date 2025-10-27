% pq2DualQuat: converts a position vector and unit quaternion into the
% corresponding dual quaternions
% 
% dual_quat = pq2DualQuat(pos, quat)
% takes in a position vector and unit rotation quaternion and creates the
% dual quaternion that encodes the same total transformation
% 
% dual_quat = qual quaternion that encodes the same transformation as the
% inputs
% 
% pos = given position (really displacement since this is a transformation) vector 
% quat = given unit quaternion for the rotation component
% 
% Zachary Royal
% 10891021
% MEGN 544
% 2025-10-03
function dual_quat = pq2DualQuat(pos, quat)

    pos_q = [0 ; pos(1); pos(2); pos(3)];

    dual_quat.rot = quat;
    dual_quat.disp = 0.5*multiplyQuat(pos_q, quat);

end