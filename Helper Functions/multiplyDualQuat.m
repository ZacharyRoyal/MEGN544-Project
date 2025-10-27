% multiplyDualQuat: Multuiplies two dual quaternions
% 
% dual_quat = multiplyDualQuat(dual_quat_left, dual_quat_right)
% Takes in two dual quaternions as structs containing 2 4-vectors and multiplies them
% 
% dual_quat = resulting multiplication
% 
% dual_quat_left = first dual quaternion of the multiplication
% dual_quat_right = second dual quaternion of the multiplication
% 
% Zachary Royal
% 10891021
% MEGN 544
% 2025-10-03
function dual_quat = multiplyDualQuat(dual_quat_left, dual_quat_right)

    l_rot = dual_quat_left.rot;
    r_rot = dual_quat_right.rot;

    l_disp = dual_quat_left.disp;
    r_disp = dual_quat_right.disp;

    dual_quat.rot = multiplyQuat(l_rot, r_rot);
    dual_quat.disp = multiplyQuat(l_rot, r_disp) + multiplyQuat(l_disp, r_rot);

end