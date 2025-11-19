% multiplyQuat:Returns the quaternion product between two quaternions
% that are stored in 4 × 1 vectors as [q0;q_vec].
%
%  Q = multiplyQuat(Q_left, Q_right)
% Computes the resulting multiplication of two quatenions it computes
%  PQ = (p_0 q_0 - q_vec.T p_vec, p_0 * q_vec + q_0 * p_vec + S(p_vec) *
%  q_vec
%
% Where S(p_vec) computes the skew symmtric matrix
%
% Q = The 4 x 1 Quaternion vector [q0;q_vec].
%
% Q_left = A 4 x 1 Vector 
% Q_right = A 4 x 1 Vector 
%
% Oluwatosin Oseni
% CWID: 10937682
% ROBO 554
% 13-Oct-2025

function  H =  dhFwdKine(linkList, paramList)
    paramidx = 1;
    H = eye(4);
    for i = 1:length(linkList)
        link_i = linkList(i);
        
        if link_i.isRotary == 1
            link_i.theta = paramList(paramidx) - link_i.offset;
            paramidx = paramidx + 1;
        elseif link_i.isRotary == 0
            link_i.d = paramList(paramidx) - link_i.offset;
            paramidx = paramidx + 1;
        end
        
        H_new = dhTransform(link_i.a, link_i.d, link_i.alpha, link_i.theta);
        H = H * H_new;
    end
end