% dhTransform: create homogeneous transformation matrix from one joint to
% the next given DH parameters
% 
% function H = dhTransform(a, d, alpha, theta) 
% Creates homogeneous transformation matrix from one joint (i-1) to
% the next (i) given the DH parameters
% 
% H = homogeneous transformation matrix
% 
% a = the length of the starting (i-1th) link (in meters)
% d = the actuated translation between the links connected by this joint (in meters)
% alpha = the twist of the starting (i-1th)link (in radians)
% theta = the actuated angle between the links connected by this joint
% 
% Zachary Royal
% 10891021
% MEGN 544
% 2025-09-26
function H = dhTransform(a, d, alpha, theta) 
    trans_z = [
               [1 0 0 0];
               [0 1 0 0];
               [0 0 1 d];
               [0 0 0 1];
              ];
    
    theta_rot = rotZ(theta);

    rot_z =  [
              [theta_rot(1,1) theta_rot(1,2) theta_rot(1,3) 0];
              [theta_rot(2,1) theta_rot(2,2) theta_rot(2,3) 0];
              [theta_rot(3,1) theta_rot(3,2) theta_rot(3,3) 0];
              [0 0 0 1];
             ];

    trans_x = [
               [1 0 0 a];
               [0 1 0 0];
               [0 0 1 0];
               [0 0 0 1];
              ];

    alpha_rot = rotX(alpha);

    rot_x =  [
              [alpha_rot(1,1) alpha_rot(1,2) alpha_rot(1,3) 0];
              [alpha_rot(2,1) alpha_rot(2,2) alpha_rot(2,3) 0];
              [alpha_rot(3,1) alpha_rot(3,2) alpha_rot(3,3) 0];
              [0 0 0 1];
             ];

    H = trans_z * rot_z * trans_x * rot_x;

end