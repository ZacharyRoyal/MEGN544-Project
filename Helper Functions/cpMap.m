% cpMap: create skew matrix for cross product from a given vector
% 
% function [X] = cpMap(w) 
% Creates skew matrix for use with computing the cross product from a given vector
% 
% X = skew matrix
% 
% w = vector 
% 
% Zachary Royal
% 10891021
% MEGN 544
% 2025-09-26
function [X] = cpMap(w)
    packed_matrix = [
                     [0 -w(3) w(2)];
                     [w(3) 0 -w(1)];
                     [-w(2) w(1) 0];
                    ];
    X = packed_matrix;
end