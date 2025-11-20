% myFunctionName: Short description of what this function does.
% To solve the case for when theta is pi=180 we refrence this 
% page https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation
% [output1, output2] = myFunctionName(input1, input2)
% A multiline description of the function would be appropriate.
%
% output1 = description of the first output (include units if appropriate)
% output2 = description of the second output (include units if appropriate)
%
% input1 = description of the first input (include units if appropriate)
% input2 = description of the second input (include units if appropriate)
%
% Oluwatosin Oseni
% CWID: 10937682
% ROBO 554
% 13-Oct-2025

function [Omega] = rot2AngleAxis(R)
    cos_theta =  0.5 * (trace(R) - 1 );
    inner_norm = [R(3,2) - R(2,3);
                    R(1,3) - R(3,1);
                    R(2,1)- R(1,2)];
    sin_theta = 0.5 * norm (inner_norm);
    theta = atan2(sin_theta, cos_theta);
    if sin_theta < 1e-12
        % Theta is eiter zero or pi
        if cos_theta >0 % Here theta is zero
            Omega = zeros(3, 1);
        else
            %{
                Rodryugos formula
                R = I + S_k Sin(a) + S_k^2 (1 - cos(a))
                a = pi
                R = I + 2S_k^2 
                S_k^2 = kk^T - I
                R = I + 2( kk^T - I)
                 kk^T = 1/2 (R +I)
                B = the sqrt of diagonals
            %}
            k_k_T = 0.5 * (R + eye(3));
            k_squared = diag(k_k_T);
            k = sqrt(k_squared);
            %{
                  Also form the wiki
                so the diagonal terms of B are the squares of the elements
                of ω and the signs (up to sign ambiguity) can be determined
                from the signs of the off-axis terms of B.
            %}
            [~, max_idx] = max(k);
            if max_idx == 1
                k(2) = R(1,2) / (2 * k(1));
                k(3) = R(1,3) / (2 * k(1));
            elseif max_idx == 2
                k(1) = R(1,2) / (2 * k(2));
                k(3) = R(2,3) / (2 * k(2));
            else % max_idx == 3
                k(1) = R(1,3) / (2 * k(3));
                k(2) = R(2,3) / (2 * k(3));
            end
            
            Omega = theta * k;
        end
    else
         if abs(theta) < 1e-4
             Omega = 0.5 * inner_norm;
         else
             k = (0.5/ sin_theta) * inner_norm;
             Omega = theta * k;
         end
    end
    % if sin zero is approximately zero
   
end