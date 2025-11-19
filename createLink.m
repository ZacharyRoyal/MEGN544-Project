% createLink: Returns the quaternion product between two quaternions
% that are stored in 4 × 1 vectors as [q0;q_vec].
%
%   L =  createLink(a, d, alpha,  theta, offset, centOfMass, mass, inertia)
% Computes the resulting multiplication of two quatenions it computes 
% a = description of the first output (include units if appropriate)
% d = description of the second output (include units if appropriate)
% alpha = description of the second output (include units if appropriate)
% theta = description of the second output (include units if appropriate)
% offset = description of the second output (include units if appropriate)
% centOfMass = description of the second output (include units if appropriate)
% mass = description of the second output (include units if appropriate)
% inertia = description of the second output (include units if appropriate)
%
%
% input1 = description of the first input (include units if appropriate)
% input2 = description of the second input (include units if appropriate)
%
% Oluwatosin Oseni
% CWID: 10937682
% ROBO 554
% 13-Oct-2025\

function  L =  createLink(a, d, alpha,  theta, offset, centOfMass, ...
                    mass, inertia)
% Create a structure for the link parameters

    L = struct('a', a, 'd', d, 'alpha', alpha, 'theta', theta, ...
                    'offset', offset, 'com', centOfMass, ...
                    'mass', mass, 'inertia', inertia);

   if isempty(L.theta)
        % If theta is empty, it's a rotary joint
        L.isRotary = 1;
        
    elseif isempty(L.d)
        % If d is empty, it's a prismatic joint
        L.isRotary = 0;
        
    else
        % If neither is empty, it's a fixed/static joint
        L.isRotary = -1;
    end
end