% myFunctionName: Short description of what this function does.
 %
 % [output1, output2] = myFunctionName(input1, input2)
 % A multiline description of the function would be appropriate.
 %
 % output1 = description of the first output (include units if appropriate)
 % output2 = description of the second output (include units if appropriate)
 %
 % input1 = description of the first input (include units if appropriate)
 % input2 = description of the second input (include units if appropriate)
 %
 % Your Name
 % Your Student Number
 % Course Number
 % Date
function  [Jv, JvDot]  = velocityJacobian(linkList, paramList, paramRateList)
    % Compute end pose of end effector
    T = eye(4);
    N = length(linkList);

    d_list = zeros(3, N);
    z_list = zeros(3, N);
    w_list = zeros(3, N+1);
    ddot_list = zeros(3, N);

    w_total = [0;0;0]; % Base anguilar velocity
    v_total = [0;0;0];

    for i = 1:length(linkList)
        link_i = linkList(i);
        if exist('paramRateList', 'var')
            q_dot_i = paramRateList(i);
        else
            q_dot_i = 0;
        end
        q_i = paramList(i);
        
        d_list(:, i) = T(1:3, 4);
        z_list(:, i) = T(1:3, 3);


        if exist('paramRateList', 'var') 
            if link_i.isRotary == 1
                 w_total = w_total + (q_dot_i * z_list(:, i));
            end
            w_list(:, i) = w_total;
        end
        T_i = dhFwdKine(link_i, q_i);
        if exist('paramRateList', 'var')
            
            inner_trans = T(1:3, 1:3) * T_i(1:3, 4);
            v_total =  v_total + cross(w_total,  inner_trans);
            if link_i.isRotary == 0
                v_total = v_total + (q_dot_i * z_list(:, i));
            end
            ddot_list(:,i) = v_total;
        end
        T = T * T_i;
    end
   
    Jv = zeros(6, N);
    d_total = T(1:3, 4);

    for i = 1:length(linkList)
         link_i = linkList(i);
         if link_i.isRotary
            Jv(:, i) = [cross(z_list(:, i), (d_total - d_list(:, i))); ...
                        z_list(:, i)];
         else
            Jv(:, i) = [z_list(:, i); zeros(3, 1)];
         end
    end
    if exist('paramRateList', 'var')
         JvDot = zeros(6, N);% Angular velocity Jacobian
        
         for i = 1:length(linkList)
             link_i = linkList(i);
             if i == 1
                 w_parent = [0;0;0];
                 v_parent = [0;0;0];
             else
                 w_parent = w_list(:, i-1);
                 v_parent = ddot_list(:, i-1);
             end

             z_dot = cross(w_parent, z_list(:, i));
             if link_i.isRotary
                 d_diff = d_total - d_list(:, i);
                 % v_diff = v_total - ddot_list(:, i);
                 v_diff = v_total - v_parent;
    
                 lhs_sum = cross(z_dot, d_diff);
                 rhs_sum = cross(z_list(:, i), v_diff);
                 inner_sum = lhs_sum + rhs_sum;
                 
                 JvDot(:, i) = [inner_sum; z_dot ];
             else
                 JvDot(:, i) = [z_dot; zeros(3, 1)];
             end
         end
    else
        JvDot = [];
    end
end