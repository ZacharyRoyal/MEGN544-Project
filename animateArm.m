function animateArm(traj)
    %% traj: N x 3 matrix of joint angles [t1 t2 t3]
    %% wallPlane, laserLen: optional, passed to drawArm
    nSteps = max(size(traj));
    fig = figure(1);

    wallPlane.n = [-1;0;0];       % normal along +y
    wallPlane.p0 = [1;0;0];  % point on the plane
    
    laserLen = 2.0;
    
    % Store drawn points (with NaN gaps)
    drawn = nan(nSteps,3);
    
    for k = 1:nSteps
        t1 = traj(1, k)
        t2 = traj(2, k);
        t3 = traj(3, k);
    
        % Draw arm and laser
        drawArm(t1, t2, t3, fig);
    
        % % Compute intersection again (or modify drawArm to return it)
        [pHit, valid] = laserIntersection(t1,t2,t3,wallPlane);
        
        if valid
            drawn(k,:) = pHit';
        else
            drawn(k,:) = [NaN NaN NaN]; % break line
        end
        
        % Overlay the drawn path
        hold on;
        plot3(drawn(:,1), drawn(:,2), drawn(:,3), 'm-','LineWidth',1.5);
        hold off;
    
        pause(0.05);
    end
end

function [pHit, valid] = laserIntersection(t1,t2,t3,wallPlane)
    % Build transforms (same as in drawArm)
    Rz1 = rotZ(t1); Ry = rotY(t2); Rz2 = rotZ(t3);

    Rz1_T = eye(4);
    Rz1_T(1:3,1:3) = Rz1;

    Ry_T = eye(4);
    Ry_T(1:3,1:3) = Ry;

    Rz2_T = eye(4);
    Rz2_T(1:3,1:3) = Rz2;

    L1=0.8; L2=0.6; L3=0.4;

    L1_T = eye(4); 
    L1_T(1:3,4) = [0 0 L1];

    L2_T = eye(4); 
    L2_T(1:3,4) = [L2 0 0];

    L3_T = eye(4); 
    L3_T(1:3,4) = [L3 0 0];
    

    T1 = Rz1_T*L1_T;
    T2 = T1*Ry_T*L2_T;
    T3 = T2*Rz2_T*L3_T;
    
    o = T3(1:3,4);
    R = T3(1:3,1:3);
    d = R(:,1);
    
    n = wallPlane.n(:)/norm(wallPlane.n);
    p0w = wallPlane.p0(:);
    den = dot(n,d);
    
    if abs(den) < 1e-9
        valid = false; pHit = [NaN;NaN;NaN];
    else
        tHit = dot(n,(p0w-o))/den;
        if tHit > 0
            pHit = o + tHit*d;
            valid = true;
        else
            pHit = [NaN;NaN;NaN];
            valid = false;
        end
    end
end
