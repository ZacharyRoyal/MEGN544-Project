function animateArm(traj, wallPlane, laserLen)
    %% traj: N x 3 matrix of joint angles [t1 t2 t3]
    %% wallPlane, laserLen: optional, passed to drawArm
    nSteps = size(traj,1);
    fig = figure(1);
    
    % Store drawn points (with NaN gaps)
    drawn = nan(nSteps,3);
    
    for k = 1:nSteps
        t1 = traj(k,1);
        t2 = traj(k,2);
        t3 = traj(k,3);
    
        % Draw arm and laser
        drawArm(t1, t2, t3, fig, wallPlane, laserLen);
    
        % Compute intersection again (or modify drawArm to return it)
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
    
        pause(0.03);
    end
end

function [pHit, valid] = laserIntersection(t1,t2,t3,wallPlane)
    % Build transforms (same as in drawArm)
    Rz1 = trotz(t1); Ry = troty(t2); Rz2 = trotz(t3);
    L1=0.8; L2=0.6; L3=0.4;
    T1 = Rz1*transl([0 0 L1]);
    T2 = T1*Ry*transl([L2 0 0]);
    T3 = T2*Rz2*transl([L3 0 0]);
    
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

