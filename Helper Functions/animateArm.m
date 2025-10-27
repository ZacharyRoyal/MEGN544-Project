%% animateLaser.m - Animate ZYZ arm drawing on a wall
clear; clc; close all;

% Trajectory
nSteps = 100;
t = linspace(0, 2*pi, nSteps);

t1_traj = .5*sin(t)-pi/2;                 % sweep
t2_traj = .5*cos(t)-pi;      % oscillate
t3_traj = .000001*t;      % oscillate

% Wall plane
wall.n  = [0;1;0];
wall.p0 = [0;1.5;0];

% Storage for drawn points
drawn = nan(nSteps,3);

fig = figure(1);
for k = 1:nSteps
    t1 = t1_traj(k);
    t2 = t2_traj(k);
    t3 = t3_traj(k);

    % Draw arm and laser; also compute intersection inside drawArm
    drawArm(t1,t2,t3, fig, wall, 2.0);

    % Recompute intersection to record the point (mirrors drawArm logic)
    % Minimal duplication: EE origin and direction
    T0 = eye(4);
    Rz1 = [cos(t1) -sin(t1) 0 0; sin(t1) cos(t1) 0 0; 0 0 1 0; 0 0 0 1];
    Ry  = [cos(t2) 0 sin(t2) 0; 0 1 0 0; -sin(t2) 0 cos(t2) 0; 0 0 0 1];
    Rz2 = [cos(t3) -sin(t3) 0 0; sin(t3) cos(t3) 0 0; 0 0 1 0; 0 0 0 1];

    L1=0.1; L2=0.1; L3=0.1;
    T1 = T0 * Rz1 * transl([0 0 L1]);
    T2 = T1 * Ry  * transl([L2 0 0]);
    T3 = T2 * Rz2 * transl([L3 0 0]);

    o = T3(1:3,4);
    R = T3(1:3,1:3);
    d = R(:,1);  % laser along local x

    n = wall.n / norm(wall.n);
    p0w = wall.p0;
    den = dot(n,d);
    if abs(den) > 1e-9
        tHit = dot(n, (p0w - o)) / den;
        if tHit > 0
            pHit = o + tHit*d;
            if ~isnan(tHit) && tHit > 0
                drawn(k,:) = pHit';
            else
                drawn(k,:) = [NaN NaN NaN];  % break the line
            end
        end
    end

    % Overlay the drawn path
    hold on;
    good = ~any(isnan(drawn),2);
    plot3(drawn(:,1), drawn(:,2), drawn(:,3), 'm-','LineWidth',1.5);
    hold off;

    pause(0.03);
end

function T = transl(v)
T = eye(4); T(1:3,4) = v(:);
end
