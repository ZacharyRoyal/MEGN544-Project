%% drawArm(t1,t2,t3, figureNum) takes in the joint parameters for
% the arm as described in our project and draws the configuration in 3D
%
% t1,t2,t3 - The variable DH parameters for the arm
% figureNum - the figure number to plot the arm in, if it is not provided,
% a new figure will be generated
%%
function drawArm(t1,t2,t3, figureNum, wallPlane, laserLen)
%% drawArm(t1,t2,t3, figureNum, wallPlane, laserLen)
% Draws a 3-axis ZYZ robot arm and a laser pointing to a wall.
%
% t1,t2,t3  : ZYZ joint angles (radians)
% figureNum : optional figure handle/number
% wallPlane : optional struct with fields:
%             .n  (3x1 unit normal) and .p0 (3x1 point on the plane)
%             default: z = zWall (n = [0;0;1], p0 = [0;0;zWall])
% laserLen  : optional finite length for drawing the laser ray segment
%             (only for visualization). Intersection is computed analytically.

if ~exist('figureNum','var') || isempty(figureNum)
    figureNum = figure();
end

% Default wall: vertical plane at y = yWall
if ~exist('wallPlane','var') || isempty(wallPlane)
    yWall = 1.2;
    wallPlane.n = [0;5;0];       % normal along +y
    wallPlane.p0 = [0;yWall;0];  % point on the plane
end

% Default laser segment length for visualization
if ~exist('laserLen','var') || isempty(laserLen)
    laserLen = 2.0;
end

% Link lengths
a1 = 0.1;
a2 = 0.1;
a3 = 0.1;

% Base frame and helpers
T0 = eye(4);
p0 = T0(1:3,4)';

Rz1 = trotz(t1);
Ry  = troty(t2);
Rz2 = trotz(t3);

% Serial chain with small translations for visibility
T1 = T0 * Rz1 * transl([0 0 a1]);
T2 = T1 * Ry  * transl([a2 0 0]);
T3 = T2 * Rz2 * transl([a3 0 0]);

% Arm points
p1 = T1(1:3,4)';
p2 = T2(1:3,4)';
p3 = T3(1:3,4)';

% End-effector rotation and laser direction (local x-axis)
R  = T3(1:3,1:3);
d  = R(:,1);         % laser points along EE x-axis
o  = p3(:);          % laser origin

% Laser-wall intersection (ray-plane)
% Solve for t: o + t d lies on plane with normal n through p0
n = wallPlane.n(:) / norm(wallPlane.n);
p0w = wallPlane.p0(:);
den = dot(n, d);
if abs(den) < 1e-9
    tHit = NaN;          % nearly parallel: no intersection
    pHit = [NaN;NaN;NaN];
else
    tHit = dot(n, (p0w - o)) / den;
    pHit = o + tHit * d;
end

% Collect points and plot arm
L = [p0; p1; p2; p3];
figure(figureNum)
plot3(L(:,1),L(:,2),L(:,3),'y-','LineWidth',2); hold on;
plot3(L(:,1),L(:,2),L(:,3),'ko','MarkerSize',6,'MarkerFaceColor','k');

% Plot laser ray (finite segment for visualization)
laserEnd = o + laserLen * d;
plot3([o(1) laserEnd(1)], [o(2) laserEnd(2)], [o(3) laserEnd(3)], 'r--', 'LineWidth', 1.5);

% Plot wall as a transparent patch (optional: only for y = const planes)
if norm(n - [0;1;0]) < 1e-12
    yWall = p0w(2);
    s = 2.0;
    [Xw,Zw] = meshgrid(linspace(-s,s,2), linspace(0,2,2));
    Yw = yWall * ones(size(Xw));
    surf(Xw, Yw, Zw, 'FaceAlpha', 0.05, 'EdgeColor', 'none', ...
         'FaceColor', [0.6 0.6 0.9]);
end

% Plot intersection point if it exists in front of the arm
if ~isnan(tHit) && tHit > 0
    plot3(pHit(1), pHit(2), pHit(3), 'm.', 'MarkerSize', 18);
end

axis equal; grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
axis([-2,2,-2,2,0,2])
title('3-axis ZYZ arm with laser intersection')
hold off;

end

%% Helpers
function T = trotz(theta)
T = [cos(theta) -sin(theta) 0 0;
     sin(theta)  cos(theta) 0 0;
     0           0          1 0;
     0           0          0 1];
end

function T = troty(theta)
T = [cos(theta)  0 sin(theta) 0;
     0           1 0          0;
    -sin(theta)  0 cos(theta) 0;
     0           0 0          1];
end

function T = transl(v)
T = eye(4);
T(1:3,4) = v(:);
end

