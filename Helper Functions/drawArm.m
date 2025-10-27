%% drawArm(t1,t2,t3, figureNum) takes in the joint parameters for
% the arm as described in our project and draws the configuration in 3D
%
% t1,t2,t3 - The variable DH parameters for the arm
% TargetPose - the target the arm is trying to reach
% figureNum - the figure number to plot the arm in, if it is not provided,
% a new figure will be generated
%%
function drawArm(t1,t2,t3, figureNum, TargetPose)

% Link lengths ADJUST AS NEEDED
a1 = 1;
a2 = 0.5;
a3 = 0.3;

% Base point
p0 = [0;0;0];

% First link along z-axis
p1 = p0 + [0;0;a1];

% Second link along x-axis
p2 = p1 + [a2;0;0];

% Third link along x-axis
p3 = p2 + [a3;0;0];

% Compute orientation using ZYZ convention
Rz1 = [cos(t1) -sin(t1) 0;
       sin(t1)  cos(t1) 0;
       0        0       1];

Ry  = [cos(t2)  0 sin(t2);
       0        1 0;
      -sin(t2)  0 cos(t2)];

Rz2 = [cos(t3) -sin(t3) 0;
       sin(t3)  cos(t3) 0;
       0        0       1];

R = Rz1 * Ry * Rz2;

% Collect points for plotting
L = [p0 p1 p2 p3]';

figure(figureNum)
plot3(L(:,1),L(:,2),L(:,3),'y-','LineWidth',2);
hold on;

% Draw target pose if provided
if exist('TargetPose','var')
    quiver3(ones(3,1)*TargetPose(1,4), ones(3,1)*TargetPose(2,4), ones(3,1)*TargetPose(3,4), TargetPose(1,1:3)', TargetPose(2,1:3)', TargetPose(3,1:3)', 0.5,'r');
    plot3(TargetPose(1,4),TargetPose(2,4),TargetPose(3,4), 'r*','MarkerSize',8);
end

% Draw orientation axes at end-effector
quiver3(ones(3,1)*p3(1), ones(3,1)*p3(2), ones(3,1)*p3(3), R(1,:)',R(2,:)',R(3,:)',0.3,'g');

hold off;
axis equal
grid on
xlabel('X'); ylabel('Y'); zlabel('Z');
axis([-1.5,1.5,-1.5,1.5,-1.5,1.5])
title('3-axis ZYZ Robot Arm')

end