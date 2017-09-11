function h = plotPlanarMobile2Arms(marm, p, vehsize, color, width)
%PLOTPLANARMOBILE2ARMS Summary of this function goes here
%   Detailed explanation goes here

import gtsam.*
import gpmp2.*

pose = p.pose();
% vehicle corners
corner1 = pose.transform_from(Point2(vehsize(1)/2, vehsize(2)/2));
corner2 = pose.transform_from(Point2(-vehsize(1)/2, vehsize(2)/2));
corner3 = pose.transform_from(Point2(-vehsize(1)/2, -vehsize(2)/2));
corner4 = pose.transform_from(Point2(vehsize(1)/2, -vehsize(2)/2));

% vehicle base black lines
h(1) = plot([corner1.x() corner2.x() corner3.x() corner4.x() corner1.x()], ...
    [corner1.y() corner2.y() corner3.y() corner4.y() corner1.y()], 'k-');

% arm
position = marm.forwardKinematicsPosition(p);
position = position(1:2, :);

style = strcat(color, '-');

h(2) = plot(position(1,1:marm.arm1.dof+1), position(2,1:marm.arm1.dof+1), ...
    style, 'LineWidth', width);
h(2) = plot(position(1,[1,marm.arm1.dof+2:end]), position(2,[1,marm.arm1.dof+2:end]), ...
    style, 'LineWidth', width);

h(3) = plot(position(1,1:marm.arm1.dof), position(2,1:marm.arm1.dof), ...
    'k.', 'MarkerSize', 5);
h(3) = plot(position(1,marm.arm1.dof+2:end-1), position(2,marm.arm1.dof+2:end-1), ...
    'k.', 'MarkerSize', 5);

end

