function h = plotPlannarMobileArm(marm, p, vehsize, color, width)
%PLOTPLANNARMOBILEARM Summary of this function goes here
%   Detailed explanation goes here

import gtsam.*
import gpmp2.*

pose = p.pose();
% vehicle corners
corner1 = pose.transformFrom(Point2(vehsize(1)/2, vehsize(2)/2));
corner2 = pose.transformFrom(Point2(-vehsize(1)/2, vehsize(2)/2));
corner3 = pose.transformFrom(Point2(-vehsize(1)/2, -vehsize(2)/2));
corner4 = pose.transformFrom(Point2(vehsize(1)/2, -vehsize(2)/2));

% vehicle base black lines
h(1) = plot([corner1(1) corner2(1) corner3(1) corner4(1) corner1(1)], ...
    [corner1(2) corner2(2) corner3(2) corner4(2) corner1(2)], 'k-');

% arm
position = marm.forwardKinematicsPosition(p);
position = position(1:2, :);

style = strcat(color, '-');
h(2) = plot(position(1,:), position(2,:), style, 'LineWidth', width);

h(3) = plot(position(1,1:end-1), position(2,1:end-1), 'k.', 'MarkerSize', 5);

end

