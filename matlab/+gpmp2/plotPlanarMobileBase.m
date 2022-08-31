function h = plotPlanarMobileBase(robot, pose, vehsize, color, width)
%PLOTPLANARMOBILEBASE Summary of this function goes here
%   Detailed explanation goes here

import gtsam.*
import gpmp2.*

% vehicle corners
corner1 = pose.transformFrom(Point2(vehsize(1)/2, vehsize(2)/2));
corner2 = pose.transformFrom(Point2(-vehsize(1)/2, vehsize(2)/2));
corner3 = pose.transformFrom(Point2(-vehsize(1)/2, -vehsize(2)/2));
corner4 = pose.transformFrom(Point2(vehsize(1)/2, -vehsize(2)/2));

% vehicle base black lines
h(1) = plot([corner1(1) corner2(1) corner3(1) corner4(1) corner1(1)], ...
    [corner1(2) corner2(2) corner3(2) corner4(2) corner1(2)], 'k-');

end
