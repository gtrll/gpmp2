function h = plotPlanarMobileBase(robot, pose, vehsize, color, width)
%PLOTPLANARMOBILEBASE Summary of this function goes here
%   Detailed explanation goes here

import gtsam.*
import gpmp2.*

% vehicle corners
corner1 = pose.transform_from(Point2(vehsize(1)/2, vehsize(2)/2));
corner2 = pose.transform_from(Point2(-vehsize(1)/2, vehsize(2)/2));
corner3 = pose.transform_from(Point2(-vehsize(1)/2, -vehsize(2)/2));
corner4 = pose.transform_from(Point2(vehsize(1)/2, -vehsize(2)/2));

% vehicle base black lines
h(1) = plot([corner1.x() corner2.x() corner3.x() corner4.x() corner1.x()], ...
    [corner1.y() corner2.y() corner3.y() corner4.y() corner1.y()], 'k-');

end
