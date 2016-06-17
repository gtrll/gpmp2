function plotArm(arm, conf, color, width)
%PLOTARM Plot Arm class in 3D
%
%   Usage: PLOTARM(arm, conf, color, width)
%   @arm    Arm object
%   @conf   arm configuration vector
%   @color  color string, use plot convention, e.g. 'r' is red
%   @width  line width

position = arm.forwardKinematicsPosition(conf);

style = strcat(color, '-');
plot3(position(1,:), position(2,:), position(3,:), style, 'LineWidth', width);

plot3(position(1,1:end-1), position(2,1:end-1), position(3,1:end-1), 'k.', ...
    'MarkerSize', 10*width);

end

