function h = plotPlanarArm(arm, conf, color, width)
%PLOTPLANARARM Plot Arm class in 2D
%
%   Usage: PLOTPLANARARM(arm, conf, color, width)
%   @arm    Arm object
%   @conf   arm configuration vector
%   @color  color string, use plot convention, e.g. 'r' is red
%   @width  line width

position = arm.forwardKinematicsPosition(conf);
position = position(1:2, :);
position = [[0;0], position];

style = strcat(color, '-');
h(1) = plot(position(1,:), position(2,:), style, 'LineWidth', width);

h(2) = plot(position(1,1:end-1), position(2,1:end-1), 'k.', 'MarkerSize', 20);

end

