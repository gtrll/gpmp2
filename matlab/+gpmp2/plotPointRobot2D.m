function h = plotPointRobot2D(robot, conf, color_rgb)
%plotPointRobot2D Plot PointRobotModel in 2D
%
%   Usage: plotRobotModel(robot, conf, color_rgb)
%   @robot      PointRobotModel object
%   @conf       robot configuration vector
%   @color_rgb  optional color RGB values, default is gray [0.4 0.4 0.4]

switch nargin
    case 2
        color_rgb = [0.4 0.4 0.4];
end

% points
body_points = robot.sphereCentersMat(conf);
r = robot.sphere_radius(0);

theta = linspace(0,2*pi);
x = r * cos(theta) + body_points(1,:);
y = r * sin(theta) + body_points(2,:);
h = plot(x, y, 'Color', color_rgb);

end
