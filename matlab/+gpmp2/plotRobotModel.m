function h = plotRobotModel(robot, conf, color_rgb)
%plotRobotModel Plot RobotModel class in 3D, visualize the body spheres
%   also it can plot any child class of RobotModelm like ArmModel
%
%   Usage: plotRobotModel(robot, conf, color_rgb)
%   @robot      RobotModel(or child) object
%   @conf       robot configuration vector
%   @color_rgb  optional color RGB values, default is gray [0.4 0.4 0.4]

switch nargin
    case 2
        color_rgb = [0.4 0.4 0.4];
end

% points
body_points = robot.sphereCentersMat(conf);

% show
colormap(color_rgb);
[X_ball, Y_ball, Z_ball] = sphere(16);

for i=1:robot.nr_body_spheres()
    h(i) = surf(X_ball * robot.sphere_radius(i-1) + body_points(1, i), ...
                Y_ball * robot.sphere_radius(i-1) + body_points(2, i), ...
                Z_ball * robot.sphere_radius(i-1) + body_points(3, i));
end

end

