% @brief    Point Robot 2D example, building factor graph in matlab
% @author   Mustafa Mukadam
% @date     July 20, 2016

close all
clear

%% Load libraries
import gtsam.*
import gpmp2.*

%% small dataset
dataset = generate2Ddataset('MultiObstacleDataset');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;
origin_point2 = Point2(dataset.origin_x, dataset.origin_y);

% signed distance field
field = signedDistanceField2D(dataset.map, cell_size);
sdf = PlanarSDF(origin_point2, cell_size, field);

% % plot sdf
% figure(2)
% plotSignedDistanceField2D(field, dataset.origin_x, dataset.origin_y, dataset.cell_size);
% title('Signed Distance Field')


%% settings
total_time_sec = 5.0;
total_time_step = 10;
total_check_step = 50;
delta_t = total_time_sec / total_time_step;
check_inter = total_check_step / total_time_step - 1;

% use GP interpolation
use_GP_inter = true;

% point robot model
pR = PointRobot(2,1);
spheres_data = [0  0.0  0.0  0.0  1.5];
nr_body = size(spheres_data, 1);
sphere_vec = BodySphereVector;
sphere_vec.push_back(BodySphere(spheres_data(1,1), spheres_data(1,5), ...
        Point3(spheres_data(1,2:4)')));
pR_model = PointRobotModel(pR, sphere_vec);

% GP
Qc = eye(2);
Qc_model = noiseModel.Gaussian.Covariance(Qc); 

% obstacle cost settings
cost_sigma = 0.3;
epsilon_dist = 2;

% prior to start/goal
pose_fix = noiseModel.Isotropic.Sigma(2, 0.0001);
vel_fix = noiseModel.Isotropic.Sigma(2, 0.0001);

% start and end conf
start_conf = [-15, -8]';
start_vel = [0, 0]';
end_conf = [17, 14]';
end_vel = [0, 0]';
avg_vel = (end_conf / total_time_step) / delta_t;

% plot param
pause_time = total_time_sec / total_time_step;

% % plot start / end configuration
% figure(1), hold on
% plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
% plotPointRobot2D(pR_model, start_conf);
% plotPointRobot2D(pR_model, end_conf);
% title('Layout')
% hold off


%% init optimization
graph = NonlinearFactorGraph;
init_values = Values;

for i = 0 : total_time_step
    key_pos = symbol('x', i);
    key_vel = symbol('v', i);
    
    % initial values: straght line
    pose = start_conf * (total_time_step-i)/total_time_step + end_conf * i/total_time_step;
    vel = avg_vel;
    init_values.insert(key_pos, pose);
    init_values.insert(key_vel, vel);
    
    % priors
    if i==0
        graph.add(PriorFactorVector(key_pos, start_conf, pose_fix));
        graph.add(PriorFactorVector(key_vel, start_vel, vel_fix));
    elseif i==total_time_step
        graph.add(PriorFactorVector(key_pos, end_conf, pose_fix));
        graph.add(PriorFactorVector(key_vel, end_vel, vel_fix));
    end
    
    % GP priors and cost factor
    if i > 0
        key_pos1 = symbol('x', i-1);
        key_pos2 = symbol('x', i);
        key_vel1 = symbol('v', i-1);
        key_vel2 = symbol('v', i);
        graph.add(GaussianProcessPriorLinear(key_pos1, key_vel1, ...
            key_pos2, key_vel2, delta_t, Qc_model));
        
        % cost factor
        graph.add(ObstaclePlanarSDFFactorPointRobot(...
            key_pos, pR_model, sdf, cost_sigma, epsilon_dist));
        
        % GP cost factor
        if use_GP_inter & check_inter > 0
            for j = 1:check_inter
                tau = j * (total_time_sec / total_check_step);
                graph.add(ObstaclePlanarSDFFactorGPPointRobot( ...
                    key_pos1, key_vel1, key_pos2, key_vel2, ...
                    pR_model, sdf, cost_sigma, epsilon_dist, ...
                    Qc_model, delta_t, tau));
            end
        end
    end
end


%% optimize!
use_trustregion_opt = false;

if use_trustregion_opt
    parameters = DoglegParams;
    parameters.setVerbosity('ERROR');
    optimizer = DoglegOptimizer(graph, init_values, parameters);
else
    parameters = GaussNewtonParams;
    parameters.setVerbosity('ERROR');
    optimizer = GaussNewtonOptimizer(graph, init_values, parameters);
end
tic
optimizer.optimize();
toc
result = optimizer.values();
% result.print('Final results')


%% plot final values
for i=0:total_time_step
    figure(4), hold on
    title('Optimized Values')
    % plot world
    plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
    % plot arm
    conf = result.atVector(symbol('x', i));
    plotPointRobot2D(pR_model, conf);
    pause(pause_time), hold off
end
