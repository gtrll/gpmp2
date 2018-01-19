close all
clear

import gtsam.*
import gpmp2.*


%% small dataset
dataset = generate2Ddataset('MobileMap1');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;
origin_point2 = Point2(dataset.origin_x, dataset.origin_y);

% signed distance field
field = signedDistanceField2D(dataset.map, cell_size);
sdf = PlanarSDF(origin_point2, cell_size, field);

% plot sdf
figure(2)
plotSignedDistanceField2D(field, dataset.origin_x, dataset.origin_y, dataset.cell_size);
title('Signed Distance Field')


%% settings
total_time_sec = 5.0;
total_time_step = 50;
total_check_step = 50;
delta_t = total_time_sec / total_time_step;
check_inter = total_check_step / total_time_step - 1;

% use 2d vehicle dynamics
use_vehicle_dynamics = true;
dynamics_sigma = 0.001;

% use GP interpolation
use_GP_inter = false;

% arm model
marm = generateMobileArm('SimpleTwoLinksArm');

% GP
Qc = 1 * eye(5);
Qc_model = noiseModel.Gaussian.Covariance(Qc);

% Obstacle avoid settings
cost_sigma = 0.1;
epsilon_dist = 0.1;

% prior to start/goal
pose_fix = noiseModel.Isotropic.Sigma(5, 0.0001);
vel_fix = noiseModel.Isotropic.Sigma(5, 0.0001);

% start and end conf
start_pose = Pose2(-1, 0, pi/2);
start_conf = [0, 0]';
pstart = Pose2Vector(start_pose, start_conf);
start_vel = [0, 0, 0, 0, 0]';

end_pose = Pose2(1, 0, pi/2);
end_conf = [0 0]';
pend = Pose2Vector(end_pose, end_conf);
end_vel = [0, 0, 0, 0, 0]';

avg_vel = [end_pose.x()-start_pose.x(); end_pose.y()-start_pose.y(); ...
    end_pose.theta()-start_pose.theta(); (end_conf / total_time_step)] / delta_t;

% plot param
pause_time = total_time_sec / total_time_step;

% plot start / end configuration
figure(1), hold on
plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
title('Layout')
plotPlanarMobileArm(marm.fk_model(), pstart, [0.4 0.2], 'b', 1);
plotPlanarMobileArm(marm.fk_model(), pend, [0.4 0.2], 'r', 1);
hold off


%% initial values
init_values = Values;

for i = 0 : total_time_step
    key_pos = symbol('x', i);
    key_vel = symbol('v', i);
    
    % initialize as straight line in conf space
    conf = start_conf * (total_time_step-i)/total_time_step + end_conf * i/total_time_step;
    pose = Pose2(start_pose.x() * (total_time_step-i)/total_time_step + ...
        end_pose.x() * i/total_time_step, ...
        start_pose.y() * (total_time_step-i)/total_time_step + ...
        end_pose.y() * i/total_time_step, ...
        start_pose.theta() * (total_time_step-i)/total_time_step + ...
        end_pose.theta() * i/total_time_step);
    vel = avg_vel;
    
    insertPose2VectorInValues(key_pos, Pose2Vector(pose, conf), init_values);
    init_values.insert(key_vel, vel);
end


%% build graph
graph = NonlinearFactorGraph;

for i = 0 : total_time_step
    key_pos = symbol('x', i);
    key_vel = symbol('v', i);
    
    % start/end priors
    if i==0
        graph.add(PriorFactorPose2Vector(key_pos, pstart, pose_fix));
        graph.add(PriorFactorVector(key_vel, start_vel, vel_fix));
    elseif i==total_time_step
        graph.add(PriorFactorPose2Vector(key_pos, pend, pose_fix));
        graph.add(PriorFactorVector(key_vel, end_vel, vel_fix));
    end
    
    % cost factor
    graph.add(ObstaclePlanarSDFFactorPose2MobileArm(key_pos, ...
        marm, sdf, cost_sigma, epsilon_dist));
    
    % vehicle dynamics
    if use_vehicle_dynamics
        graph.add(VehicleDynamicsFactorPose2Vector(key_pos, key_vel, ...
            dynamics_sigma));
    end
    
    % GP priors and cost factor
    if i > 0
        key_pos1 = symbol('x', i-1);
        key_pos2 = symbol('x', i);
        key_vel1 = symbol('v', i-1);
        key_vel2 = symbol('v', i);
        graph.add(GaussianProcessPriorPose2Vector(key_pos1, key_vel1, ...
            key_pos2, key_vel2, delta_t, Qc_model));
        
        
        
        % GP cost factor
        if use_GP_inter & check_inter > 0
            for j = 1:check_inter
                tau = j * (total_time_sec / total_check_step);
                graph.add(ObstaclePlanarSDFFactorGPPose2MobileArm( ...
                    key_pos1, key_vel1, key_pos2, key_vel2, ...
                    marm, sdf, cost_sigma, epsilon_dist, ...
                    Qc_model, delta_t, tau));
            end
        end
    end
end

% %% plot initial values
% for i=0:total_time_step
%     figure(3), hold on
%     title('Initial Values')
%     % plot world
%     plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
%     % plot arm
%     p = atPose2VectorValues(symbol('x', i), init_values);
%     plotPlanarMobileArm(marm.fk_model(), p, [0.4 0.2], 'b', 2);
%     pause(pause_time), hold off
% end


%% optimize!
use_trustregion_opt = true;
use_LM_opt = true;

if use_trustregion_opt
    parameters = DoglegParams;
    parameters.setVerbosity('ERROR');
    optimizer = DoglegOptimizer(graph, init_values, parameters);
elseif use_LM_opt
    parameters = LevenbergMarquardtParams;
    parameters.setVerbosity('ERROR');
    optimizer = LevenbergMarquardtOptimizer(graph, init_values, parameters);
else
    parameters = GaussNewtonParams;
    parameters.setVerbosity('ERROR');
    optimizer = GaussNewtonOptimizer(graph, init_values, parameters);
end

optimizer.optimize();
result = optimizer.values();
% result.print('Final results')


%% plot final values
for i=0:total_time_step
    figure(4), hold on
    title('Optimized Values')
    % plot world
    plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
    % plot arm
    p = atPose2VectorValues(symbol('x', i), result);
    plotPlanarMobileArm(marm.fk_model(), p, [0.4 0.2], 'b', 1);
    pause(pause_time), hold off
end


