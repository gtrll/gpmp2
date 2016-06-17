% planar arm obstacle avoidance with reaching a goal
% @author Jing Dong
% @date Nov 23, 2015

close all
clear

import gtsam.*
import gpmp2.*


%% small dataset
dataset = generate2Ddataset('TwoObstaclesDataset');
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
total_time_step = 10;
total_check_step = 50;
delta_t = total_time_sec / total_time_step;
check_inter = total_check_step / total_time_step - 1;

% use GP interpolation
use_GP_inter = true;

% abstract arm
a = [5, 5, 5]';
d = [0, 0, 0]';
alpha = [0, 0, 0]';
arm = Arm(3, a, alpha, d);

% arm model
arm = generateArm('SimpleThreeLinksArm');

% GP
Qc = eye(3);
Qc_model = noiseModel.Gaussian.Covariance(Qc); 

% obstacle cost settings
cost_sigma = 0.1;
epsilon_dist = 0.1;

% prior model on start conf/velocity
pose_fix = noiseModel.Isotropic.Sigma(3, 0.0001);
vel_fix = noiseModel.Isotropic.Sigma(3, 0.0001);

% final goal point and noise model
goal = [0, 1.1]';
goal_point3 = Point3([goal; 0]);
goal_fix = noiseModel.Isotropic.Sigma(3, 0.0001);

% start and end conf
start_conf = [0, 0, 0]';
start_vel = [0, 0, 0]';

% end conf initial values
end_conf_init = [0, 0, 0]';

% end velocity
end_vel = [0, 0, 0]';
avg_vel = ((end_conf_init - start_conf) / total_time_step) / delta_t;

% plot settings
plot_smooth = false;
plot_inter = 10;

% plot start configuration / goal point
figure(1), hold on
plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
plotPlanarArm(arm.fk_model(), start_conf, 'b', 2);
plot(goal(1), goal(2), 'r*');
title('Layout')
hold off


%% init optimization
graph = NonlinearFactorGraph;
init_values = Values;

for i = 0 : total_time_step
    key_pos = symbol('x', i);
    key_vel = symbol('v', i);
    
    pose = start_conf * (total_time_step-i)/total_time_step + end_conf_init * i/total_time_step;
    vel = avg_vel;
    init_values.insert(key_pos, pose);
    init_values.insert(key_vel, vel);
    
    % priors
    if i==0
        graph.add(PriorFactorVector(key_pos, start_conf, pose_fix));
        graph.add(PriorFactorVector(key_vel, start_vel, vel_fix));
    elseif i==total_time_step
        graph.add(GoalFactorArm(key_pos, goal_fix, arm.fk_model(), goal_point3));
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
        graph.add(ObstaclePlanarSDFFactorArm(...
            key_pos, arm, sdf, cost_sigma, epsilon_dist));
        
        % GP cost factor
        if use_GP_inter & check_inter > 0
            for j = 1:check_inter
                tau = j * (total_time_sec / total_check_step);
                graph.add(ObstaclePlanarSDFFactorGPArm( ...
                    key_pos1, key_vel1, key_pos2, key_vel2, ...
                    arm, sdf, cost_sigma, epsilon_dist, ...
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
%     conf = init_values.atVector(symbol('x', i));
%     plotPlanarArm(arm, conf, 'b', 2);
%     pause(pause_time), hold off
% end


%% optimize!
use_LM = false;
use_trustregion_opt = true;

if use_LM
    parameters = LParams;
    parameters.setVerbosity('ERROR');
    optimizer = DoglegOptimizer(graph, init_values, parameters);
elseif use_trustregion_opt
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
% interpolation for smooth traj to plot

if plot_smooth
    % smooth version
    total_plot_step = total_time_step * plot_inter;
    plot_values = interpolateArm3Traj(result, Qc_model, delta_t, plot_inter-1);
    pause_time = total_time_sec / (total_time_step * plot_inter);
else
    % non-smooth version
    total_plot_step = total_time_step;
    plot_values = result;
    pause_time = total_time_sec / total_time_step;
end

% generate traj line
% each two rows are end eff pos
opt_traj_line = [6, total_time_step+1];
for i=0:total_time_step
    conf = result.atVector(symbol('x', i));
    position = arm.fk_model().forwardKinematicsPosition(conf);
    for j=1:3
        opt_traj_line(j*2-1:j*2, i+1) = position(1:2, j);
    end
end

smooth_traj_line = [6, total_plot_step+1];
for i=0:total_plot_step
    conf = plot_values.atVector(symbol('x', i));
    position = arm.fk_model().forwardKinematicsPosition(conf);
    for j=1:3
        smooth_traj_line(j*2-1:j*2, i+1) = position(1:2, j);
    end
end


% plot
for i=0:total_plot_step
    figure(4), hold on
    title('Optimized Values')
    % plot world
    plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
%     plotSignedDistanceField2D(field, dataset.origin_x, dataset.origin_y, dataset.cell_size, epsilon_dist);
    % plot traj
    for j=1:3
        plot(opt_traj_line(2*j-1,:), opt_traj_line(2*j,:), 'r.', 'MarkerSize', 10);
        plot(smooth_traj_line(2*j-1,:), smooth_traj_line(2*j,:), 'r-.');
    end
    % plot arm
    conf = plot_values.atVector(symbol('x', i));
    plotPlanarArm(arm.fk_model(), conf, 'b', 2);
    % plot goal
    plot(goal(1), goal(2), 'r*', 'MarkerSize', 10);
    pause(pause_time), hold off
end


