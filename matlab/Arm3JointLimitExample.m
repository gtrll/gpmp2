% planar arm obstacle avoidance example, with joint limits applied
% @author Jing Dong
% @date Aug 22, 2017

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
total_time_step = 50;
total_check_step = 50;
delta_t = total_time_sec / total_time_step;
check_inter = total_check_step / total_time_step - 1;

% use GP interpolation
use_GP_inter = true;

% arm model
arm = generateArm('SimpleThreeLinksArm');

% GP
Qc = 1.0 * eye(3);
Qc_model = noiseModel.Gaussian.Covariance(Qc); 

% obstacle cost settings
cost_sigma = 0.1;
epsilon_dist = 0.2;

% prior to start/goal
pose_fix = noiseModel.Isotropic.Sigma(3, 0.0001);
vel_fix = noiseModel.Isotropic.Sigma(3, 0.0001);

% start and end conf
start_conf = [0, 0, 0]';
start_vel = [0, 0, 0]';
end_conf = [0.9, pi/2-0.9, 0]';
end_vel = [0, 0, 0]';
avg_vel = (end_conf / total_time_step) / delta_t;


% joint limit param
% we apply a hard fixed joint limit on 3rd joint, fix it to zero

% note the constraint is not *exact* in output, since it's soft constraint
% given by joint_limit_model, although it's very near to hard constraint
% if you need *exact* constraints, post-process the output as you want

flag_joint_limit = true;
joint_limit_vec_down = [-1000, -1000, 0.0]';
joint_limit_vec_up = [1000, 1000, 0.0]';
joint_limit_thresh = 0.001 * ones(3,1);
joint_limit_model = noiseModel.Isotropic.Sigma(3, 0.001);

% joint velocity limit param
flag_joint_vel_limit = true;
joint_vel_limit_vec = [1, 1, 1]';
joint_vel_limit_thresh = 0.01 * ones(3,1);
joint_vel_limit_model = noiseModel.Isotropic.Sigma(3, 0.1);

% plot param
pause_time = total_time_sec / total_time_step;

% plot start / end configuration
figure(1), hold on
plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
plotPlanarArm(arm.fk_model(), start_conf, 'b', 2);
plotPlanarArm(arm.fk_model(), end_conf, 'r', 2);
title('Layout')
hold off


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
    
    % joint limit factor on every pose
    if flag_joint_limit
        graph.add(JointLimitFactorVector(key_pos, joint_limit_model, joint_limit_vec_down, ...
            joint_limit_vec_up, joint_limit_thresh));
    end
    
    % joint velocity limit factor on every velocity
    if flag_joint_vel_limit
        graph.add(VelocityLimitFactorVector(key_vel, joint_vel_limit_model, ...
            joint_vel_limit_vec, joint_vel_limit_thresh));
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
%     conf = init_values.atFixed(symbol('x', i), 2);
%     plotPlanarArm(arm, conf, 'b', 2);
%     pause(pause_time), hold off
% end


%% optimize!
use_trustregion_opt = true;

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


%% joint speed values

fprintf('est speed\t\t\tactual speed\t\t\tdiff\n')
% do not count first and last pose
for i=1:total_time_step-1
    % estimated joint speed
    vel_est = result.atVector(symbol('v', i));
    % actual joint speed
    conf_diff = result.atVector(symbol('x', i+1)) - result.atVector(symbol('x', i-1));
    vel_act = conf_diff / (2*delta_t);
    % show difference
    diff_vel = vel_act - vel_est;
    fprintf('%+6.4f %+6.4f %+6.4f\t\t%+6.4f %+6.4f %+6.4f\t\t%f\n', ...
        vel_est(1), vel_est(2), vel_est(3), vel_act(1), vel_act(2), vel_act(3), ...
        norm(diff_vel))
end



%% plot final values
for i=0:total_time_step
    figure(4), hold on
    title('Optimized Values')
    % plot world
    plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
    % plot arm
    conf = result.atVector(symbol('x', i));
    plotPlanarArm(arm.fk_model(), conf, 'b', 2);
    pause(pause_time), hold off
end

