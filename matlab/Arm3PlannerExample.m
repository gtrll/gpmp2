% planar arm obstacle avoidance example, with integrated planner
% @author Jing Dong
% @date Nov 16, 2015

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

% arm model
arm = generateArm('SimpleThreeLinksArm');

% GP
Qc = eye(3);
Qc_model = noiseModel.Gaussian.Covariance(Qc);

% obstacle cost settings
cost_sigma = 0.1;
epsilon_dist = 0.2;

% prior to start/goal
pose_fix_sigma = 0.0001;
vel_fix_sigma = 0.0001;

% start and end conf
start_conf = [0, 0, 0]';
start_vel = [0, 0, 0]';
end_conf = [0.9, pi/2-0.9, 0]';
end_vel = [0, 0, 0]';


% joint limit param
% we apply a hard fixed joint limit on 3rd joint, fix it to zero

% note the constraint is not *exact* in output, since it's soft constraint
% given by joint_limit_model, although it's very near to hard constraint
% if you need *exact* constraints, post-process the output as you want

flag_limit = true;

joint_limit_vec_down = [-1e3, -1e3, 0.0]';
joint_limit_vec_up = [1e3, 1e3, 0.0]';
joint_limit_thresh = 0.001 * ones(3,1);
joint_limit_model = 0.001 * ones(3,1);

% joint velocity limit param
joint_vel_limit_vec = 1 * ones(3,1);
joint_vel_limit_thresh = 0.01 * ones(3,1);
joint_vel_limit_model = 0.1 * ones(3,1);


% plot settings
plot_smooth = false;
plot_inter = 10;

% plot start / end configuration
figure(1), hold on
plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
plotPlanarArm(arm.fk_model(), start_conf, 'b', 2);
plotPlanarArm(arm.fk_model(), end_conf, 'r', 2);
grid on
title('Layout')
hold off


%% init values
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);

% % plot initial values
% for i=0:total_time_step
%     figure(3), hold on
%     title('Initial Values')
%     % plot world
%     plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
%     % plot arm
%     conf = init_values.atFixed(symbol('x', i), 3);
%     plotPlanarArm(arm, conf, 'b', 2);
%     % write video
%     if output_video
%         currFrame = getframe;
%         writeVideo(vwi, currFrame);
%     end
%     pause(pause_time), hold off
% end


%% optimization settings
opt_setting = TrajOptimizerSetting(3);
opt_setting.set_total_step(total_time_step);
opt_setting.set_total_time(total_time_sec);
opt_setting.set_conf_prior_model(pose_fix_sigma);
opt_setting.set_vel_prior_model(vel_fix_sigma);

opt_setting.set_flag_pos_limit(flag_limit);
opt_setting.set_flag_vel_limit(flag_limit);
opt_setting.set_joint_pos_limits_up(joint_limit_vec_up);
opt_setting.set_joint_pos_limits_down(joint_limit_vec_down);
opt_setting.set_vel_limits(joint_vel_limit_vec);
opt_setting.set_pos_limit_thresh(joint_limit_thresh);
opt_setting.set_vel_limit_thresh(joint_vel_limit_thresh);
opt_setting.set_pos_limit_model(joint_limit_model);
opt_setting.set_vel_limit_model(joint_vel_limit_model);

opt_setting.set_epsilon(epsilon_dist);
opt_setting.set_cost_sigma(cost_sigma);
opt_setting.set_obs_check_inter(check_inter);

opt_setting.set_Qc_model(Qc);

opt_setting.setDogleg();

% opt_setting.setGaussNewton();
% opt_setting.setLM();
% opt_setting.setVerbosityError();
opt_setting.setOptimizationNoIncrase(true);

% optimize!
tic
result = BatchTrajOptimize2DArm(arm, sdf, start_conf, start_vel, end_conf, ...
    end_vel, init_values, opt_setting);
toc

% result.print('Final results')


%% interpolate plot values

if plot_smooth
    % smooth version
    total_plot_step = total_time_step * plot_inter;
    plot_values = interpolateArmTraj(result, Qc_model, delta_t, plot_inter-1);
    pause_time = total_time_sec / (total_time_step * plot_inter);
else
    % non-smooth version
    total_plot_step = total_time_step;
    plot_values = result;
    pause_time = total_time_sec / total_time_step;
end

% plot final values
for i=0:total_plot_step
    figure(4), hold on
    title('Optimized Values')
    % plot world
    plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
    % plot arm
    conf = plot_values.atVector(symbol('x', i));
    vel = plot_values.atVector(symbol('v', i));
    plotPlanarArm(arm.fk_model(), conf, 'b', 2);
    pause(pause_time), hold off
end



