% 7DOF WAM arm replanning example
% @author Jing Dong


close all
clear

import gtsam.*
import gpmp2.*


%% dataset
dataset = generate3Ddataset('WAMDeskDataset');
origin = [dataset.origin_x, dataset.origin_y, dataset.origin_z];
origin_point3 = Point3(origin');
cell_size = dataset.cell_size;

% sdf
disp('calculating signed distance field ...');
field = signedDistanceField3D(dataset.map, dataset.cell_size);
disp('calculating signed distance field done');

% arm: WAM arm
arm = generateArm('WAMArm');

% plan conf
start_conf = [-0.8,-1.70,1.64,1.29,1.1,-0.106,2.2]';
end_conf = [0.0,0.94,0,1.6,0,-0.919,1.55]';
start_vel = zeros(7,1);
end_vel = zeros(7,1);

% replan conf
replan_pose_idx = 5;
replan_end_conf = [-0.6,0.94,0,1.6,0,-0.919,1.55]';

% plot problem setting
figure(1), hold on
title('Problem Settings')
plotMap3D(dataset.corner_idx, origin, cell_size);
plotArm(arm.fk_model(), start_conf, 'r', 2)
plotArm(arm.fk_model(), end_conf, 'b', 2)
plotArm(arm.fk_model(), replan_end_conf, 'g', 2)
% plot config
set3DPlotRange(dataset)
grid on, view(3)
hold off


%% settings
total_time_sec = 2;
total_time_step = 10;
total_check_step = 100;
delta_t = total_time_sec / total_time_step;
check_inter = total_check_step / total_time_step - 1;

% GP
Qc = 1 * eye(7);
Qc_model = noiseModel.Gaussian.Covariance(Qc); 

% algo settings
cost_sigma = 0.02;
epsilon_dist = 0.2;

% noise model
pose_fix_sigma = 0.0001;
vel_fix_sigma = 0.0001;

% init sdf
sdf = SignedDistanceField(origin_point3, cell_size, size(field, 1), ...
    size(field, 2), size(field, 3));
for z = 1:size(field, 3)
    sdf.initFieldData(z-1, field(:,:,z)');
end

% plot settings
plot_inter_traj = true;
plot_inter = 5;
if plot_inter_traj
    total_plot_step = total_time_step * (plot_inter + 1);
else
    total_plot_step = total_time_step;
    plot_inter = 0;
end
pause_time = total_time_sec / total_plot_step;


%% isam

% settings
opt_setting = TrajOptimizerSetting(7);
opt_setting.set_total_step(total_time_step);
opt_setting.set_total_time(total_time_sec);
opt_setting.set_epsilon(epsilon_dist);
opt_setting.set_cost_sigma(cost_sigma);
opt_setting.set_obs_check_inter(check_inter);
opt_setting.set_conf_prior_model(pose_fix_sigma);
opt_setting.set_vel_prior_model(vel_fix_sigma);
opt_setting.set_Qc_model(Qc);

% initial values by batch
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
ori_traj_values = BatchTrajOptimize3DArm(arm, sdf, start_conf, start_vel, end_conf, ...
    end_vel, init_values, opt_setting);

% isam init
isam_planner = ISAM2TrajOptimizer3DArm(arm, sdf, opt_setting);
% initial graph
isam_planner.initFactorGraph(start_conf, start_vel, end_conf, end_vel);
% insert original traj values
isam_planner.initValues(ori_traj_values);
% one update let isam accept original values
isam_planner.update();


%% replanning

% fix currernt conf and vel, update replanned end conf and vel
curr_conf = ori_traj_values.atVector(symbol('x', replan_pose_idx));
curr_vel = ori_traj_values.atVector(symbol('v', replan_pose_idx));
isam_planner.fixConfigAndVel(replan_pose_idx, curr_conf, curr_vel);
% update replanned end conf and vel
isam_planner.changeGoalConfigAndVel(replan_end_conf, end_vel);
% optimize! 
isam_planner.update();
% get results
replan_result = isam_planner.values();


%% plot results
if plot_inter_traj
    plot_values = interpolateArmTraj(ori_traj_values, Qc_model, delta_t, plot_inter);
    plot_replan_result = interpolateArmTraj(replan_result, Qc_model, delta_t, plot_inter);
else
    plot_values = ori_traj_values;
    plot_replan_result = replan_result;
end

% plot final values
figure(4)
title('Result Values')
grid on, view(-83, 86)
hold on

% plot world
plotMap3D(dataset.corner_idx, origin, dataset.cell_size);

% first plann results
for i=0:total_plot_step
    % plot arm
    conf = plot_values.atVector(symbol('x', i));
    plotArm(arm.fk_model(), conf, 'b', 1);
    % plot config
    set3DPlotRange(dataset);
    pause(pause_time)
end

% replann results
for i=replan_pose_idx*(plot_inter+1) : total_plot_step
    % plot arm
    conf = plot_replan_result.atVector(symbol('x', i));
    plotArm(arm.fk_model(), conf, 'r', 1);
    % plot config
    set3DPlotRange(dataset);
    pause(pause_time)
end

hold off

