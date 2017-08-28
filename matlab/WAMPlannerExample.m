% 7DOF WAM arm example, with integrated planner
% @author Jing Dong
% @date Jun 15, 2015

close all
clear

import gtsam.*
import gpmp2.*


%% dataset
dataset = generate3Ddataset('WAMDeskDataset');
origin = [dataset.origin_x, dataset.origin_y, dataset.origin_z];
origin_point3 = Point3(origin');
cell_size = dataset.cell_size;

% init sdf
disp('calculating signed distance field ...');
field = signedDistanceField3D(dataset.map, dataset.cell_size);
disp('calculating signed distance field done');

sdf = SignedDistanceField(origin_point3, cell_size, size(field, 1), ...
    size(field, 2), size(field, 3));
for z = 1:size(field, 3)
    sdf.initFieldData(z-1, field(:,:,z)');
end

% arm: WAM arm
arm = generateArm('WAMArm');

start_conf = [-0.8,-1.70,1.64,1.29,1.1,-0.106,2.2]';
end_conf = [-0.0,0.94,0,1.6,0,-0.919,1.55]';
start_vel = zeros(7,1);
end_vel = zeros(7,1);

% plot problem setting
figure(1), hold on
title('Problem Settings')
plotMap3D(dataset.corner_idx, origin, cell_size);
plotRobotModel(arm, start_conf);
plotRobotModel(arm, end_conf);
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

% plot settings
plot_inter_traj = false;
plot_inter = 4;
if plot_inter_traj
    total_plot_step = total_time_step * (plot_inter + 1);
else
    total_plot_step = total_time_step;
end
pause_time = total_time_sec / total_plot_step;


%% initial traj
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);

% % plot initial traj
% if plot_inter_traj
%     plot_values = interpolateArmTraj(init_values, Qc_model, delta_t, plot_inter);
% else
%     plot_values = init_values;
% end
% 
% % plot init values
% figure(3),
% hold on
% title('Initial Values')
% % plot world
% plotMap3D(dataset.corner_idx, origin, cell_size);
% for i=0:total_plot_step
%     % plot arm
%     conf = plot_values.atVector(symbol('x', i));
%     plotRobotModel(arm, conf)
%     % plot config
%     set3DPlotRange(dataset)
%     grid on, view(2)
%     pause(pause_time)
% end
% hold off


%% optimize!
opt_setting = TrajOptimizerSetting(7);
opt_setting.set_total_step(total_time_step);
opt_setting.set_total_time(total_time_sec);
opt_setting.set_epsilon(epsilon_dist);
opt_setting.set_cost_sigma(cost_sigma);
opt_setting.set_obs_check_inter(check_inter);
opt_setting.set_conf_prior_model(pose_fix_sigma);
opt_setting.set_vel_prior_model(vel_fix_sigma);
opt_setting.set_Qc_model(Qc);

opt_setting.setDogleg();
% opt_setting.setGaussNewton();
% opt_setting.setLM();
% opt_setting.setVerbosityError();

tic
result = BatchTrajOptimize3DArm(arm, sdf, start_conf, start_vel, end_conf, ...
    end_vel, init_values, opt_setting);
fprintf('Optimization Time: %f\n', toc)

% result.print('Final results')


%% plot results
if plot_inter_traj
    plot_values = interpolateArmTraj(result, Qc_model, delta_t, plot_inter);
else
    plot_values = result;
end

% plot final values
figure(4)
hold on
title('Result Values')
grid on, view(-70, 20)
% plot world
plotMap3D(dataset.corner_idx, origin, cell_size);
for i=0:total_plot_step
    % plot arm
    conf = plot_values.atVector(symbol('x', i));
    plotArm(arm.fk_model(), conf, 'b', 2);
    % plot config
    set3DPlotRange(dataset)
    pause(pause_time)
end
hold off

% plot final values
figure(5)
for i=0:total_plot_step
    clf
    hold on, view(-5, 12)
    title('Result Values')
    % plot world
    plotMap3D(dataset.corner_idx, origin, cell_size);
    % plot arm
    conf = plot_values.atVector(symbol('x', i));
%     plotArm(arm, conf, 'b', 2)
    plotRobotModel(arm, conf);
    % plot config
    set3DPlotRange(dataset)
    grid on
    pause(0.01)
end
hold off



