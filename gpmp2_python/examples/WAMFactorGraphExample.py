import numpy as np
from gtsam import *
from gpmp2 import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D


from gpmp2_python.datasets.generate3Ddataset import generate3Ddataset
from gpmp2_python.robots.generateArm import generateArm
from gpmp2_python.utils.plot_utils import *
from gpmp2_python.utils.signedDistanceField3D import signedDistanceField3D


# dataset
dataset = generate3Ddataset("WAMDeskDataset")
origin = np.asarray([dataset.origin_x, dataset.origin_y, dataset.origin_z])
origin_point3 = Point3(origin)
cell_size = dataset.cell_size

# sdf
print("calculating signed distance field ...")
field = signedDistanceField3D(dataset.map, dataset.cell_size)
print("calculating signed distance field done")

# arm: WAM arm
arm = generateArm("WAMArm")

start_conf = np.asarray([-0.8, -1.70, 1.64, 1.29, 1.1, -0.106, 2.2])
end_conf = np.asarray([-0.0, 0.94, 0, 1.6, 0, -0.919, 1.55])
start_vel = np.zeros(7)
end_vel = np.zeros(7)

# plot problem setting
figure0 = plt.figure(0)
axis0 = Axes3D(figure0)
axis0.set_title("Problem Settings")
set3DPlotRange(figure0, axis0, dataset)
plotRobotModel(figure0, axis0, arm, start_conf)
plotRobotModel(figure0, axis0, arm, end_conf)
plotMap3D(figure0, axis0, dataset.corner_idx, origin, cell_size)


## settings
total_time_sec = 2.0
total_time_step = 10
total_check_step = 100
delta_t = total_time_sec / total_time_step
check_inter = total_check_step / total_time_step - 1

# GP
Qc = np.identity(7)
Qc_model = noiseModel_Gaussian.Covariance(Qc)

# algo settings
cost_sigma = 0.02
epsilon_dist = 0.2

# noise model
fix_sigma = 0.0001
pose_fix_model = noiseModel_Isotropic.Sigma(7, fix_sigma)
vel_fix_model = noiseModel_Isotropic.Sigma(7, fix_sigma)

# init sdf
sdf = SignedDistanceField(
    origin_point3, cell_size, field.shape[0], field.shape[1], field.shape[2]
)
for z in range(field.shape[2]):
    sdf.initFieldData(
        z, field[:, :, z]
    )  # TODO: check this line with its matlab counterpart

#% plot settings
plot_inter_traj = False
plot_inter = 4
if plot_inter_traj:
    total_plot_step = total_time_step * (plot_inter + 1)
else:
    total_plot_step = total_time_step
pause_time = total_time_sec / total_plot_step


## initial traj
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step)

# plot initial traj
if plot_inter_traj:
    plot_values = interpolateArmTraj(init_values, Qc_model, delta_t, plot_inter)
else:
    plot_values = init_values

# plot init values
figure1 = plt.figure(1)
axis1 = Axes3D(figure1)
axis1.set_title("Initial Values")
# plot world
plotMap3D(figure1, axis1, dataset.corner_idx, origin, cell_size)
set3DPlotRange(figure1, axis1, dataset)
for i in range(total_plot_step):
    conf = plot_values.atVector(symbol(ord("x"), i))
    plotArm(figure1, axis1, arm.fk_model(), conf, "b", 2)
    plt.pause(pause_time)


## init optimization
graph = NonlinearFactorGraph()
graph_obs = NonlinearFactorGraph()

for i in range(total_time_step + 1):
    key_pos = symbol(ord("x"), i)
    key_vel = symbol(ord("v"), i)

    # priors
    if i == 0:
        graph.push_back(PriorFactorVector(key_pos, start_conf, pose_fix_model))
        graph.push_back(PriorFactorVector(key_vel, start_vel, vel_fix_model))
    elif i == total_time_step:
        graph.push_back(PriorFactorVector(key_pos, end_conf, pose_fix_model))
        graph.push_back(PriorFactorVector(key_vel, end_vel, vel_fix_model))

    # GP priors and cost factor
    if i > 0:
        key_pos1 = symbol(ord("x"), i - 1)
        key_pos2 = symbol(ord("x"), i)
        key_vel1 = symbol(ord("v"), i - 1)
        key_vel2 = symbol(ord("v"), i)
        graph.push_back(
            GaussianProcessPriorLinear(
                key_pos1, key_vel1, key_pos2, key_vel2, delta_t, Qc_model
            )
        )

        # cost factor
        graph.push_back(
            ObstacleSDFFactorArm(key_pos, arm, sdf, cost_sigma, epsilon_dist)
        )
        graph_obs.push_back(
            ObstacleSDFFactorArm(key_pos, arm, sdf, cost_sigma, epsilon_dist)
        )

        # GP cost factor
        if check_inter > 0:
            for j in range(1, check_inter + 1):
                tau = j * (total_time_sec / total_check_step)
                graph.push_back(
                    ObstacleSDFFactorGPArm(
                        key_pos1,
                        key_vel1,
                        key_pos2,
                        key_vel2,
                        arm,
                        sdf,
                        cost_sigma,
                        epsilon_dist,
                        Qc_model,
                        delta_t,
                        tau,
                    )
                )
                graph_obs.push_back(
                    ObstacleSDFFactorGPArm(
                        key_pos1,
                        key_vel1,
                        key_pos2,
                        key_vel2,
                        arm,
                        sdf,
                        cost_sigma,
                        epsilon_dist,
                        Qc_model,
                        delta_t,
                        tau,
                    )
                )

## optimize!
use_LM = False
use_trustregion_opt = True

if use_LM:
    parameters = LevenbergMarquardtParams()  # Todo: check why this fails
    parameters.setVerbosity("ERROR")
    # parameters.setVerbosityLM('LAMBDA');
    parameters.setlambdaInitial(1000.0)
    optimizer = LevenbergMarquardtOptimizer(graph, init_values, parameters)
elif use_trustregion_opt:
    parameters = DoglegParams()
    parameters.setVerbosity("ERROR")
    optimizer = DoglegOptimizer(graph, init_values, parameters)
else:
    parameters = GaussNewtonParams()
    parameters.setVerbosity("ERROR")
    optimizer = GaussNewtonOptimizer(graph, init_values, parameters)


print("Initial Error = %d\n", graph.error(init_values))
print("Initial Collision Cost: %d\n", graph_obs.error(init_values))


optimizer.optimizeSafely()
result = optimizer.values()

print("Error = %d\n", graph.error(result))
print("Collision Cost End: %d\n", graph_obs.error(result))

# plot results
if plot_inter_traj:
    plot_values = interpolateArmTraj(result, Qc_model, delta_t, plot_inter)
else:
    plot_values = result


# plot final values
figure2 = plt.figure(2)
axis2 = Axes3D(figure2)
axis2.set_title("Result Values")
plotMap3D(figure2, axis2, dataset.corner_idx, origin, cell_size)
set3DPlotRange(figure2, axis2, dataset)
for i in range(total_plot_step):
    conf = plot_values.atVector(symbol(ord("x"), i))
    plotArm(figure2, axis2, arm.fk_model(), conf, "b", 2)
    plt.pause(pause_time)


# plot final values
figure3 = plt.figure(3)
axis3 = Axes3D(figure3)
axis3.set_title("Result Values")
plotMap3D(figure3, axis3, dataset.corner_idx, origin, cell_size)
set3DPlotRange(figure3, axis3, dataset)
for i in range(total_plot_step):
    conf = plot_values.atVector(symbol(ord("x"), i))
    plotRobotModel(figure3, axis3, arm, conf)
    plt.pause(pause_time)

plt.show()
