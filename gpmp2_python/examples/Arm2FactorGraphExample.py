import numpy as np
from gtsam import *
from gpmp2 import *
import matplotlib.pyplot as plt

from gpmp2_python.datasets.generate2Ddataset import generate2Ddataset
from gpmp2_python.robots.generateArm import generateArm
from gpmp2_python.utils.plot_utils import *
from gpmp2_python.utils.signedDistanceField2D import signedDistanceField2D


# small dataset
dataset = generate2Ddataset("OneObstacleDataset")
rows = dataset.rows
cols = dataset.cols
cell_size = dataset.cell_size
origin_point2 = Point2(dataset.origin_x, dataset.origin_y)

# signed distance field
field = signedDistanceField2D(dataset.map, cell_size)
sdf = PlanarSDF(origin_point2, cell_size, field)


# plot sdf
figure1 = plt.figure(0)
axis1 = figure1.gca()  # for 3-d, set gca(projection='3d')
plotSignedDistanceField2D(
    figure1, axis1, field, dataset.origin_x, dataset.origin_y, dataset.cell_size
)

# settings
total_time_sec = 5.0
total_time_step = 10
total_check_step = 50
delta_t = total_time_sec / total_time_step
check_inter = total_check_step / total_time_step - 1

# use GP interpolation
use_GP_inter = False

# arm model
arm = generateArm("SimpleTwoLinksArm")

# GP
Qc = np.identity(2)
Qc_model = noiseModel_Gaussian.Covariance(Qc)

# Obstacle avoid settings
cost_sigma = 0.1
epsilon_dist = 0.2

# prior to start/goal
pose_fix = noiseModel_Isotropic.Sigma(2, 0.0001)
vel_fix = noiseModel_Isotropic.Sigma(2, 0.0001)

# start and end conf
start_conf = np.asarray([0, 0])
start_vel = np.asarray([0, 0])
end_conf = np.asarray([np.pi / 2, 0])
end_vel = np.asarray([0, 0])
avg_vel = (end_conf / total_time_step) / delta_t

# plot param
pause_time = total_time_sec / total_time_step

# plot start / end configuration
figure2 = plt.figure(1)
axis2 = figure2.gca()  # for 3-d, set gca(projection='3d')
plotEvidenceMap2D(
    figure2, axis2, dataset.map, dataset.origin_x, dataset.origin_y, cell_size
)
axis2.set_title("Layout")
plotPlanarArm(figure2, axis2, arm.fk_model(), start_conf, "b", 2)
plotPlanarArm(figure2, axis2, arm.fk_model(), end_conf, "r", 2)


# init optimization
graph = NonlinearFactorGraph()
init_values = Values()
graph_obs = NonlinearFactorGraph()

for i in range(0, total_time_step + 1):
    key_pos = symbol(ord("x"), i)  # TODO: check this mustafa
    key_vel = symbol(ord("v"), i)

    # initialize as straight line in conf space
    pose = (
        start_conf * (total_time_step - i) / total_time_step
        + end_conf * i / total_time_step
    )
    vel = avg_vel
    init_values.insert(key_pos, pose)
    init_values.insert(key_vel, vel)

    # start/end priors
    if i == 0:
        graph.push_back(PriorFactorVector(key_pos, start_conf, pose_fix))
        graph.push_back(PriorFactorVector(key_vel, start_vel, vel_fix))
    elif i == total_time_step:
        graph.push_back(PriorFactorVector(key_pos, end_conf, pose_fix))
        graph.push_back(PriorFactorVector(key_vel, end_vel, vel_fix))

    # GP priors and cost factor
    if i > 0:
        key_pos1 = symbol(ord("x"), i - 1)
        key_pos2 = symbol(ord("x"), i)
        key_vel1 = symbol(ord("v"), i - 1)
        key_vel2 = symbol(ord("v"), i)

        temp = GaussianProcessPriorLinear(
            key_pos1, key_vel1, key_pos2, key_vel2, delta_t, Qc_model
        )
        graph.push_back(temp)

        # cost factor
        graph.push_back(
            ObstaclePlanarSDFFactorArm(key_pos, arm, sdf, cost_sigma, epsilon_dist)
        )

        graph_obs.push_back(
            ObstaclePlanarSDFFactorArm(key_pos, arm, sdf, cost_sigma, epsilon_dist)
        )

        # GP cost factor
        if use_GP_inter and check_inter > 0:
            for j in range(1, check_inter + 1):
                tau = j * (total_time_sec / total_check_step)
                graph.add(
                    ObstaclePlanarSDFFactorGPArm(
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
                graph_obs.add(
                    ObstaclePlanarSDFFactorGPArm(
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

# optimize!
use_trustregion_opt = False

if use_trustregion_opt:
    parameters = DoglegParams()
    parameters.setVerbosity("ERROR")
    optimizer = DoglegOptimizer(graph, init_values, parameters)
else:
    parameters = GaussNewtonParams()
    parameters.setRelativeErrorTol(1e-5)
    # Do not perform more than N iteration steps
    parameters.setMaxIterations(100)
    # Create the optimizer ...
    parameters.setVerbosity("ERROR")
    optimizer = GaussNewtonOptimizer(graph, init_values, parameters)

print("Initial Error = %d\n", graph.error(init_values))
print("Initial Collision Cost: %d\n", graph_obs.error(init_values))

optimizer.optimizeSafely()
result = optimizer.values()

print("Final Error = %d\n", graph.error(result))
print("FInal Collision Cost: %d\n", graph_obs.error(result))

# plot final values
figure3 = plt.figure(2)
axis3 = figure3.gca()
# plot world
plotEvidenceMap2D(
    figure3, axis3, dataset.map, dataset.origin_x, dataset.origin_y, cell_size
)
for i in range(total_time_step + 1):
    axis3.set_title("Optimized Values")
    # plot arm
    conf = result.atVector(symbol(ord("x"), i))
    plotPlanarArm(figure3, axis3, arm.fk_model(), conf, "b", 2)
    plotRobotModel2D(figure3, axis3, arm, conf)
    plt.pause(pause_time)

plt.show()
