import numpy as np
from gtsam import *
from gpmp2 import *
import matplotlib.pyplot as plt


from gpmp2_python.datasets.generate2Ddataset import generate2Ddataset
from gpmp2_python.robots.generateArm import generateArm
from gpmp2_python.utils.plot_utils import *
from gpmp2_python.utils.signedDistanceField2D import signedDistanceField2D


dataset = generate2Ddataset("MultiObstacleDataset")
rows = dataset.rows
cols = dataset.cols
cell_size = dataset.cell_size
origin_point2 = Point2(dataset.origin_x, dataset.origin_y)

# Signed Distance field
field = signedDistanceField2D(dataset.map, cell_size)
sdf = PlanarSDF(origin_point2, cell_size, field)

figure1 = plt.figure(0)
axis1 = figure1.gca()  # for 3-d, set gca(projection='3d')
plotSignedDistanceField2D(
    figure1, axis1, field, dataset.origin_x, dataset.origin_y, dataset.cell_size
)


# settings
total_time_sec = 10.0
total_time_step = 20
total_check_step = 50.0
delta_t = total_time_sec / total_time_step
check_inter = int(total_check_step / total_time_step - 1)

use_GP_inter = True


# point robot model
pR = PointRobot(2, 1)
spheres_data = np.asarray([0.0, 0.0, 0.0, 0.0, 1.5])
nr_body = spheres_data.shape[0]
sphere_vec = BodySphereVector()
sphere_vec.push_back(
    BodySphere(spheres_data[0], spheres_data[4], Point3(spheres_data[1:4]))
)
pR_model = PointRobotModel(pR, sphere_vec)

# GP
Qc = np.identity(2)
Qc_model = noiseModel_Gaussian.Covariance(Qc)

# Obstacle avoid settings
cost_sigma = 0.5
epsilon_dist = 4.0

# prior to start/goal
pose_fix = noiseModel_Isotropic.Sigma(2, 0.0001)
vel_fix = noiseModel_Isotropic.Sigma(2, 0.0001)


# start and end conf
start_conf = np.asarray([0, 0])
start_vel = np.asarray([0, 0])
end_conf = np.asarray([17, 14])
end_vel = np.asarray([0, 0])
avg_vel = (end_conf / total_time_step) / delta_t


# plot param
pause_time = total_time_sec / total_time_step


# init optimization
graph = NonlinearFactorGraph()
init_values = Values()


for i in range(0, total_time_step + 1):
    key_pos = symbol(ord("x"), i)
    key_vel = symbol(ord("v"), i)

    #% initialize as straight line in conf space
    pose = start_conf * float(total_time_step - i) / float(
        total_time_step
    ) + end_conf * i / float(total_time_step)
    vel = avg_vel
    print(pose)
    init_values.insert(key_pos, pose)
    init_values.insert(key_vel, vel)

    #% start/end priors
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

        #% cost factor
        graph.push_back(
            ObstaclePlanarSDFFactorPointRobot(
                key_pos, pR_model, sdf, cost_sigma, epsilon_dist
            )
        )

        #% GP cost factor
        if use_GP_inter and check_inter > 0:
            for j in range(1, check_inter + 1):
                tau = j * (total_time_sec / total_check_step)
                graph.add(
                    ObstaclePlanarSDFFactorGPPointRobot(
                        key_pos1,
                        key_vel1,
                        key_pos2,
                        key_vel2,
                        pR_model,
                        sdf,
                        cost_sigma,
                        epsilon_dist,
                        Qc_model,
                        delta_t,
                        tau,
                    )
                )

use_trustregion_opt = True

if use_trustregion_opt:
    parameters = DoglegParams()
    parameters.setVerbosity("ERROR")
    optimizer = DoglegOptimizer(graph, init_values, parameters)
else:
    parameters = GaussNewtonParams()
    # parameters.setRelativeErrorTol(1e-5)
    # parameters.setMaxIterations(100)
    parameters.setVerbosity("ERROR")
    optimizer = GaussNewtonOptimizer(graph, init_values, parameters)

print("Initial Error = %d\n", graph.error(init_values))


optimizer.optimizeSafely()
result = optimizer.values()

print("Final Error = %d\n", graph.error(result))


#%% plot final values
figure = plt.figure(1)
axis = figure.gca()
# plot world
plotEvidenceMap2D(
    figure, axis, dataset.map, dataset.origin_x, dataset.origin_y, cell_size
)
for i in range(total_time_step + 1):
    axis.set_title("Optimized Values")
    # plot arm
    conf = result.atVector(symbol(ord("x"), i))
    plotPointRobot2D(figure, axis, pR_model, conf)
    plt.pause(pause_time)


plt.show()
