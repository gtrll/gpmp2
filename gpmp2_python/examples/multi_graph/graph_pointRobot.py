import numpy as np
from gtsam import *
from gpmp2 import *

import matplotlib.pyplot as plt
from gpmp2_python.datasets.generate2Ddataset import generate2Ddataset
from gpmp2_python.utils.signedDistanceField2D import signedDistanceField2D
from gpmp2_python.utils.plot_utils import *
from graph_utils import *
import time


if __name__ == "__main__":

    problem = Problem()
    problem.use_GP_inter = True
    problem.gp_factor_function = GaussianProcessPriorLinear
    problem.obstacle_factor_function = ObstaclePlanarSDFFactorPointRobot
    problem.obstalce_gp_factor_function = ObstaclePlanarSDFFactorGPPointRobot

    problem.dataset = generate2Ddataset("MultiObstacleDataset")

    cell_size = problem.dataset.cell_size
    origin_point2 = Point2(problem.dataset.origin_x, problem.dataset.origin_y)

    # Signed Distance field
    field = signedDistanceField2D(problem.dataset.map, cell_size)
    problem.sdf = PlanarSDF(origin_point2, cell_size, field)

    # settings
    problem.total_time_sec = 10.0
    problem.total_time_step = 20
    problem.total_check_step = 50.0
    problem.delta_t = problem.total_time_sec / problem.total_time_step
    problem.check_inter = int(problem.total_check_step / problem.total_time_step - 1)

    # point robot model
    pR = PointRobot(2, 1)
    spheres_data = np.asarray([0.0, 0.0, 0.0, 0.0, 1.5])
    nr_body = spheres_data.shape[0]
    sphere_vec = BodySphereVector()
    sphere_vec.push_back(
        BodySphere(spheres_data[0], spheres_data[4], Point3(spheres_data[1:4]))
    )
    problem.gpmp_robot = PointRobotModel(pR, sphere_vec)

    # GP
    problem.Qc = np.identity(2)
    problem.Qc_model = noiseModel_Gaussian.Covariance(problem.Qc)

    # Obstacle avoid settings
    problem.cost_sigma = 0.5
    problem.epsilon_dist = 4.0

    # prior to start/goal
    problem.pose_fix_model = noiseModel_Isotropic.Sigma(2, 0.0001)
    problem.vel_fix_model = noiseModel_Isotropic.Sigma(2, 0.0001)

    # start and end conf
    problem.start_conf = np.asarray([0, 0])
    problem.start_vel = np.asarray([0, 0])
    problem.end_conf = np.asarray([17, 14])
    problem.end_vel = np.asarray([0, 0])
    problem.avg_vel = (problem.end_conf / problem.total_time_step) / problem.delta_t

    # plot param
    problem.pause_time = problem.total_time_sec / problem.total_time_step

    inits = get_initializations(4, problem)
    print(inits)

    problem.dropout_prob = 0.5
    problem.seed_val = 1
    planner_graph = get_planner_graph(inits, problem)

    gtsam_graph, init_values = get_gtsam_graph(planner_graph, problem)

    use_trustregion_opt = True

    if use_trustregion_opt:
        parameters = DoglegParams()
        # parameters.setVerbosity('ERROR')
        optimizer = DoglegOptimizer(gtsam_graph, init_values, parameters)
    else:
        parameters = GaussNewtonParams()
        # parameters.setRelativeErrorTol(1e-5)
        # parameters.setMaxIterations(100)
        # parameters.setVerbosity('ERROR')
        optimizer = GaussNewtonOptimizer(gtsam_graph, init_values, parameters)

    print("Initial Error = %d\n", gtsam_graph.error(init_values))

    optimizer.optimizeSafely()
    result = optimizer.values()

    print("Final Error = %d\n", gtsam_graph.error(result))

    update_planner_graph(result, planner_graph)

    planner = Planner(result, gtsam_graph, planner_graph)
    path = planner.get_shortest_path()
    print(path)

    ## plot final values
    figure = plt.figure()
    axis = figure.gca()
    # plot world
    plotEvidenceMap2D(
        figure,
        axis,
        problem.dataset.map,
        problem.dataset.origin_x,
        problem.dataset.origin_y,
        cell_size,
    )
    for i in range(problem.total_time_step + 1):
        axis.set_title("Optimized Values")
        # plot arm
        conf = path[i]
        # conf = result.atVector(symbol(ord('x'), i))
        plotPointRobot2D(figure, axis, problem.gpmp_robot, conf)
        plt.pause(problem.pause_time)
