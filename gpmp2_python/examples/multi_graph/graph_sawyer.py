import numpy as np
from gtsam import *
from gpmp2 import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D  # <-- Note the capitalization!
from gpmp_utils.generate3Ddataset import generate3Ddataset
from gpmp_utils.signedDistanceField3D import signedDistanceField3D
from gpmp_utils.generateArm import generateArm
from gpmp_utils.plotMap3D import plotMap3D
from gpmp_utils.plotRobotModel import plotRobotModel
from gpmp_utils.set3DPlotRange import set3DPlotRange
from gpmp_utils.plotArm import plotArm


from pyrobot import Robot
from graph_utils import *
import time


if __name__ == "__main__":

    problem = Problem()
    problem.use_GP_inter = True
    problem.gp_factor_function = GaussianProcessPriorLinear
    problem.obstacle_factor_function = ObstacleSDFFactorArm
    problem.obstalce_gp_factor_function = ObstacleSDFFactorGPArm

    problem.dataset = generate3Ddataset("WAMDeskDataset")
    origin = np.asarray(
        [problem.dataset.origin_x, problem.dataset.origin_y, problem.dataset.origin_z]
    )

    # dataset
    origin_point3 = Point3(origin)
    cell_size = problem.dataset.cell_size

    # sdf
    print("calculating signed distance field ...")
    field = signedDistanceField3D(problem.dataset.map, problem.dataset.cell_size)

    # init sdf
    problem.sdf = SignedDistanceField(
        origin_point3, cell_size, field.shape[0], field.shape[1], field.shape[2]
    )
    for z in range(field.shape[2]):
        problem.sdf.initFieldData(z, field[:, :, z])
    print("calculating signed distance field done")

    # arm: WAM arm
    problem.gpmp_robot = generateArm("SAWYERArm")

    # Make PyRobot Object
    robot = Robot("sawyer")
    robot.arm.go_home()
    problem.start_conf = robot.arm.get_joint_angles()
    # start_conf[0] = np.pi/2

    robot.arm.move_to_neutral()
    problem.end_conf = robot.arm.get_joint_angles()
    # end_conf[0] = np.pi/2
    problem.start_vel = np.zeros(7)
    problem.end_vel = np.zeros(7)

    # plot problem setting
    figure0 = plt.figure(0)
    axis0 = Axes3D(figure0)
    axis0.set_title("Problem Settings")
    set3DPlotRange(figure0, axis0, problem.dataset)
    plotRobotModel(figure0, axis0, problem.gpmp_robot, problem.start_conf)
    plotRobotModel(figure0, axis0, problem.gpmp_robot, problem.end_conf)
    plotMap3D(figure0, axis0, problem.dataset.corner_idx, origin, cell_size)

    ## settings
    problem.total_time_sec = 2.0
    problem.total_time_step = 10
    problem.total_check_step = 100
    problem.delta_t = problem.total_time_sec / problem.total_time_step
    problem.check_inter = problem.total_check_step / problem.total_time_step - 1
    problem.avg_vel = (problem.end_conf / problem.total_time_step) / problem.delta_t

    # GP
    problem.Qc = np.identity(7)
    problem.Qc_model = noiseModel_Gaussian.Covariance(problem.Qc)

    # algo settings
    problem.cost_sigma = 0.02
    problem.epsilon_dist = 0.2

    # noise model
    problem.fix_sigma = 0.0001
    problem.pose_fix_model = noiseModel_Isotropic.Sigma(7, problem.fix_sigma)
    problem.vel_fix_model = noiseModel_Isotropic.Sigma(7, problem.fix_sigma)

    #% plot settings
    plot_inter_traj = False
    plot_inter = 4
    if plot_inter_traj:
        total_plot_step = problem.total_time_step * (plot_inter + 1)
    else:
        total_plot_step = problem.total_time_step
    problem.pause_time = problem.total_time_sec / total_plot_step

    start = time.time()
    inits = get_initializations(100, problem)
    # print(inits)

    problem.dropout_prob = 0.5
    problem.seed_val = 1
    planner_graph = get_planner_graph(inits, problem)

    gtsam_graph, init_values = get_gtsam_graph(planner_graph, problem)
    end = time.time()
    print("Time taken to build planner graph:", end - start)

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

    start = time.time()
    optimizer.optimizeSafely()
    end = time.time()
    print("Time taken to optimize:", end - start)

    result = optimizer.values()

    print("Final Error = %d\n", gtsam_graph.error(result))

    start = time.time()
    update_planner_graph(result, planner_graph)
    planner = Planner(result, gtsam_graph, planner_graph)

    path = planner.get_shortest_path()
    end = time.time()
    print("Time taken to plan:", end - start)

    # plot final values
    figure2 = plt.figure(2)
    axis2 = Axes3D(figure2)
    axis2.set_title("Result Values")
    plotMap3D(figure2, axis2, problem.dataset.corner_idx, origin, cell_size)
    set3DPlotRange(figure2, axis2, problem.dataset)
    for i in range(total_plot_step):
        conf = path[i]
        plotArm(figure2, axis2, problem.gpmp_robot.fk_model(), conf, "b", 2)
        plt.pause(problem.pause_time)

    plt.show()

    # ### Executing the Final Trajectory on Sawyer in Gazebo
    # robot.arm.set_joint_positions(start_conf)

    # from joint_trajectory_client_sawyer_ros import Trajectory

    # joint_names = [ 'right_j' + str(x) for x in range(7)]
    # action_name = '/robot/limb/right/follow_joint_trajectory'
    # traj = Trajectory(joint_names, action_name)

    # #Encode results into list
    # final_trajectory = []
    # for i in range(total_plot_step):
    #   conf = path[i]
    #   traj.add_point(conf, pause_time*5)
    #   final_trajectory.append(conf)

    # traj.start()
    # traj.wait(20) # TODO: Change this to wait till finish of trajectory execution
