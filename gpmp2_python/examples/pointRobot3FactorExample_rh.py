import numpy as np
from gtsam import *
from gpmp2 import *
import matplotlib.pyplot as plt

from gpmp2_python.datasets.generate2Ddataset import generate2Ddataset
from gpmp2_python.robots.generateArm import generateArm
from gpmp2_python.utils.plot_utils import *
from gpmp2_python.utils.signedDistanceField2D import signedDistanceField2D


def get_plan(start_conf_val, start_vel, end_conf_val, end_vel, sdf, params):

    start_conf = start_conf_val
    end_conf = end_conf_val

    avg_vel = (end_conf_val - start_conf_val / params.total_time_step) / params.delta_t

    # plot param

    # init optimization
    graph = NonlinearFactorGraph()
    init_values = Values()

    for i in range(0, params.total_time_step + 1):
        key_pos = symbol(ord("x"), i)
        key_vel = symbol(ord("v"), i)

        #% initialize as straight line in conf space
        # pose = Pose2(start_conf_val * float(params.total_time_step-i)/float(params.total_time_step) + end_conf_val * i/float(params.total_time_step))
        pose = start_conf_val
        vel = avg_vel

        init_values.insert(key_pos, pose)
        init_values.insert(key_vel, vel)

        #% start/end priors
        if i == 0:
            graph.push_back(PriorFactorVector(key_pos, start_conf, params.pose_fix))
            graph.push_back(PriorFactorVector(key_vel, start_vel, params.vel_fix))

        graph.add(VehicleDynamicsFactorVector(key_pos, key_vel, 0.001))

        # GP priors and cost factor
        if i > 0:
            graph.push_back(PriorFactorVector(key_pos, end_conf, params.pose_fix_model))
            graph.push_back(PriorFactorVector(key_vel, end_vel, params.vel_fix_model))
            key_pos1 = symbol(ord("x"), i - 1)
            key_pos2 = symbol(ord("x"), i)
            key_vel1 = symbol(ord("v"), i - 1)
            key_vel2 = symbol(ord("v"), i)

            temp = GaussianProcessPriorLinear(
                key_pos1, key_vel1, key_pos2, key_vel2, params.delta_t, params.Qc_model
            )
            graph.push_back(temp)

            #% cost factor
            graph.push_back(
                ObstaclePlanarSDFFactorPointRobot(
                    key_pos,
                    params.pR_model,
                    sdf,
                    params.cost_sigma,
                    params.epsilon_dist,
                )
            )

            #% GP cost factor
            if params.use_GP_inter and params.check_inter > 0:
                for j in range(1, params.check_inter + 1):
                    tau = j * (params.total_time_sec / params.total_check_step)
                    graph.add(
                        ObstaclePlanarSDFFactorGPPointRobot(
                            key_pos1,
                            key_vel1,
                            key_pos2,
                            key_vel2,
                            params.pR_model,
                            sdf,
                            params.cost_sigma,
                            params.epsilon_dist,
                            params.Qc_model,
                            params.delta_t,
                            tau,
                        )
                    )

    if params.use_trustregion_opt:
        parameters = DoglegParams()
        # parameters.setVerbosity('ERROR')
        optimizer = DoglegOptimizer(graph, init_values, parameters)
    else:
        parameters = GaussNewtonParams()
        # parameters.setVerbosity('ERROR')
        optimizer = GaussNewtonOptimizer(graph, init_values, parameters)

    print("Initial Error = %d\n", graph.error(init_values))

    optimizer.optimizeSafely()
    result = optimizer.values()

    print("Final Error = %d\n", graph.error(result))

    res_flag = True
    if graph.error(result) > params.acceptable_error_threshold:
        res_flag = False
    return result, res_flag


def get_sdf(occ_grid_topic):  # TODO this will change to ros occupancy grid processonr
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

    return sdf, dataset


class Parameters(object):  # TODO: read from yaml file or rosparams
    # settings
    total_time_sec = 2.5
    total_time_step = 5
    total_check_step = 10.0
    delta_t = total_time_sec / total_time_step
    check_inter = int(total_check_step / total_time_step - 1)

    use_GP_inter = True

    # point robot model
    pR = PointRobot(3, 1)
    spheres_data = np.asarray([0.0, 0.0, 0.0, 0.0, 1.5])
    nr_body = spheres_data.shape[0]
    sphere_vec = BodySphereVector()
    sphere_vec.push_back(
        BodySphere(spheres_data[0], spheres_data[4], Point3(spheres_data[1:4]))
    )
    pR_model = PointRobotModel(pR, sphere_vec)

    # GP
    Qc = np.identity(pR_model.dof())
    Qc_model = noiseModel_Gaussian.Covariance(Qc)

    # Obstacle avoid settings
    cost_sigma = 0.2
    epsilon_dist = 4.0

    # prior to start/goal
    pose_fix = pose_fix_model = noiseModel_Isotropic.Sigma(pR_model.dof(), 0.0001)
    vel_fix = vel_fix_model = noiseModel_Isotropic.Sigma(pR_model.dof(), 0.0001)

    use_trustregion_opt = True

    pause_time = total_time_sec / total_time_step

    # Fixed window params
    goal_region_threshold = 0.1
    acceptable_error_threshold = 200
    sigma_goal = 1


def get_robot_state(result):  # todo: this will change to ros subscriber.

    conf = result.atVector(symbol(ord("x"), 1))
    vel = result.atVector(symbol(ord("v"), 1))
    return conf, vel


def get_robot_action(result):  # TODO: Get action from result and command therobot
    action_vel = None
    action_vel_traj = None
    return action_vel, action_vel_traj


def command_robot(action_vel, action_vel_traj):  # TODO: this should be non-blocking
    pass


def plot_path(result, axis, params):
    x = []
    y = []
    for i in range(params.total_time_step + 1):
        conf = result.atVector(symbol(ord("x"), i))
        x.append(conf[0])
        y.append(conf[1])
    (handle,) = axis.plot(x, y, "s")
    return handle


def main():

    sdf, dataset = get_sdf(occ_grid_topic=None)
    params = Parameters()

    # start and end conf
    start_conf_val = curstate_val = np.asarray([0, 0, 0])
    start_vel = curstate_vel = np.asarray([0, 0, 0])
    end_conf_val = np.asarray([17, 14, 0.1])
    end_vel = np.asarray([0, 0, 0])

    ## Plots
    figure = plt.figure(1)
    axis = figure.gca()
    plotEvidenceMap2D(
        figure, axis, dataset.map, dataset.origin_x, dataset.origin_y, dataset.cell_size
    )

    init_distance = np.linalg.norm(curstate_val - end_conf_val)
    graph_handle = None
    while np.linalg.norm(curstate_val - end_conf_val) > params.goal_region_threshold:
        # Goal prior factors
        params.pose_fix_model = noiseModel_Isotropic.Sigma(
            3,
            params.sigma_goal
            * np.linalg.norm(curstate_val - end_conf_val)
            / init_distance,
        )
        params.vel_fix_model = noiseModel_Isotropic.Sigma(
            3,
            params.sigma_goal
            * np.linalg.norm(curstate_val - end_conf_val)
            / init_distance,
        )

        result, res_flag = get_plan(
            curstate_val, curstate_vel, end_conf_val, end_vel, sdf, params
        )
        graph_handle = plot_path(result, axis, params)

        action_vel, action_vel_traj = get_robot_action(result)
        command_robot(action_vel, action_vel_traj)

        curstate_val, blah = get_robot_state(result)
        print(curstate_val, curstate_vel)

        ## Plots
        plotPointRobot2D_theta(figure, axis, params.pR_model, curstate_val)
        plt.pause(params.pause_time)

        # raw_input("Press Enter to continue...")

        if graph_handle is not None:
            graph_handle.remove()

    plt.show()


if __name__ == "__main__":
    main()
