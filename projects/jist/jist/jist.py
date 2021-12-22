import numpy as np
from gtsam import *
from gpmp2 import *
from random import seed
from random import Random
import copy
import math
import time
from utils.plot_utils import *
from utils.signedDistanceField2D import signedDistanceField2D


import numpy as np
from gtsam import *
from gpmp2 import *
import copy
import math
from abc import ABCMeta, abstractmethod


class test_status:
    COLLISION = 0
    TIMEOUT = 1
    INVALIDSTART = 2
    SUCCESS = 3


class Result(object):
    """docstring for Result"""

    def __init__(self):
        self.seed_val = None
        self.start_conf = None
        self.goal_conf = None
        self.obstacle_num = None

        self.iterations = None
        self.optimizer_time = None
        self.total_time = None

        self.distance = None

        self.status = None

        self.vels = []


class Problem:
    """General Interface to set the GPMP problem"""

    def __init__(self):

        self.gpmp_robot = None
        self.start_conf = None
        self.end_conf = None
        self.start_vel = None
        self.end_vel = None

        self.total_time_sec = None
        self.total_time_step = None
        self.total_check_step = None
        self.delta_t = None
        self.check_inter = None
        self.avg_vel = None

        self.Qc = None
        self.Qc_model = None

        self.cost_sigma = None
        self.epsilon_dist = None

        self.fix_sigma = None
        self.pose_fix_model = None
        self.vel_fix_model = None

        self.sdf = None

        self.seed_val = None
        self.dropout_prob = None

        self.use_GP_inter = None

        self.gp_factor_function = None
        self.obstacle_factor_function = None
        self.obstalce_gp_factor_function = None

        # Fixed window specific params
        self.window_size = None  # number of time steps in the window.
        self.init_fraction_length = (
            None  # number between 0,1 for deciding inti window length
        )

        self.same_state_init = None
        self.vehicle_dynamics = None


class Node(object):
    """docstring for Node"""

    def __init__(self, planner_id, pose, vel=None):
        self.planner_id = planner_id
        self.pose = pose
        self.vel = vel

        # A* search specific things
        self.visited = None
        self.parent_id = None

        # gtsam factor graph related stuff
        self.gt_graph_ob_id = None  # obstacle factor id at the node
        # key is planner_id and value is list of gtsam factor ids
        # here value list consists of gp factors and interpolation obstacle factors
        self.neighbours = {}

        # rrt specific variable
        self.depth = None

        self.goal_factor_cost = None

    def add_neighbour(self, node_key):
        if node_key in self.neighbours:
            print("The specified node is already a neighbour")
            return
        self.neighbours[node_key] = []

    def remove_neighbour(self, node_key):
        if node_key not in self.neighbours:
            print("The specified node is not a neighbour")
            return
        del self.neighbours[node_key]

    def __str__(self):
        return self.planner_id, self.neighbours.keys()


#### Dijkstra specific stuff

from Queue import PriorityQueue


class PlannerBase(object):
    """docstring for Graph"""

    def __init__(self, result, gtsam_graph, planner_graph):
        self.result = result
        self.gtsam_graph = gtsam_graph
        self.planner_graph = planner_graph

    def get_factor_error(self, gt_factor_id):

        return self.gtsam_graph.at(gt_factor_id).error(self.result)

    def get_edge_cost(self, first_idx, second_idx):
        cost = 0
        # add cost of gp and obstacle interpolation factors
        for gt_factor_id in self.planner_graph[first_idx].neighbours[second_idx]:
            cost += self.get_factor_error(gt_factor_id)
        # add cost of state obstacle factor
        
        cost += self.get_factor_error(self.planner_graph[second_idx].gt_graph_ob_id)
            
        cost += self.get_factor_error(self.planner_graph[second_idx].goal_factor_cost)
        return cost

    @abstractmethod
    def get_shortest_path(self, **kwargs):
        pass


def update_planner_graph(result, planner_graph):
    for key in planner_graph:
        planner_graph[key].pose = result.atVector(
            symbol(ord("x"), planner_graph[key].planner_id)
        )
        planner_graph[key].vel = result.atVector(
            symbol(ord("v"), planner_graph[key].planner_id)
        )


def print_graph(nodes):
    print("Graph:")
    for key in nodes:
        print(key, nodes[key].neighbours.keys())


from matplotlib import collections as mc


def plot_graph(nodes, axis):
    lines = []
    # go through each node
    for key in nodes:
        start_pose = nodes[key].pose
        for neigh_id in nodes[key].neighbours:
            end_pose = nodes[neigh_id].pose
            line = [(start_pose[0], start_pose[1]), (end_pose[0], end_pose[1])]
            lines.append(line)
    lc = mc.LineCollection(lines, colors=(0, 0, 0, 1.0), linewidths=0.5)
    axis.add_collection(lc)
    return lc


def plot_path(path, axis):
    x = []
    y = []
    for point in path:
        x.append(point[0])
        y.append(point[1])
    (handle,) = axis.plot(x, y)
    return handle


def get_initializations_simple(problem):

    inits = []
    inits.append(np.tile(problem.start_conf, (problem.window_size + 1, 1)))
    return inits


def make_point_robot(num_dof=2, radius=1.5):

    # point robot model
    pR = PointRobot(num_dof, 1)
    spheres_data = np.asarray([0.0, 0.0, 0.0, 0.0, radius])
    nr_body = spheres_data.shape[0]
    sphere_vec = BodySphereVector()
    sphere_vec.push_back(
        BodySphere(spheres_data[0], spheres_data[4], Point3(spheres_data[1:4]))
    )
    return PointRobotModel(pR, sphere_vec)


def get_local_frame_vels(temp_vel, yaw):
    # temp_vel is velocity in global frame.

    vel = np.asarray([1.0, 1.0, 1.0])

    vel[0] = temp_vel[0] * np.cos(yaw) + temp_vel[1] * np.sin(yaw)
    vel[1] = -1.0 * temp_vel[0] * np.sin(yaw) + temp_vel[1] * np.cos(yaw)
    vel[2] = temp_vel[2]

    return vel


# Useful for upsampling gpmp solution and plotting
def get_interpolated_points(curNode, nextNode, problem):

    values = Values()
    values.insert(symbol(ord("x"), 0), curNode.pose)
    values.insert(symbol(ord("v"), 0), curNode.vel)
    values.insert(symbol(ord("x"), 1), nextNode.pose)
    values.insert(symbol(ord("v"), 1), nextNode.vel)

    interpolated_values = interpolateArmTraj(
        values, problem.Qc_model, problem.delta_t, problem.inter_step
    )

    total_time_step = interpolated_values.size()
    points = []
    for i in range(total_time_step / 2):
        points.append(interpolated_values.atVector(symbol(ord("x"), i)))

    return points

def get_center(x, y, dataset):

    center = (
        np.asarray([y - dataset.origin_y, x - dataset.origin_x]) / dataset.cell_size
    )
    return center.astype(int)
    
def in_collision(pose, sq_size, field, dataset):
        # robot is approximated with a square that enguls the true robot.
        idxs = get_center(pose[0], pose[1], dataset)
        if (
            idxs[0] < 0
            or idxs[1] < 0
            or idxs[0] >= dataset.rows
            or idxs[1] >= dataset.cols
        ):
            return False

        if field[idxs[0], idxs[1]] < sq_size:
            return True

        return False


def make_problem_from_config(
    problem_config, start_conf, goal_conf, start_vel=None, goal_vel=None
):

    problem = Problem()

    problem.seed_val = problem_config.seed_val
    problem.time_out_time = problem_config.time_out_time

    # Robot
    problem.radius = problem_config.radius
    problem.vehicle_dynamics = problem_config.vehicle_dynamics
    problem.vehicle_model = problem_config.vehicle_model

    # Noise model
    problem.use_noise = problem_config.use_noise
    problem.sensor_noise = problem_config.sensor_noise
    problem.action_noise = problem_config.action_noise

    # Sensor/SDF
    problem.sdf_side = problem_config.sdf_side

    # Core GPMP config
    problem.use_GP_inter = problem_config.use_GP_inter
    problem.inter_step = problem_config.inter_step

    # GPMP-Factor related
    problem.use_vel_limit = problem_config.use_vel_limit
    problem.sigma_vel_limit = problem_config.sigma_vel_limit
    problem.cost_sigma = problem_config.cost_sigma
    problem.epsilon_dist = problem_config.epsilon_dist
    problem.sigma_goal_rh = problem_config.sigma_goal_rh
    problem.sigma_goal_costco = problem_config.sigma_goal_costco
    problem.sigma_start = problem_config.sigma_start
    problem.use_trustregion_opt = problem_config.use_trustregion_opt

    # Receding Horizon specific things
    problem.same_state_init = problem_config.same_state_init
    problem.use_prev_graph = problem_config.use_prev_graph
    problem.window_size = problem_config.node_budget
    problem.init_fraction_length = problem_config.init_fraction_length
    problem.goal_region_threshold = problem_config.goal_region_threshold
    problem.connection_threshold = problem_config.connection_threshold

    # RRT specific things
    problem.node_num = problem_config.node_budget
    problem.width = problem_config.width
    problem.height = problem_config.height
    problem.move_along_val = problem_config.move_along_val
    problem.move_along_val_theta = problem_config.move_along_val_theta

    # start and end conf
    problem.start_conf = np.asarray(start_conf)
    problem.end_conf = np.asarray(goal_conf)

    assert (
        problem.start_conf.shape == problem.end_conf.shape
    ), "Shape of the start does not match the goal"

    problem.num_dof = problem.start_conf.shape[0]

    if start_vel is None:
        problem.start_vel = np.zeros(problem.num_dof)
    if goal_vel is None:
        problem.end_vel = np.zeros(problem.num_dof)

    problem.gp_factor_function = GaussianProcessPriorLinear
    problem.obstacle_factor_function = ObstaclePlanarSDFFactorPointRobot
    problem.obstalce_gp_factor_function = ObstaclePlanarSDFFactorGPPointRobot

    problem.delta_t = problem_config.delta_t
    problem.avg_vel = problem_config.avg_vel * np.ones(problem.num_dof)

    # point robot model
    problem.gpmp_robot = make_point_robot(
        num_dof=problem.num_dof, radius=problem.radius
    )

    # GP
    problem.Qc = np.identity(problem.gpmp_robot.dof())
    problem.Qc_model = noiseModel_Gaussian.Covariance(problem.Qc)

    # Obstacle avoid settings

    # prior to start/goal

    problem.pose_fix_model = noiseModel_Isotropic.Sigma(
        problem.gpmp_robot.dof(), problem.sigma_goal_costco
    )
    problem.vel_fix_model = noiseModel_Isotropic.Sigma(
        problem.gpmp_robot.dof(), problem.sigma_goal_costco
    )

    problem.pose_fix_model_start = noiseModel_Isotropic.Sigma(
        problem.gpmp_robot.dof(), problem.sigma_start
    )
    problem.vel_fix_model_start = noiseModel_Isotropic.Sigma(
        problem.gpmp_robot.dof(), problem.sigma_start
    )

    problem.vel_limit_model = noiseModel_Isotropic.Sigma(
        problem.gpmp_robot.dof(), problem.sigma_vel_limit
    )

    problem.pause_time = problem.delta_t / float(
        problem.inter_step + 1
    )  # problem.delta_t

    # Id to start the 1st state on the planner graph with.
    # Note that this should alwasy be zero and shouldn't change.
    problem.planner_id = 0
    problem.curr_conf = problem.start_conf

    problem.costco_goal_scaling = problem_config.costco_goal_scaling

    return problem





random_rrt = Random()


def grow_rrt_planner_graph(nodes, problem):

    if problem.num_dof == 3:
        return grow_rrt_planner_graph_theta(nodes, problem)

    node_num = problem.node_num
    planner_id = problem.planner_id
    width = problem.width
    height = problem.height
    move_along_val = problem.move_along_val

    for i in range(len(nodes), node_num):

        # sample a random x,y point
        x = (problem.curr_conf[0] - width / 2) + width * random_rrt.random()
        y = (problem.curr_conf[1] - height / 2) + height * random_rrt.random()
        # Find the point closest to it on the tree

        min_dist = np.inf
        min_id = None
        for key in nodes:
            pose = nodes[key].pose
            if min_dist > np.linalg.norm([pose[0] - x, pose[1] - y]):
                min_dist = np.linalg.norm([pose[0] - x, pose[1] - y])
                min_id = key

        # Get new point and add it to the tree.

        unit_vect = np.asarray([x, y]) - nodes[min_id].pose
        unit_vect = unit_vect / (np.linalg.norm(unit_vect))
        new_x = nodes[min_id].pose[0] + move_along_val * unit_vect[0]
        new_y = nodes[min_id].pose[1] + move_along_val * unit_vect[1]

        # Add node to the tree
        planner_id += 1
        new_node = Node(planner_id, np.asarray([new_x, new_y]), problem.avg_vel)
        nodes[planner_id] = new_node
        nodes[min_id].add_neighbour(planner_id)
        nodes[planner_id].depth = nodes[min_id].depth + 1.0

    problem.planner_id = planner_id
    return nodes


def grow_rrt_planner_graph_theta(nodes, problem):

    node_num = problem.node_num
    planner_id = problem.planner_id
    width = problem.width
    height = problem.height
    move_along_val = problem.move_along_val
    move_along_val_theta = problem.move_along_val_theta

    for i in range(len(nodes), node_num):

        # sample a random x,y point
        x = (problem.curr_conf[0] - width / 2) + width * random_rrt.random()
        y = (problem.curr_conf[1] - height / 2) + height * random_rrt.random()
        theta = 2 * np.pi * random_rrt.random()
        # Find the point closest to it on the tree

        min_dist = np.inf
        min_id = None
        for key in nodes:
            pose = nodes[key].pose
            if min_dist > np.linalg.norm([pose[0] - x, pose[1] - y]):
                min_dist = np.linalg.norm([pose[0] - x, pose[1] - y])
                min_id = key

        # Get new point and add it to the tree.

        unit_vect = np.asarray([x, y]) - nodes[min_id].pose[0:2]
        unit_vect = unit_vect / (np.linalg.norm(unit_vect))
        new_x = nodes[min_id].pose[0] + move_along_val * unit_vect[0]
        new_y = nodes[min_id].pose[1] + move_along_val * unit_vect[1]

        if theta > nodes[min_id].pose[2]:
            new_theta = nodes[min_id].pose[2] + move_along_val_theta
        else:
            new_theta = nodes[min_id].pose[2] - move_along_val_theta

        # Add node to the tree
        planner_id += 1
        new_node = Node(
            planner_id, np.asarray([new_x, new_y, new_theta]), problem.avg_vel
        )
        nodes[planner_id] = new_node
        nodes[min_id].add_neighbour(planner_id)
        nodes[planner_id].depth = nodes[min_id].depth + 1.0

    problem.planner_id = planner_id
    return nodes


def get_rrt_planner_graph(problem):

    random_rrt.seed(problem.seed_val)
    nodes = {}  # a.k.a RRT tree
    planner_id = problem.planner_id

    start_node = Node(
        planner_id, problem.start_conf, problem.avg_vel
    )  # TODO: Check if its ave_vel
    nodes[planner_id] = start_node
    nodes[planner_id].depth = 0.0

    if problem.num_dof == 3:
        return grow_rrt_planner_graph_theta(nodes, problem)
    else:
        return grow_rrt_planner_graph(nodes, problem)


def get_gtsam_graph(node_list, problem):

    # init optimization
    graph = NonlinearFactorGraph()
    init_values = Values()

    # add all nodes
    for i in node_list:
        key_pos = symbol(ord("x"), i)
        key_vel = symbol(ord("v"), i)

        #% initialize as straight line in conf space
        init_values.insert(key_pos, node_list[i].pose)
        init_values.insert(key_vel, node_list[i].vel)

        #% start/end priors
        if i == problem.cur_planner_st_id:
            graph.push_back(
                PriorFactorVector(
                    key_pos, node_list[i].pose, problem.pose_fix_model_start
                )
            )
            graph.push_back(
                PriorFactorVector(
                    key_vel, problem.start_vel, problem.vel_fix_model_start
                )
            )

        if problem.vehicle_dynamics:
            graph.add(
                VehicleDynamicsFactorVector(key_pos, key_vel, problem.vehicle_model)
            )

        vel_limits = np.asarray([3.0, 3.0, 0.6]) #TODO: Move to hyperparams
        tresh = 1.0

        if problem.use_vel_limit:
            graph.push_back(
                VelocityLimitFactorVector(
                        key_vel,problem.vel_limit_model, vel_limits, tresh*np.ones(problem.num_dof)
                    )
                )

        if i != problem.cur_planner_st_id:

            graph.push_back(
                PriorFactorVector(key_pos, problem.end_conf, problem.pose_fix_model)
            )

            node_list[i].goal_factor_cost = graph.size() - 1

            graph.push_back(
                PriorFactorVector(key_vel, problem.end_vel, problem.vel_fix_model)
            )
            #% cost factor
            graph.push_back(
                problem.obstacle_factor_function(
                    key_pos,
                    problem.gpmp_robot,
                    problem.sdf,
                    problem.cost_sigma,
                    problem.epsilon_dist,
                )
            )

            node_list[i].gt_graph_ob_id = graph.size() - 1

        # add edges for each node

        for neigh_id in node_list[i].neighbours:
            key_pos1 = symbol(ord("x"), i)
            key_pos2 = symbol(ord("x"), neigh_id)
            key_vel1 = symbol(ord("v"), i)
            key_vel2 = symbol(ord("v"), neigh_id)

            graph.push_back(
                problem.gp_factor_function(
                    key_pos1,
                    key_vel1,
                    key_pos2,
                    key_vel2,
                    problem.delta_t,
                    problem.Qc_model,
                )
            )
            node_list[i].neighbours[neigh_id].append(graph.size() - 1)

            #% GP cost factor
            if problem.use_GP_inter and problem.inter_step > 0:
                for j in range(1, problem.inter_step + 1):
                    tau = j * (problem.delta_t / float(problem.inter_step + 1))
                    graph.push_back(
                        problem.obstalce_gp_factor_function(
                            key_pos1,
                            key_vel1,
                            key_pos2,
                            key_vel2,
                            problem.gpmp_robot,
                            problem.sdf,
                            problem.cost_sigma,
                            problem.epsilon_dist,
                            problem.Qc_model,
                            problem.delta_t,
                            tau,
                        )
                    )
                    node_list[i].neighbours[neigh_id].append(graph.size() - 1)
    return graph, init_values


def pruned_graph(cur_planner_id, nodes):
    # dfs through the graph and make new node list and terminal list
    new_nodes = {}
    new_terminal_node_ids = []

    def dfs_(cur_id):
        if cur_id in new_nodes or cur_id == None:
            return

        depth = nodes[cur_id].depth
        new_nodes[cur_id] = Node(
            nodes[cur_id].planner_id, nodes[cur_id].pose, nodes[cur_id].vel
        )
        new_nodes[cur_id].depth = depth

        for neigh_id in nodes[cur_id].neighbours:
            new_nodes[cur_id].add_neighbour(neigh_id)
            dfs_(neigh_id)

    dfs_(cur_planner_id)

    return new_nodes


#### Dijkstra specific stuff

from Queue import PriorityQueue


class PlannerRRT(PlannerBase):
    """docstring for Planner"""

    def __init__(self, result, gtsam_graph, planner_graph):
        super(PlannerRRT, self).__init__(result, gtsam_graph, planner_graph)

    def get_shortest_path(self, start_id, goal_pose):

        start_dept = self.planner_graph[start_id].depth
        terminal_node_ids = []
        terminal_node_costs = []

        cur_id = start_id
        priority_q = PriorityQueue()
        priority_q.put((0, cur_id))
        self.planner_graph[cur_id].visited = True
        while priority_q.qsize() > 0:
            cur_cost, cur_id = priority_q.get()

            # Terminal node!!
            if not self.planner_graph[cur_id].neighbours:
                terminal_node_ids.append(cur_id)

                #cost = cur_cost / (self.planner_graph[cur_id].depth - start_dept) ** 2
                cost = cur_cost / abs(self.planner_graph[cur_id].depth - start_dept)
                # cost += (np.linalg.norm(self.planner_graph[cur_id].pose - goal_pose))**2
                # cost += problem.costco_goal_scaling* np.linalg.norm(self.planner_graph[cur_id].pose - goal_pose)
                terminal_node_costs.append(cost)

            for neigh_id in self.planner_graph[cur_id].neighbours:
                if self.planner_graph[neigh_id].visited == True:
                    continue
                cost = self.get_edge_cost(cur_id, neigh_id)
                priority_q.put((cur_cost + cost, neigh_id))
                self.planner_graph[neigh_id].visited = True
                self.planner_graph[neigh_id].parent_id = cur_id

        cur_id = terminal_node_ids[terminal_node_costs.index(min(terminal_node_costs))]
        path = []
        next_planner_st_id = None
        while cur_id != start_id:
            path.append(self.planner_graph[cur_id].pose)
            parent_id = self.planner_graph[cur_id].parent_id
            if parent_id == start_id:
                next_planner_st_id = cur_id
            cur_id = parent_id

        path.append(self.planner_graph[cur_id].pose)
        path.reverse()
        return path, next_planner_st_id



    def get_shortest_path_and_vels(self, start_id, goal_pose, problem):

        start_dept = self.planner_graph[start_id].depth
        terminal_node_ids = []
        terminal_node_costs = []

        cur_id = start_id
        priority_q = PriorityQueue()
        priority_q.put((0, cur_id))
        self.planner_graph[cur_id].visited = True
        while priority_q.qsize() > 0:
            cur_cost, cur_id = priority_q.get()

            # Terminal node!!
            if not self.planner_graph[cur_id].neighbours:
                terminal_node_ids.append(cur_id)

                cost = cur_cost / abs(self.planner_graph[cur_id].depth - start_dept)
                # cost += problem.costco_goal_scaling* np.linalg.norm(self.planner_graph[cur_id].pose - goal_pose)

                #cost += self.get_factor_error(self.planner_graph[cur_id].goal_factor_cost)
                terminal_node_costs.append(cost)

            for neigh_id in self.planner_graph[cur_id].neighbours:
                if self.planner_graph[neigh_id].visited == True:
                    continue
                cost = self.get_edge_cost(cur_id, neigh_id)
                priority_q.put((cur_cost + cost, neigh_id))
                self.planner_graph[neigh_id].visited = True
                self.planner_graph[neigh_id].parent_id = cur_id

        cur_id = terminal_node_ids[terminal_node_costs.index(min(terminal_node_costs))]
        path = []
        vels = []
        next_planner_st_id = None
        while cur_id != start_id:
            path.append(self.planner_graph[cur_id].pose)
            vels.append(self.planner_graph[cur_id].vel)

            parent_id = self.planner_graph[cur_id].parent_id
            if parent_id == start_id:
                next_planner_st_id = cur_id
            cur_id = parent_id

        path.append(self.planner_graph[cur_id].pose)
        vels.append(self.planner_graph[cur_id].vel)
        path.reverse()
        vels.reverse()
        return path, vels, next_planner_st_id

import cv2


def rrt_chain(start_conf, goal_conf, dataset, problem, plot_mode="debug"):

    random_noise = Random()
    random_noise.seed(problem.seed_val + 1)

    sdf_side = problem.sdf_side

    prog_start_time = time.time()
    optimizer_time = 0.0

    result = Result()

    # Signed Distance field
    # temp_dataset = dataset.get_dataset()
    temp_dataset = dataset.get_dataset(start_conf, [sdf_side, sdf_side])
    field = signedDistanceField2D(temp_dataset.map, temp_dataset.cell_size)
    origin_point2 = Point2(temp_dataset.origin_x, temp_dataset.origin_y)
    problem.sdf = PlanarSDF(origin_point2, temp_dataset.cell_size, field)

    # Start state validity
    if in_collision(start_conf, problem.radius, field, temp_dataset):  # diameter of the robot
        return test_status.INVALIDSTART, np.inf, np.inf, np.inf, np.inf, [], np.inf

    planner_graph = get_rrt_planner_graph(problem)

    curstate = problem.start_conf
    cur_planner_st_id = 0  # start state!
    problem.cur_planner_st_id = cur_planner_st_id

    iterations = 0
    distance = 0.0
    total_time = 0.0
    init_distance = np.linalg.norm(curstate - problem.end_conf)

    if plot_mode == "rich":
        plot_iteration = 0
        figure = plt.figure(0, dpi=300)
    elif plot_mode == "debug":
        #figure = plt.figure(0)
        figure = plt.figure(0, dpi=300)
    if plot_mode == "debug" or plot_mode == "rich":
        figure.clf()
        axis = figure.gca()
        plot2dMap(axis, dataset)
        plot2dMap(axis, dataset)
        plotPointRobot2D(figure, axis, problem.gpmp_robot, problem.start_conf)
        plot_graph(planner_graph, axis)
        axis.set_title("COSTCO")

    path_points = []
    path_vels = []
    while np.linalg.norm(curstate[:2] - problem.end_conf[:2]) > problem.goal_region_threshold:
        prev_node = copy.deepcopy(planner_graph[cur_planner_st_id])
        loop_start_time = time.time()
        path_points.append(planner_graph[cur_planner_st_id].pose)
        path_vels.append(planner_graph[cur_planner_st_id].vel)
        # gtsam graph
        gtsam_graph, init_values = get_gtsam_graph(planner_graph, problem)

        # optimize,
        if problem.use_trustregion_opt:
            parameters = DoglegParams()
            optimizer = DoglegOptimizer(gtsam_graph, init_values, parameters)
        else:
            parameters = GaussNewtonParams()
            optimizer = GaussNewtonOptimizer(gtsam_graph, init_values, parameters)

        optimizer_temp_start = time.time()
        optimizer.optimizeSafely()
        optimizer_time += time.time() - optimizer_temp_start
        result = optimizer.values()

        prev_pose = planner_graph[cur_planner_st_id].pose
        # prune,

        update_planner_graph(result, planner_graph)
        planner = PlannerRRT(result, gtsam_graph, planner_graph)
        path, next_planner_st_id = planner.get_shortest_path(
            cur_planner_st_id, problem.end_conf
        )
        planner_graph = pruned_graph(next_planner_st_id, planner_graph)
        temp_planner_graph = planner_graph

        cur_planner_st_id = next_planner_st_id  # this is implied at the start

        if problem.use_noise:

            # add execution noise
            rand_temp = [random_noise.random() for i in range(problem.num_dof)]
            planner_graph[
                cur_planner_st_id
            ].pose += problem.action_noise * np.asarray(rand_temp)

            # add measurement noise
            rand_temp = [random_noise.random() for i in range(problem.num_dof)]
            planner_graph[
                cur_planner_st_id
            ].pose += problem.sensor_noise * np.asarray(rand_temp)

        problem.cur_planner_st_id = cur_planner_st_id
        problem.start_conf = planner_graph[cur_planner_st_id].pose
        problem.start_vel = planner_graph[cur_planner_st_id].vel
        problem.curr_conf = problem.start_conf

        # add new states add connect them on graph!
        if not problem.use_prev_graph:
            new_planner_graph = {
                cur_planner_st_id: Node(
                    cur_planner_st_id, problem.start_conf, problem.start_vel
                )
            }
            new_planner_graph[cur_planner_st_id].depth = 0
            planner_graph = grow_rrt_planner_graph(new_planner_graph, problem)
        else:
            planner_graph = grow_rrt_planner_graph(planner_graph, problem)

        curstate = planner_graph[cur_planner_st_id].pose

        problem.pose_fix_model = noiseModel_Isotropic.Sigma(
            problem.gpmp_robot.dof(),
            problem.sigma_goal_costco
            * np.linalg.norm(curstate - problem.end_conf)
            / init_distance,
        )
        problem.vel_fix_model = noiseModel_Isotropic.Sigma(
            problem.gpmp_robot.dof(),
            problem.sigma_goal_costco
            * np.linalg.norm(curstate - problem.end_conf)
            / init_distance,
        )

        total_time += time.time() - loop_start_time
        distance += np.linalg.norm(planner_graph[cur_planner_st_id].pose - prev_pose)

        if plot_mode == "rich":
            x = [point[0] for point in path_points]
            y = [point[1] for point in path_points]
            print("Iteration Num: ", iterations, "Time:", iterations*problem.delta_t, 
                  "curstate:", planner_graph[cur_planner_st_id].pose,
                  "curvel:", planner_graph[cur_planner_st_id].vel,  )
            interpolated_states = get_interpolated_points(
                prev_node, planner_graph[cur_planner_st_id], problem
            )

            interpolated_path_x = [point[0] for point in interpolated_states[1:]]
            interpolated_path_y = [point[1] for point in interpolated_states[1:]]

            for plotstate in interpolated_states[1:]:  # dont include the first state
                dataset.simulate(problem.pause_time)

                x.append(plotstate[0])
                y.append(plotstate[1])

                # temp_dataset = dataset.get_dataset()
                temp_dataset = dataset.get_dataset(
                    plotstate, [sdf_side, sdf_side]
                )  # pass in map size
                field = signedDistanceField2D(temp_dataset.map, temp_dataset.cell_size)

                axis.cla()
                axis.set_title(
                    "JIST - Time : {:5.2f} s".format(
                        problem.pause_time * plot_iteration
                    )
                )
                axis.tick_params(
                axis='x',          # changes apply to the x-axis
                which='both',      # both major and minor ticks are affected
                bottom=False,      # ticks along the bottom edge are off
                top=False,         # ticks along the top edge are off
                labelbottom=False) # labels along the bottom edge are off
                axis.tick_params(
                axis='y',          # changes apply to the x-axis
                which='both',      # both major and minor ticks are affected
                left=False,      # ticks along the bottom edge are off
                right=False,         # ticks along the top edge are off
                labelbottom=False) # labels along the bottom edge are off

                # Turn off tick labels
                axis.set_yticklabels([])
                axis.set_xticklabels([])

                plot_graph(planner_graph, axis)
                plotPointRobot2D(figure, axis, problem.gpmp_robot, plotstate)
                plot2dMap(axis, dataset)
                axis.imshow(
                    cv2.flip(field, flipCode=0),
                    extent=[
                        temp_dataset.origin_x,
                        temp_dataset.origin_x
                        + temp_dataset.cell_size * temp_dataset.cols,
                        temp_dataset.origin_y,
                        temp_dataset.origin_y
                        + temp_dataset.cell_size * temp_dataset.rows,
                    ],
                    alpha=0.5,
                )
                axis.plot(
                    interpolated_path_x,
                    interpolated_path_y,
                    color=(0, 0, 0),
                    linewidth=1,
                )
                axis.plot(x, y, "-g")
                axis.plot(start_conf[0], start_conf[1], "ro", markersize=3)
                axis.plot(goal_conf[0], goal_conf[1], "go", markersize=3)
                # figure.savefig('costco_' + str(plot_iteration) + '.eps', format='eps')
                figure.savefig("costco_" + str(plot_iteration) + ".png", dpi=figure.dpi)
                plt.pause(problem.pause_time)
                plot_iteration += 1

        elif plot_mode == "debug":
            # Signed Distance field
            dataset.simulate(problem.delta_t)
            # temp_dataset = dataset.get_dataset()
            temp_dataset = dataset.get_dataset(
                curstate, [sdf_side, sdf_side]
            )  # pass in map size
            field = signedDistanceField2D(temp_dataset.map, temp_dataset.cell_size)

            axis.cla()
            axis.set_title(
                    "JIST: {:5.2f} sec".format(
                        problem.delta_t * iterations
                    )
            )
            axis.tick_params(
            axis='x',          # changes apply to the x-axis
            which='both',      # both major and minor ticks are affected
            bottom=False,      # ticks along the bottom edge are off
            top=False,         # ticks along the top edge are off
            labelbottom=False) # labels along the bottom edge are off
            axis.tick_params(
            axis='y',          # changes apply to the x-axis
            which='both',      # both major and minor ticks are affected
            left=False,      # ticks along the bottom edge are off
            right=False,         # ticks along the top edge are off
            labelbottom=False) # labels along the bottom edge are off

            # Turn off tick labels
            axis.set_yticklabels([])
            axis.set_xticklabels([])
            print("Iteration Num: ", iterations, "Time:", iterations*problem.delta_t, 
                  "curstate:", planner_graph[cur_planner_st_id].pose,
                  "curvel:", planner_graph[cur_planner_st_id].vel,  )

            plot_graph(planner_graph, axis)
            plotPointRobot2D(figure, axis, problem.gpmp_robot, curstate)
            plot2dMap(axis, dataset)

            axis.imshow(
                cv2.flip(field, flipCode=0),
                extent=[
                    temp_dataset.origin_x,
                    temp_dataset.origin_x + temp_dataset.cell_size * temp_dataset.cols,
                    temp_dataset.origin_y,
                    temp_dataset.origin_y + temp_dataset.cell_size * temp_dataset.rows,
                ],
                alpha=0.5,
            )

            x = [point[0] for point in path_points]
            y = [point[1] for point in path_points]
            x.append(curstate[0])
            y.append(curstate[1])
            axis.plot(x, y, "-g", linewidth=2)

            axis.plot(start_conf[0], start_conf[1], "rx", markersize=14)
            axis.plot(goal_conf[0], goal_conf[1], "gx", markersize=14)
            figure.savefig('costco_' + str(iterations) + '.png', dpi=figure.dpi)
            plt.pause(problem.pause_time)

        elif plot_mode == "suppress":
            # Signed Distance field
            dataset.simulate(problem.delta_t)

            # temp_dataset = dataset.get_dataset()
            temp_dataset = dataset.get_dataset(
                curstate, [sdf_side, sdf_side]
            )  # pass in map size
            field = signedDistanceField2D(temp_dataset.map, temp_dataset.cell_size)

        else:
            raise ValueError('Invalid plot_mode')

        origin_point2 = Point2(temp_dataset.origin_x, temp_dataset.origin_y)
        problem.sdf = PlanarSDF(origin_point2, temp_dataset.cell_size, field)

        iterations += 1
        
        normalized_distance = distance / init_distance

        # Collision check
        if in_collision(curstate, problem.radius, field, temp_dataset):  # diameter of the robot
            return (
                test_status.COLLISION, 
                iterations,
                optimizer_time,
                total_time,
                normalized_distance,
                path_vels,
                np.linalg.norm(curstate[:2]- goal_conf[:2]),
            )

        # Time out check
        if time.time() - prog_start_time > problem.time_out_time:

            return (
                test_status.TIMEOUT,
                iterations,
                optimizer_time,
                total_time,
                normalized_distance,
                path_vels,
                np.linalg.norm(curstate[:2]- goal_conf[:2])
            )

    return (
        test_status.SUCCESS,
        iterations,
        optimizer_time,
        total_time,
        normalized_distance,
        path_vels,
        np.linalg.norm(curstate[:2]- goal_conf[:2])
    )



