import numpy as np
from gtsam import *
from gpmp2 import *
from random import seed
from random import random
import copy


class Problem:
    """General Interface to set the GPMP problem"""

    def __init__(self):

        self.gpmp_robot = None
        self.dataset = None
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


def get_initializations(nr_chains, problem):
    mean_chain = []
    pos_keys = []
    # init optimization
    graph = NonlinearFactorGraph()
    init_values = Values()

    for i in range(0, problem.total_time_step + 1):
        key_pos = symbol(ord("x"), i)
        pos_keys.append(key_pos)
        key_vel = symbol(ord("v"), i)

        # initialize as straight line in conf space
        pose = problem.start_conf * float(problem.total_time_step - i) / float(
            problem.total_time_step
        ) + problem.end_conf * i / float(problem.total_time_step)

        mean_chain.append(pose)
        vel = problem.avg_vel
        init_values.insert(key_pos, pose)
        init_values.insert(key_vel, vel)

        # start/end priors
        if i == 0:
            graph.push_back(
                PriorFactorVector(key_pos, problem.start_conf, problem.pose_fix_model)
            )
            graph.push_back(
                PriorFactorVector(key_vel, problem.start_vel, problem.vel_fix_model)
            )
        elif i == problem.total_time_step:
            graph.push_back(
                PriorFactorVector(key_pos, problem.end_conf, problem.pose_fix_model)
            )
            graph.push_back(
                PriorFactorVector(key_vel, problem.end_vel, problem.vel_fix_model)
            )

        # GP priors and cost factor
        if i > 0:
            key_pos1 = symbol(ord("x"), i - 1)
            key_pos2 = symbol(ord("x"), i)
            key_vel1 = symbol(ord("v"), i - 1)
            key_vel2 = symbol(ord("v"), i)

            temp = problem.gp_factor_function(
                key_pos1,
                key_vel1,
                key_pos2,
                key_vel2,
                problem.delta_t,
                problem.Qc_model,
            )
            graph.push_back(temp)

    parameters = GaussNewtonParams()
    optimizer = GaussNewtonOptimizer(graph, init_values, parameters)
    result = optimizer.values()

    pos_keys = np.asarray(pos_keys)
    # TODO: check with mustafa if to include start and goal
    pos_key_vect = createKeyVector(pos_keys)
    marginals = Marginals(graph, result)
    joint_marginal = marginals.jointMarginalCovariance(pos_key_vect)
    cov_mat = joint_marginal.fullMatrix()
    mean_chain = np.asarray(mean_chain)
    mean_chain = mean_chain.flatten()
    samples = np.random.multivariate_normal(mean_chain, cov_mat, nr_chains)

    initializations = []
    # fix start and goal state of the samples and also resize them properly!
    for i in range(nr_chains):
        initializations.append(
            samples[i, :].reshape(
                (problem.total_time_step + 1, problem.start_conf.shape[0])
            )
        )
        initializations[-1][0, :] = problem.start_conf
        initializations[-1][-1, :] = problem.end_conf

    return initializations


def get_parabolic_initialization(nr_chains, problem):

    mean_chain = []

    for i in range(0, problem.window_size + 1):
        # initialize as straight line in conf space
        pose = problem.start_conf * float(problem.window_size - i) / float(
            problem.window_size
        ) + problem.end_conf * (problem.init_fraction_length * i) / float(
            problem.window_size
        )
        mean_chain.append(pose)

        local_chain = []

        theta = math.atan2(
            problem.end_conf[1] - problem.start_conf[1],
            problem.end_conf[0] - problem.start_conf[0],
        )

    for pose in mean_chain:

        temp = pose - problem.start_conf

        temp2 = np.asarray([0.0, 0.0])

        # global frame to local frame
        temp2[0] = np.cos(theta) * temp[0] + np.sin(theta) * temp[1]
        # temp2[1]  = np.cos(angle)*temp[1] - np.sin(angle)*temp[0]
        temp2[1] = math.sqrt(temp2[0])
        local_chain.append(temp2)

    # translation
    trans = problem.start_conf

    problem.parabola_factor = 1

    co_effs = np.linspace(
        -1 * problem.parabola_factor, problem.parabola_factor, num=nr_chains
    )

    global_chains = []
    for i in range(co_effs.shape[0]):
        temp_chain = []
        for pose in local_chain:
            temp = np.asarray([0.0, 0.0])
            temp[0] = pose[0]
            temp[1] = co_effs[i] * pose[1]

            # local to global
            temp2 = np.asarray([0.0, 0.0])
            temp2[0] = np.cos(theta) * temp[0] - np.sin(theta) * temp[1]
            temp2[1] = np.sin(theta) * temp[0] + np.cos(theta) * temp[1]
            temp_chain.append(temp2 + problem.start_conf)
        global_chains.append(temp_chain)

    return np.asarray(global_chains)


def get_parabolic_planner_graph(inits, problem):

    # we have total_time_step+1 points in each trajectory
    total_time_step = inits[0].shape[0]
    nr_chains = len(inits)

    # if nr_chains==1:
    #   print("Single chain passed. Please pass multiple chains")
    #   return

    if total_time_step < 3:
        print("total_time_step cannot be less than 3")
        return

    # planner id is same as idx in nodes
    nodes = {}  # contains all nodes.

    map_ = {}  # key is (chain_number, timestamp)

    planner_id = 0
    map_[(0, 0)] = 0
    start_node = Node(planner_id, inits[0][0, :], problem.avg_vel)

    # planner_id += 1
    # map_[(0,total_time_step)] = 1
    # goal_node = Node(planner_id, inits[0][-1,:], problem.avg_vel)

    nodes[planner_id] = start_node
    # nodes.append(goal_node)

    terminal_node_ids = []

    # create all nodes
    for i in range(nr_chains):  # go through each chain
        for j in range(1, total_time_step):  # go through each time point

            planner_id = len(nodes)
            if j == total_time_step - 1:
                terminal_node_ids.append(planner_id)
            nodes[planner_id] = Node(planner_id, inits[i][j, :], problem.avg_vel)
            map_[(i, j)] = planner_id

    for i in range(nr_chains):  # go through each chain
        for j in range(0, total_time_step - 1):  # go through each time point
            # connect to start node
            if j == 0:
                first_idx = map_[(0, 0)]
                second_idx = map_[(i, j + 1)]
                # connect to goal node
                # elif j == total_time_step-1:
                #   first_idx = map_[(i,j)]
                #   second_idx = map_[(0,total_time_step)]
            else:
                first_idx = map_[(i, j)]
                second_idx = map_[(i, j + 1)]
            nodes[first_idx].add_neighbour(second_idx)

    # add random inter connections
    if problem.seed_val is not None:
        seed(problem.seed_val)
    for i in range(nr_chains):  # go through each chain
        for j in range(1, total_time_step - 2):  # go through each time point
            for k in range(nr_chains):  # choose a branch to connect to
                if i == k:
                    continue

                # print("yoooo",i,j,k)
                rand_num = random()
                if rand_num < problem.dropout_prob:
                    first_idx = map_[(i, j)]
                    second_idx = map_[(k, j + 1)]
                    nodes[first_idx].add_neighbour(second_idx)

    # primary chains is a list of lists of plannerid's
    return nodes, terminal_node_ids, primary_chains


def get_initializations_simple(problem):

    return (problem.window_size + 1) * [problem.start_conf]


def get_planner_graph(inits, problem):

    # we have total_time_step+1 points in each trajectory
    total_time_step = inits[0].shape[0] - 1
    nr_chains = len(inits)

    # if nr_chains==1:
    #   print("Single chain passed. Please pass multiple chains")
    #   return

    if total_time_step == 0:
        print("total_time_step cannot be 0")
        return

    # planner id is same as idx in nodes
    nodes = []  # contains all nodes.

    map_ = {}  # key is (chain_number, timestamp)

    planner_id = 0
    map_[(0, 0)] = 0
    start_node = Node(planner_id, inits[0][0, :], problem.avg_vel)

    planner_id += 1
    map_[(0, total_time_step)] = 1
    goal_node = Node(planner_id, inits[0][-1, :], problem.avg_vel)

    nodes.append(start_node)
    nodes.append(goal_node)

    # create all nodes
    for i in range(nr_chains):  # go through each chain
        for j in range(1, total_time_step):  # go through each time point

            planner_id = len(nodes)
            nodes.append(Node(planner_id, inits[i][j, :], problem.avg_vel))
            map_[(i, j)] = planner_id

    for i in range(nr_chains):  # go through each chain
        for j in range(0, total_time_step):  # go through each time point
            # connect to start node
            if j == 0:
                first_idx = map_[(0, 0)]
                second_idx = map_[(i, j + 1)]
            # connect to goal node
            elif j == total_time_step - 1:
                first_idx = map_[(i, j)]
                second_idx = map_[(0, total_time_step)]
            else:
                first_idx = map_[(i, j)]
                second_idx = map_[(i, j + 1)]
            nodes[first_idx].add_neighbour(second_idx)

    # add random inter connections
    if problem.seed_val is not None:
        seed(problem.seed_val)

    for i in range(nr_chains):  # go through each chain
        for j in range(1, total_time_step - 1):  # go through each time point
            for k in range(nr_chains):  # choose a branch to connect to
                if i == k:
                    continue

                rand_num = random()
                if rand_num < problem.dropout_prob:
                    first_idx = map_[(i, j)]
                    second_idx = map_[(k, j + 1)]
                    nodes[first_idx].add_neighbour(second_idx)

    return nodes


def get_gtsam_graph(node_list, problem):

    # init optimization
    graph = NonlinearFactorGraph()
    init_values = Values()

    # add all nodes
    for i in range(len(node_list)):
        key_pos = symbol(ord("x"), i)
        key_vel = symbol(ord("v"), i)

        #% initialize as straight line in conf space
        init_values.insert(key_pos, node_list[i].pose)
        init_values.insert(key_vel, node_list[i].vel)

        #% start/end priors
        if i == 0:
            graph.push_back(
                PriorFactorVector(key_pos, node_list[i].pose, problem.pose_fix_model)
            )
            graph.push_back(
                PriorFactorVector(key_vel, problem.start_vel, problem.vel_fix_model)
            )
        elif i == 1:
            graph.push_back(
                PriorFactorVector(key_pos, node_list[i].pose, problem.pose_fix_model)
            )
            graph.push_back(
                PriorFactorVector(key_vel, problem.end_vel, problem.vel_fix_model)
            )

        if i > 0:
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
            if problem.use_GP_inter and problem.check_inter > 0:
                for j in range(1, problem.check_inter + 1):
                    tau = j * (problem.total_time_sec / problem.total_check_step)
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


#### Dijkstra specific stuff

from Queue import PriorityQueue


class Planner(object):
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
        if second_idx != 1:
            cost += self.get_factor_error(self.planner_graph[second_idx].gt_graph_ob_id)
        return cost

    def get_shortest_path(self):

        cur_id = 0
        priority_q = PriorityQueue()
        priority_q.put((0, cur_id))
        self.planner_graph[cur_id].visited = True
        while priority_q.qsize() > 0:
            cur_cost, cur_id = priority_q.get()

            if cur_id == 1:  # goal id
                break

            for neigh_id in self.planner_graph[cur_id].neighbours:
                if self.planner_graph[neigh_id].visited == True:
                    continue
                cost = self.get_edge_cost(cur_id, neigh_id)
                priority_q.put((cur_cost + cost, neigh_id))
                self.planner_graph[neigh_id].visited = True
                self.planner_graph[neigh_id].parent_id = cur_id

        path = []
        if cur_id != 1:
            print("Failed to find a path. Check if everything is right")

        path.append(self.planner_graph[cur_id].pose)
        while cur_id != 0:
            cur_id = self.planner_graph[cur_id].parent_id
            path.append(self.planner_graph[cur_id].pose)

        path.reverse()
        return path


def update_planner_graph(result, planner_graph):
    for i in range(len(planner_graph)):
        planner_graph[i].pose = result.atVector(symbol(ord("x"), i))
        planner_graph[i].vel = result.atVector(symbol(ord("v"), i))
