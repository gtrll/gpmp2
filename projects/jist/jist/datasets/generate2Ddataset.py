import numpy as np
from gtsam import *
from gpmp2 import *
import math
from random import Random


dataset_random = Random()


class Obstacle:
    """docstring for Dataset"""

    def __init__(self):
        self.x = None
        self.y = None
        self.v_x = None
        self.v_y = None
        self.a_x = None
        self.a_y = None
        self.size = None


class Dataset:
    """docstring for Dataset"""

    def __init__(self):
        self.cols = None
        self.rows = None
        self.origin_x = None
        self.origin_y = None
        self.cell_size = None
        self.map = None


def random_number(min_val, max_val):
    return min_val + (max_val - min_val) * dataset_random.random()


def get_center(x, y, dataset):

    center = (
        np.asarray([y - dataset.origin_y, x - dataset.origin_x]) / dataset.cell_size
    )
    return center.astype(int)


def get_dim(w, h, dataset):

    return np.asarray([h, w]) / dataset.cell_size


def add_obstacle(
    position, size, map, landmarks=None, origin_x=None, origin_y=None, cell_size=None
):

    half_size_row = int(math.floor((size[0] - 1) / 2))
    half_size_col = int(math.floor((size[1] - 1) / 2))

    # occupency grid. here map is assumed to be numpy array

    temp = map[
        max(0, position[0] - half_size_row - 1) : min(map.shape[0],position[0] + half_size_row),
        max(0, position[1] - half_size_col - 1) : min(map.shape[1],position[1] + half_size_col),
    ]

    map[
        max(0, position[0] - half_size_row - 1) :min(map.shape[0], position[0] + half_size_row),
        max(0, position[1] - half_size_col - 1) :min(map.shape[1], position[1] + half_size_col),
    ] = np.ones(temp.shape)

    # landmarks
    if landmarks is not None and origin_x is not None and origin_y is not None:

        raise NotImplementedError

    return map


def generate2Ddataset(dataset_str):
    # GENERATE2DDATASET Generate 2D dataset evidence grid
    #
    #    Usage: dataset = GENERATE2DDATASET(dataset_str)
    #    @dataset_str       dataset string, existing datasets:
    #                       'OneObstacleDataset', 'TwoObstaclesDataset'
    #
    #    Output Format:
    #    dataset.map        ground truth evidence grid
    #    dataset.rows       number of rows (y)
    #    dataset.cols       number of cols (x)
    #    dataset.origin_x   origin of map x
    #    dataset.origin_y   origin of map y
    #    dataset.cell_size  cell size

    dataset = Dataset()
    # dataset 5: 1 obs dataset for 2D Arm obs avoid
    if dataset_str is "OneObstacleDataset":
        # params
        dataset.cols = 300
        dataset.rows = 300
        dataset.origin_x = -1
        dataset.origin_y = -1
        dataset.cell_size = 0.01
        # map
        dataset.map = np.zeros((dataset.rows, dataset.cols))
        # obstacles
        dataset.map = add_obstacle([190, 160], [60, 80], dataset.map)

    # dataset 6: obs dataset for 3D Arm obs avoid

    elif dataset_str is "Empty":
        # params
        dataset.cols = 300
        dataset.rows = 300
        dataset.origin_x = -1
        dataset.origin_y = -1
        dataset.cell_size = 0.01
        # map
        dataset.map = np.zeros((dataset.rows, dataset.cols))
        # obstacles
        # dataset.map = add_obstacle([200, 200], [80, 100], dataset.map)
        # dataset.map = add_obstacle([160, 80], [30, 80], dataset.map)

    elif dataset_str is "TwoObstaclesDataset":
        # params
        dataset.cols = 300
        dataset.rows = 300
        dataset.origin_x = -1
        dataset.origin_y = -1
        dataset.cell_size = 0.01
        # map
        dataset.map = np.zeros((dataset.rows, dataset.cols))
        # obstacles
        dataset.map = add_obstacle([200, 200], [80, 100], dataset.map)
        dataset.map = add_obstacle([160, 80], [30, 80], dataset.map)

    # dataset 7: multiple obs dataset for 2D Arm obs avoid
    elif dataset_str is "MultiObstacleDataset":
        # params
        dataset.cols = 400  # x
        dataset.rows = 300  # y
        dataset.origin_x = -20
        dataset.origin_y = -10
        dataset.cell_size = 0.1
        # map
        dataset.map = np.zeros((dataset.rows, dataset.cols))
        # obstacles
        dataset.map = add_obstacle(
            get_center(12, 10, dataset), get_dim(5, 7, dataset), dataset.map
        )
        dataset.map = add_obstacle(
            get_center(-7, 10, dataset), get_dim(10, 7, dataset), dataset.map
        )
        dataset.map = add_obstacle(
            get_center(0, -5, dataset), get_dim(10, 5, dataset), dataset.map
        )

    # mobile 2d map
    elif dataset_str is "MobileMap1":
        # params
        dataset.cols = 500  # x
        dataset.rows = 500  # y
        dataset.origin_x = -10
        dataset.origin_y = -10
        dataset.cell_size = 0.01
        # map
        dataset.map = np.zeros((dataset.rows, dataset.cols))
        # obstacles
        dataset.map = add_obstacle(
            get_center(0, 0, dataset), get_dim(1, 5, dataset), dataset.map
        )
        # dataset.map = add_obstacle(get_center(-2.5,-2,dataset), get_dim(5,1,dataset), dataset.map);
        # wall
        dataset.map = add_obstacle(
            get_center(0, 4.5, dataset), get_dim(10, 1, dataset), dataset.map
        )
        dataset.map = add_obstacle(
            get_center(0, -4.5, dataset), get_dim(10, 1, dataset), dataset.map
        )
        dataset.map = add_obstacle(
            get_center(4.5, 0, dataset), get_dim(1, 10, dataset), dataset.map
        )
        dataset.map = add_obstacle(
            get_center(-4.5, 0, dataset), get_dim(1, 10, dataset), dataset.map
        )

    # no such dataset
    else:
        raise NameError("No such dataset exist")

    return dataset


class Patrol2Ddataset(object):
    # keep track of objets, and their velocity.
    # add big boundary stationary walls on all 4 sides.
    # randomly intialize objets of same size at different x, y
    # set random velocity magnitudes and directions for each objects
    # simulate / move by dt time.
    # if y is less than bottow or y is > top vy *= -1
    # if x is less than left or x is > right vx *= -1
    def __init__(
        self,
        rows=400,
        cols=200,  # rows corresponds to y
        origin_x=0.0,
        origin_y=0.0,
        cell_size=0.1,
    ):

        self.dataset = Dataset()
        self.dataset.cols = cols
        self.dataset.rows = rows
        self.dataset.origin_x = origin_x
        self.dataset.origin_y = origin_y
        self.dataset.cell_size = cell_size
        # map
        self.static_obstacles = []
        self.dynamic_obstacles = []

        # left, right, bottom, up
        self.bounds = [
            origin_x,
            origin_x + cell_size * cols,
            origin_y,
            origin_y + cell_size * rows,
        ]

    def init_obstacles(self, seed_val=None, obstacle_num=100, vel_limit=0.8,  acc_limit=None,):

        if seed_val is None:
            seed_val = 0
        dataset_random.seed(seed_val)

        # make static obstacles
        self.static_obstacles.append(
            self.make_obstacle(10.0, 15.0, np.asarray([1.0, 3.0]))
        )

        self.static_obstacles.append(self.make_obstacle(3.0, 20.0, np.asarray([4, 40])))

        self.static_obstacles.append(
            self.make_obstacle(17.0, 20.0, np.asarray([4, 40]))
        )

        # make dynamic obstacles
        self.dynamic_bounds = np.asarray(self.bounds)
        self.dynamic_bounds[0] = 8.0
        self.dynamic_bounds[1] = 12.0

        self.dynamic_obstacles.append(
            self.make_obstacle(
                random_number(self.dynamic_bounds[0], self.dynamic_bounds[1]),
                10,
                np.asarray([5.0, 3.0]),
            )
        )
        self.dynamic_obstacles[0].v_y = 0.0
        self.dynamic_obstacles[0].v_x = vel_limit# * (2.0/3.0)

        if obstacle_num == 2:
            self.dynamic_obstacles.append(
                self.make_obstacle(
                    random_number(self.dynamic_bounds[0], self.dynamic_bounds[1]),
                    25,
                    np.asarray([5.0, 3.0]),
                )
            )
            self.dynamic_obstacles[1].v_y = 0.0
            self.dynamic_obstacles[1].v_x = vel_limit

            self.static_obstacles.append(
                self.make_obstacle(10.0, 30.0, np.asarray([1.0, 3.0]))
            )

        self.obstacles = self.static_obstacles + self.dynamic_obstacles

    def make_obstacle(self, x, y, size):
        obs = Obstacle()
        obs.x = x
        obs.y = y
        obs.size = size

        return obs

    def simulate(self, dt):

        for obs in self.dynamic_obstacles:
            obs.x += dt * obs.v_x
            obs.y += dt * obs.v_y

            # change velocities
            if obs.x <= self.dynamic_bounds[0] or obs.x >= self.dynamic_bounds[1]:
                obs.v_x *= -1
            if obs.y <= self.dynamic_bounds[2] or obs.y >= self.dynamic_bounds[3]:
                obs.v_y *= -1

    def get_dataset(self, pose=None, map_size=None):  # size in meters

        self.dataset.map = np.zeros((self.dataset.rows, self.dataset.cols))

        for obs in self.static_obstacles:
            self.dataset.map = add_obstacle(
                get_center(obs.x, obs.y, self.dataset),
                get_dim(obs.size[0], obs.size[1], self.dataset),
                self.dataset.map,
            )

        for obs in self.dynamic_obstacles:
            self.dataset.map = add_obstacle(
                get_center(obs.x, obs.y, self.dataset),
                get_dim(obs.size[0], obs.size[1], self.dataset),
                self.dataset.map,
            )

        if pose is None or map_size is None:
            return self.dataset

        temp_dataset = Dataset()
        temp_dataset.origin_x = pose[0] - float(map_size[0]) / 2.0
        temp_dataset.origin_y = pose[1] - float(map_size[1]) / 2.0
        temp_dataset.cell_size = self.dataset.cell_size

        lower_left = get_center(
            temp_dataset.origin_x, temp_dataset.origin_y, self.dataset
        )

        if lower_left[0] < 0:
            lower_left[0] = 0
            temp_dataset.origin_y = self.dataset.origin_y

        if lower_left[1] < 0:
            lower_left[1] = 0
            temp_dataset.origin_x = self.dataset.origin_x

        upper_right = get_center(
            pose[0] + float(map_size[0]) / 2.0,
            pose[1] + float(map_size[1]) / 2.0,
            self.dataset,
        )

        if upper_right[0] >= self.dataset.map.shape[0]:
            upper_right[0] = self.dataset.map.shape[0] - 1

        if upper_right[1] >= self.dataset.map.shape[1]:
            upper_right[1] = self.dataset.map.shape[1] - 1

        if (
            upper_right[0] < 0
            or upper_right[1] < 0
            or lower_left[0] >= self.dataset.map.shape[0]
            or lower_left[1] >= self.dataset.map.shape[1]
        ):
            temp_dataset.map = np.zeros((5, 5))  # some map of zeros
        else:
            temp_dataset.map = self.dataset.map[
                lower_left[0] : upper_right[0] + 1, lower_left[1] : upper_right[1] + 1
            ]

        temp_dataset.rows = temp_dataset.map.shape[0]
        temp_dataset.cols = temp_dataset.map.shape[1]
        return temp_dataset

    


class Dynamic2Ddataset(object):
    # keep track of objets, and their velocity.
    # add big boundary stationary walls on all 4 sides.
    # randomly intialize objets of same size at different x, y
    # set random velocity magnitudes and directions for each objects
    # simulate / move by dt time.
    # if y is less than bottow or y is > top vy *= -1
    # if x is less than left or x is > right vx *= -1

    def __init__(
        self,
        rows=900,
        cols=1200,
        origin_x=0.0,
        origin_y=0.0,
        cell_size=0.1,
    ):

        self.dataset = Dataset()
        self.dataset.cols = cols
        self.dataset.rows = rows
        self.dataset.origin_x = origin_x
        self.dataset.origin_y = origin_y
        self.dataset.cell_size = cell_size
        # map
        self.obstacles = []

        # left, right, bottom, up

        self.bounds = [
            origin_x,
            origin_x + cell_size * cols,
            origin_y,
            origin_y + cell_size * rows,
        ]

        self.obs_size = (self.bounds[1] - self.bounds[0]) / 20.0

        self.obs_dist = 10.0
        self.goal = None
        self.start = None

        self.uniform_grid = False

    def init_obstacles(self, seed_val=None, obstacle_num=100, vel_limit=None,  acc_limit=None, 
                        start=None, goal=None):


        if obstacle_num == -1:
            self.uniform_grid = True

            x = self.bounds[0]
            while x < self. bounds[1]:
                y = self.bounds[2]
                while  y < self.bounds[3]:
                    obs = Obstacle()
                    obs.x = x
                    obs.y = y
                    obs.size = np.asarray([self.obs_size, self.obs_size])

                    y += 15.0

                    if goal is not None:
                        if np.linalg.norm(np.array([obs.x-goal[0], obs.y-goal[1]])) < self.obs_dist:
                            continue
                    if start is not None:
                        if np.linalg.norm(np.array([obs.x-start[0], obs.y-start[1]])) < self.obs_dist:
                            continue
                    self.obstacles.append(obs)
                x += 15.0
            return





        if start is not None:
            self.start = start

        if goal is not None:
            self.goal = goal

        if seed_val is None:
            seed_val = 0
        dataset_random.seed(seed_val)

        if vel_limit is None:
            vel_limit = self.obs_size / 6
        self.obs_velocity = vel_limit

        if acc_limit is None:
            acc_limit = self.obs_size / 10
        self.obs_acc = acc_limit
        # change in acceleration with time
        self.acc_dt = self.obs_acc / 10

        while len(self.obstacles) < obstacle_num:
            # choose random x,y for obstacles.

            obs = Obstacle()
            obs.x = random_number(self.bounds[0], self.bounds[1])
            obs.y = random_number(self.bounds[2], self.bounds[3])
            
            if goal is not None:
                if np.linalg.norm(np.array([obs.x-goal[0], obs.y-goal[1]])) < self.obs_dist:
                    continue
            if start is not None:
                if np.linalg.norm(np.array([obs.x-start[0], obs.y-start[1]])) < self.obs_dist:
                    continue

            obs.v_x = random_number(-1 * self.obs_velocity, self.obs_velocity)
            obs.v_y = random_number(-1 * self.obs_velocity, self.obs_velocity)
            obs.a_x = random_number(-1 * self.obs_acc, self.obs_acc)
            obs.a_y = random_number(-1 * self.obs_acc, self.obs_acc)
            obs.size = np.asarray([self.obs_size, self.obs_size])
            self.obstacles.append(obs)

            # print('bounds', self.bounds, self.obs_size)

    def simulate(self, dt):

        if self.uniform_grid:
            return

        for obs in self.obstacles:

            # Set accelerations
            if random_number(0.0, 1.0) > 0.5:
                obs.a_x += self.acc_dt
            else:
                obs.a_x -= self.acc_dt

            obs.a_x = min(obs.a_x, self.obs_acc)
            obs.a_x = max(obs.a_x, -1 * self.obs_acc)

            if random_number(0.0, 1.0) > 0.5:
                obs.a_y += self.acc_dt
            else:
                obs.a_y -= self.acc_dt

            obs.a_y = min(obs.a_y, self.obs_acc)
            obs.a_y = max(obs.a_y, -1 * self.obs_acc)

            obs.v_x += dt * obs.a_x
            obs.v_y += dt * obs.a_y


            obs.v_x = min(obs.v_x, self.obs_velocity)
            obs.v_x = max(obs.v_x, -1 * self.obs_velocity)


            obs.v_y = min(obs.v_y, self.obs_velocity)
            obs.v_y = max(obs.v_y, -1 * self.obs_velocity)

            obs.x += dt * obs.v_x
            obs.y += dt * obs.v_y

            if self.goal is not None:
                if np.linalg.norm(np.array([obs.x-self.goal[0], obs.y-self.goal[1]])) < self.obs_dist:
                    # bounce off the circle around goal
                    v1 = np.array([obs.x-self.goal[0], obs.y-self.goal[1]])
                    v1 = v1/np.linalg.norm(v1)

                    v2 = -1 * np.array([obs.v_x, obs.v_y])
                    v2 = v2/np.linalg.norm(v2)

                    theta = math.acos(np.linalg.norm( np.dot(v1, v2)))

                    alpha = math.atan2(obs.v_y, obs.v_x)
                    beta = math.atan2(obs.y - self.goal[1], obs.x - self.goal[0])

                    vel_vect = np.array([obs.v_x, obs.v_y])

                    angle = beta - alpha - theta

                    rot_mat = np.array([[math.cos(angle), -1* math.sin(angle)], [ math.sin(angle), math.cos(angle)]])

                    new_vels =   np.matmul(np.asarray([obs.v_x, obs.v_y]) , rot_mat.T)

                    obs.v_x = new_vels[0]
                    obs.v_y = new_vels[1]

            if self.start is not None:
                if np.linalg.norm(np.array([obs.x-self.start[0], obs.y-self.start[1]])) < self.obs_dist:
                    # bounce off the circle around start

                    v1 = np.array([obs.x-self.start[0], obs.y-self.start[1]])
                    v1 = v1/np.linalg.norm(v1)

                    v2 = -1 * np.array([obs.v_x, obs.v_y])
                    v2 = v2/np.linalg.norm(v2)

                    theta = math.acos(np.linalg.norm( np.dot(v1, v2)))

                    alpha = math.atan2(obs.v_y, obs.v_x)
                    beta = math.atan2(obs.y - self.start[1], obs.x - self.start[0])

                    vel_vect = np.array([obs.v_x, obs.v_y])

                    angle = beta - alpha - theta

                    rot_mat = np.array([[math.cos(angle), -1* math.sin(angle)], [ math.sin(angle), math.cos(angle)]])

                    new_vels =   np.matmul(np.asarray([obs.v_x, obs.v_y]) , rot_mat.T)

                    obs.v_x = new_vels[0]
                    obs.v_y = new_vels[1]


            # change velocities
            if obs.x <= self.bounds[0] or obs.x >= self.bounds[1]:
                obs.v_x *= -1
            if obs.y <= self.bounds[2] or obs.y >= self.bounds[3]:
                obs.v_y *= -1

    def get_dataset(self, pose=None, map_size=None):  # size in meters

        self.dataset.map = np.zeros((self.dataset.rows, self.dataset.cols))

        for obs in self.obstacles:
            self.dataset.map = add_obstacle(
                get_center(obs.x, obs.y, self.dataset),
                get_dim(obs.size[0], obs.size[1], self.dataset),
                self.dataset.map,
            )

        if pose is None or map_size is None:
            return self.dataset

        temp_dataset = Dataset()
        temp_dataset.origin_x = pose[0] - float(map_size[0]) / 2.0
        temp_dataset.origin_y = pose[1] - float(map_size[1]) / 2.0
        temp_dataset.cell_size = self.dataset.cell_size

        lower_left = get_center(
            temp_dataset.origin_x, temp_dataset.origin_y, self.dataset
        )

        if lower_left[0] < 0:
            lower_left[0] = 0
            temp_dataset.origin_y = self.dataset.origin_y

        if lower_left[1] < 0:
            lower_left[1] = 0
            temp_dataset.origin_x = self.dataset.origin_x

        upper_right = get_center(
            pose[0] + float(map_size[0]) / 2.0,
            pose[1] + float(map_size[1]) / 2.0,
            self.dataset,
        )

        if upper_right[0] >= self.dataset.map.shape[0]:
            upper_right[0] = self.dataset.map.shape[0] - 1

        if upper_right[1] >= self.dataset.map.shape[1]:
            upper_right[1] = self.dataset.map.shape[1] - 1

        if (
            upper_right[0] < 0
            or upper_right[1] < 0
            or lower_left[0] >= self.dataset.map.shape[0]
            or lower_left[1] >= self.dataset.map.shape[1]
        ):
            temp_dataset.map = np.zeros((5, 5))  # some map of zeros
        else:
            temp_dataset.map = self.dataset.map[
                lower_left[0] : upper_right[0] + 1, lower_left[1] : upper_right[1] + 1
            ]

        temp_dataset.rows = temp_dataset.map.shape[0]
        temp_dataset.cols = temp_dataset.map.shape[1]
        return temp_dataset

