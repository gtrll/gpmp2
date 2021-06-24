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
        position[0] - half_size_row - 1 : position[0] + half_size_row,
        position[1] - half_size_col - 1 : position[1] + half_size_col,
    ]

    map[
        position[0] - half_size_row - 1 : position[0] + half_size_row,
        position[1] - half_size_col - 1 : position[1] + half_size_col,
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