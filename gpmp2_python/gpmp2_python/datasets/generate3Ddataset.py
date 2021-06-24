from gtsam import *
from gpmp2 import *
import math
import numpy as np


class Dataset:
    """docstring for Dataset"""

    def __init__(self):
        self.cols = None
        self.rows = None
        self.z = None
        self.origin_x = None
        self.origin_y = None
        self.origin_z = None
        self.cell_size = None
        self.map = None
        self.corner_idx = None


def add_obstacle(position, size, map, corner):
    half_size_row = int(math.floor((size[0] - 1) / 2))
    half_size_col = int(math.floor((size[1] - 1) / 2))
    half_size_z = int(math.floor((size[2] - 1) / 2))

    # occupency grid
    map[
        position[0] - half_size_row - 1 : position[0] + half_size_row,
        position[1] - half_size_col - 1 : position[1] + half_size_col,
        position[2] - half_size_z - 1 : position[2] + half_size_z,
    ] = np.ones((2 * half_size_row + 1, 2 * half_size_col + 1, 2 * half_size_z + 1))

    # corner
    temp = np.asarray(
        [
            position[0] - half_size_row - 1,
            position[0] + half_size_row - 1,
            position[1] - half_size_col - 1,
            position[1] + half_size_col - 1,
            position[2] - half_size_z - 1,
            position[2] + half_size_z - 1,
        ]
    ).reshape(1, 6)

    if corner is None:
        corner = temp
    else:
        corner = np.concatenate((corner, temp), axis=0)

    return [map, corner]


def generate3Ddataset(dataset_str):
    # GENERATE3DDATASET enerate 3D dataset evidence grid
    #
    #   Usage: dataset = GENERATE3DDATASET(dataset_str)
    #   @dataset_str       dataset string, existing datasets:
    #                      'WAMDeskDataset'
    #
    #   Dataset Format:
    #   dataset.map        ground truth evidence grid
    #   dataset.rows       number of rows (x)
    #   dataset.cols       number of cols (y)
    #   dataset.z          number of depth (z)
    #   dataset.origin_x   origin of map x
    #   dataset.origin_y   origin of map y
    #   dataset.origin_z   origin of map z
    #   dataset.cell_size  cell size
    #   dataset.corner_idx corner index to visualize edges

    dataset = Dataset()
    # dataset 1: small dataset for demo
    if dataset_str is "SmallDemo":
        # params
        dataset.cols = 200
        dataset.rows = 200
        dataset.z = 200
        dataset.origin_x = -1
        dataset.origin_y = -1
        dataset.origin_z = -1
        dataset.cell_size = 0.01
        # map
        dataset.map = np.zeros((dataset.rows, dataset.cols, dataset.z))
        # obstacles
        dataset.corner_idx = None
        dataset.map, dataset.corner_idx = add_obstacle(
            [150, 150, 150], [20, 20, 20], dataset.map, dataset.corner_idx
        )

    # dataset 2: desk dataset for WAM WAMDeskDataset
    elif dataset_str is "WAMDeskDataset":
        # params
        dataset.cols = 300
        dataset.rows = 300
        dataset.z = 300
        dataset.origin_x = -1.5
        dataset.origin_y = -1.5
        dataset.origin_z = -1.5
        dataset.cell_size = 0.01
        # map
        dataset.map = np.zeros((dataset.rows, dataset.cols, dataset.z))
        # obstacles
        dataset.corner_idx = None
        [dataset.map, dataset.corner_idx] = add_obstacle(
            [170, 220, 130], [140, 60, 5], dataset.map, dataset.corner_idx
        )
        [dataset.map, dataset.corner_idx] = add_obstacle(
            [105, 195, 90], [10, 10, 80], dataset.map, dataset.corner_idx
        )
        [dataset.map, dataset.corner_idx] = add_obstacle(
            [235, 195, 90], [10, 10, 80], dataset.map, dataset.corner_idx
        )
        [dataset.map, dataset.corner_idx] = add_obstacle(
            [105, 245, 90], [10, 10, 80], dataset.map, dataset.corner_idx
        )
        [dataset.map, dataset.corner_idx] = add_obstacle(
            [235, 245, 90], [10, 10, 80], dataset.map, dataset.corner_idx
        )

        [dataset.map, dataset.corner_idx] = add_obstacle(
            [250, 190, 145], [60, 5, 190], dataset.map, dataset.corner_idx
        )
        [dataset.map, dataset.corner_idx] = add_obstacle(
            [250, 90, 145], [60, 5, 190], dataset.map, dataset.corner_idx
        )

        [dataset.map, dataset.corner_idx] = add_obstacle(
            [200, 190, 145], [40, 5, 190], dataset.map, dataset.corner_idx
        )
        # [dataset.map, dataset.corner_idx] = add_obstacle([130 40 95], [60, 5, 190], dataset.map, dataset.corner_idx);

        [dataset.map, dataset.corner_idx] = add_obstacle(
            [250, 140, 240], [60, 100, 5], dataset.map, dataset.corner_idx
        )
        [dataset.map, dataset.corner_idx] = add_obstacle(
            [250, 140, 190], [60, 100, 5], dataset.map, dataset.corner_idx
        )
        [dataset.map, dataset.corner_idx] = add_obstacle(
            [250, 140, 140], [60, 100, 5], dataset.map, dataset.corner_idx
        )
        [dataset.map, dataset.corner_idx] = add_obstacle(
            [250, 140, 90], [60, 100, 5], dataset.map, dataset.corner_idx
        )

    # no such dataset
    else:
        error("No such dataset exist")
    return dataset
