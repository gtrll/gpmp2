import numpy as np
from gtsam import *
from gpmp2 import *
import math
from scipy import ndimage


def signedDistanceField3D(ground_truth_map, cell_size):
    # SIGNEDDISTANCEFIELD3D 3D signed distance field
    #   Given a ground truth 3D map defined in Matrix in 0-1,
    #   calculate 3D signed distance field, which is defined as a matrix
    #   map matrix and signed distance field matrix have the same resolution.
    #
    #   Usage: field = SIGNEDDISTANCEFIELD3D(ground_truth_map, cell_siz)
    #   @map        evidence grid from dataset, map use 0 show open area, 1 show objects.
    #   @cell_size  cell sizeto given metric information
    #
    #   Output:
    #   @field      sdf, row is X, col is Y, 3rd index is Z

    # regularize unknow area to open area
    cur_map = ground_truth_map > 0.75
    cur_map = cur_map.astype(int)

    if np.amax(cur_map) is 0:
        return np.ones(ground_truth_map.shape) * 1000

    # inverse map
    inv_map = 1 - cur_map

    # get signed distance from map and inverse map
    # since bwdist(foo) = ndimage.distance_transform_edt(1-foo)
    map_dist = ndimage.distance_transform_edt(inv_map)
    inv_map_dist = ndimage.distance_transform_edt(cur_map)

    field = map_dist - inv_map_dist

    # metric
    field = field * cell_size
    field = field.astype(float)

    return field
