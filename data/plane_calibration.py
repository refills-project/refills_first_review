from collections import OrderedDict

import PyKDL as kdl
import numpy as np
import scipy.linalg


def print_points(points):
    for point_name in points:
        print '{}: {}'.format(point_name, points[point_name])


def to_list(points):
    return [point for (_, point) in points.items()]


def to_np(points):
    x = []
    y = []
    z = []
    for name in points:
        x.append(points[name].x())
        y.append(points[name].y())
        z.append(points[name].z())
    return np.c_[x,y,z]


def to_kdl(a):
    return kdl.Vector(a[0], a[1], a[2])


def get_plane_normal(points, flip_sign=False):
    # algorithm taken from here: https://gist.github.com/amroamroamro/1db8d69b4b65e8bc66a6
    # best-fit linear plane
    data = to_np(points)
    A = np.c_[data[:, 0], data[:, 1], np.ones(data.shape[0])]
    C, _, _, _ = scipy.linalg.lstsq(A, data[:, 2])  # coefficients

    # extract normal as KDL vector
    n = to_kdl(C)
    n.Normalize()
    if flip_sign:
        return -1.0 * n
    else:
        return n


def get_sub_dict(orig_dict, keys):
    return {key: orig_dict[key] for key in keys}


def get_line_normal(points):
    # algorithm taken from here: https://stackoverflow.com/questions/2298390/fitting-a-line-in-3d
    # best-fit linear line
    data = to_np(points)
    datamean = data.mean(axis=0)
    # find first principal component of mean-centered data points
    uu, dd, vv = np.linalg.svd(data - datamean)
    return to_kdl(vv[0])


def get_adjusted_rot_matrix(points, line_keys):
    plane_n =  get_plane_normal(points, True)
    line_n = get_line_normal(get_sub_dict(points, line_keys))
    return kdl.Rotation(line_n , plane_n * line_n, plane_n)


# START OF THE ACTUAL SCRIPT

# dict with all points
points = OrderedDict()

# MOCAP COORDINATE SYSTEM:
# Orientation:
# +Y: straight up
# +X: along the wall behind the shelves, pointing away from the door
# +Z: according to right-hand-rule
# Position:
# In the corner and on the floor left of the shelves
# Issues:
# Walls in the corner are not rectangular --> unwanted rotation around +Y
# Floor is not flat, sinks towards the middle of the room --> unwanted rotation around +Z and +X

# DESIRED COORDINATE SYSTEM AFTER TRANSFORMATION:
# Orientation:
# +Z: straight up
# +X: along the front of the shelves
# +Y: according to right-hand-rule
# Position:
# In the corner and on the floor left of the shelves; all Z-values 0


# points on shelf corners
points['shelf1_left'] = kdl.Vector(0.637294, 0.006353, 0.643594)
points['shelf2_left'] = kdl.Vector(1.635978, 0.014217, 0.665441)
points['shelf3_left'] = kdl.Vector(2.637835, 0.022603, 0.698073)
points['shelf4_left'] = kdl.Vector(3.636703, 0.034687, 0.720942)
points['shelf4_right'] = kdl.Vector(4.628997, 0.047057, 0.740195)

# points in front of shelves
points['point6'] = kdl.Vector(1.220351, 0.008783, 1.288632)
points['point7'] = kdl.Vector(2.223905, 0.015070, 1.290442)
points['point8'] = kdl.Vector(3.598737, 0.033749, 1.303574)

# keys of shelf corner points
shelf_keys = ['shelf1_left', 'shelf2_left', 'shelf3_left', 'shelf4_left', 'shelf4_right', ]

R = get_adjusted_rot_matrix(points, shelf_keys)

for name in get_sub_dict(points, shelf_keys):
    print '{}: {}'.format(name, R.Inverse() * points[name])
#
# prints:
# shelf1_left: [    0.653156,   -0.627501, -0.00541433]
# shelf2_left: [      1.6521,   -0.624451, -0.00808298]
# shelf3_left: [     2.65449,   -0.632107,   -0.010349]
# shelf4_left: [     3.65369,   -0.630106, -0.00880811]
# shelf4_right: [     4.64623,   -0.624657, -0.00688409]
#
# Now, only set Z-values to zero.
