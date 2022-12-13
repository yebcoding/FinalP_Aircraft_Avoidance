import preliminaries3
from preliminaries3 import *

# velocity
VEL = preliminaries3.VEL
# direction
UP = preliminaries3.UP
RIGHT = preliminaries3.RIGHT
ABOVE = preliminaries3.ABOVE
DOWN = preliminaries3.DOWN
LEFT = preliminaries3.LEFT
LOW = preliminaries3.LOW


def simple_controller(time, cx1, cx2, cy1, cy2, cz1, cz2, tx1, tx2, ty1, ty2, tz1, tz2, dir1, dir2, bias_x, bias_y, bias_z):
    """
    design a simple controller, and call the function recursively until both aircraft 1 and 2 arrive their targets
    :param time: current time
    :param cx1, cx2, cy1, cy2, cz1, cz2: Current x/y/z of aircraft 1/2
    :param tx1, tx2, ty1, ty2, tz1, tz2: Target x/y/z of aircraft 1/2
    :param dir1: current direction of aircraft 1
    :param dir2: current direction of aircraft 2
    :param bias_x: bias for input x coordinates
    :param bias_y: bias for input y coordinates
    :param bias_z: bias for input y coordinates
    :return: no return,
    """
    if cx1==tx1 and cy1==ty1 and cx2==tx2 and cy2==ty2 and cz1 == tz1 and cz2 == tz2:
        output_status(time, cx1, cy1, cz1, tx1, ty1, tz1, cx2, cy2, cz2, tx2, ty2, tz2, dir1, dir2, bias_x, bias_y, bias_z, completed=True)
        return

    output_status(time, cx1, cy1, cz1, tx1, ty1, tz1, cx2, cy2, cz2, tx2, ty2, tz2, dir1, dir2, bias_x, bias_y, bias_z)
    new_cx1, new_cy1, new_cz1 = update_position(dir1, cx1, cy1,cz1)
    new_cx2, new_cy2, new_cz2 = update_position(dir2, cx2, cy2,cz2)
    new_dir1 = find_direction(new_cx1, new_cy1, new_cz1, tx1, ty1,tz1)
    new_dir2 = find_direction(new_cx2, new_cy2, new_cz2, tx2, ty2,tz2)
    if direction_change_valid(dir1, new_dir1) and direction_change_valid(dir2, new_dir2):
        simple_controller(time + 1, new_cx1, new_cx2, new_cy1, new_cy2, new_cz1, new_cz2,  tx1, tx2, ty1, ty2, tz1, tz2, new_dir1, new_dir2, bias_x, bias_y, bias_z)


if __name__ == '__main__':
    sx1, sy1, sz1 = 0, 0 ,0 # source x, y, z coordinate of aircraft 1
    tx1, ty1, tz1 = 10, 10, -10  # target x, y, z coordinate of aircraft 1
    sx2, sy2, sz2 = 0, 0, -10  # source x, y, z coordinate of aircraft 2
    tx2, ty2, tz2 = 10, 10, 0  # target x, y, z coordinate of aircraft 2

    bias_x_min, bias_y_min, bias_z_min,  bias_x_max, bias_y_max, bias_z_max = calculate_bias(sx1, sy1, sz1, tx1, ty1, tz1, sx2, sy2, sz2, tx2, ty2, tz2)
    sx1, tx1, sx2, tx2 = sx1 - bias_x_min, tx1 - bias_x_min, sx2 - bias_x_min, tx2 - bias_x_min
    sy1, ty1, sy2, ty2 = sy1 - bias_y_min, ty1 - bias_y_min, sy2 - bias_y_min, ty2 - bias_y_min
    sz1, tz1, sz2, tz2 = sz1 - bias_z_min, tz1 - bias_z_min, sz2 - bias_z_min, tz2 - bias_z_min

    simple_controller(time=0, cx1=sx1, cx2=sx2, cy1=sy1, cy2=sy2, cz1 = sz1, cz2 = sz2, tx1=tx1, tx2=tx2, ty1=ty1, ty2=ty2, tz1 = tz1, tz2 = tz2,
                      dir1=find_direction(sx1, sy1, sz1, tx1, ty1, tz1), dir2=find_direction(sx2, sy2, sz2, tx2, ty2,tz2),
                      bias_x=bias_x_min, bias_y=bias_y_min, bias_z = bias_z_min)
