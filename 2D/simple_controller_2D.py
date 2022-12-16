import preliminaries2
from preliminaries2 import *

# velocity
VEL = preliminaries2.VEL
# direction
UP = preliminaries2.UP
RIGHT = preliminaries2.RIGHT
DOWN = preliminaries2.DOWN
LEFT = preliminaries2.LEFT


def simple_controller(time, cx1, cx2, cy1, cy2, tx1, tx2, ty1, ty2, dir1, dir2, bias_x, bias_y):
    """
    design a simple controller, and call the function recursively until both aircraft 1 and 2 arrive their targets
    :param time: current time
    :param cx1, cx2, cy1, cy2: Current x/y of aircraft 1/2
    :param tx1, tx2, ty1, ty2: Target x/y of aircraft 1/2
    :param dir1: current direction of aircraft 1
    :param dir2: current direction of aircraft 2
    :param bias_x: bias for input x coordinates
    :param bias_y: bias for input y coordinates
    :return: no return,
    """
    if cx1==tx1 and cy1==ty1 and cx2==tx2 and cy2==ty2:
        output_status(time, cx1, cy1, tx1, ty1, cx2, cy2, tx2, ty2, dir1, dir2, bias_x, bias_y, completed=True)
        return

    output_status(time, cx1, cy1, tx1, ty1, cx2, cy2, tx2, ty2, dir1, dir2, bias_x, bias_y)
    new_cx1, new_cy1 = update_position(dir1, cx1, cy1)
    new_cx2, new_cy2 = update_position(dir2, cx2, cy2)
    new_dir1 = find_direction(new_cx1, new_cy1, tx1, ty1)
    new_dir2 = find_direction(new_cx2, new_cy2, tx2, ty2)
    if direction_change_valid(dir1, new_dir1) and direction_change_valid(dir2, new_dir2):
        simple_controller(time + 1, new_cx1, new_cx2, new_cy1, new_cy2, tx1, tx2, ty1, ty2, new_dir1, new_dir2, bias_x, bias_y)


if __name__ == '__main__':
    sx1, sy1 = 0, 0  # source x, y coordinate of aircraft 1
    tx1, ty1 = 0, 10  # target x, y coordinate of aircraft 1
    sx2, sy2 = -5, 5  # source x, y coordinate of aircraft 2
    tx2, ty2 = 5, 5 # target x, y coordinate of aircraft 2

    bias_x_min, bias_y_min, bias_x_max, bias_y_max = calculate_bias(sx1, sy1, tx1, ty1, sx2, sy2, tx2, ty2)
    sx1, tx1, sx2, tx2 = sx1 - bias_x_min, tx1 - bias_x_min, sx2 - bias_x_min, tx2 - bias_x_min
    sy1, ty1, sy2, ty2 = sy1 - bias_y_min, ty1 - bias_y_min, sy2 - bias_y_min, ty2 - bias_y_min

    simple_controller(time=0, cx1=sx1, cx2=sx2, cy1=sy1, cy2=sy2, tx1=tx1, tx2=tx2, ty1=ty1, ty2=ty2,
                      dir1=find_direction(sx1, sy1, tx1, ty1), dir2=find_direction(sx2, sy2, tx2, ty2),
                      bias_x=bias_x_min, bias_y=bias_y_min)
