import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d

# definition in the problem
d_col = 1  # 2d = 1km
q_exc = 4  # 2q = 4km
# velocity
VEL = 1
# direction
UP = 0
RIGHT = 1
ABOVE = 2
DOWN = 3
LEFT = 4
LOW = 5

# infinite
INFTY = 100000

route1= []
route2 = []


def calculate_bias(sx1, sy1, sz1, tx1, ty1, tz1, sx2, sy2, sz2, tx2, ty2, tz2 ):
    """
    to simplify the problem, move s/t+x/y/z +1/2 to the rectangle start from (0, 0, 0)
    :param: Source/Target + x/y/z coordinate + aircraft 1/2
    :return: bias_x_min, bias_y_min, bias_x_max, bias_y_max, bias_z_min, bias_z_min
    """
    bias_x_min = min(sx1, tx1, sx2, tx2)
    bias_x_max = max(sx1, tx1, sx2, tx2)
    bias_y_min = min(sy1, ty1, sy2, ty2)
    bias_y_max = max(sy1, ty1, sy2, ty2)
    bias_z_min = min(sz1, tz1, sz2, tz2)
    bias_z_max = max(sz1, tz1, sz2, tz2)
    return bias_x_min, bias_y_min, bias_z_min, bias_x_max, bias_y_max, bias_z_max


def direction_change_valid(dir1, dir2):
    """
    judge whether one aircraft can change its direction from dir1 to dir2
    :param dir1: the origin direction
    :param dir2: the target direction
    :return: a bool variable, indicating whether it is valid or not
    """
    if dir1 is None or dir2 is None:
        return True
    elif abs(dir1 - dir2) % 6 == 3:
        return False
    else:
        return True


def can_exchange_message(x1, y1, z1, x2, y2,z2):
    """
    judge whether two aircraft are close enough to exchange messages
    :param x1: x coordinate of aircraft 1
    :param y1: y coordinate of aircraft 1
    :param z1: z coordinate of aircraft 1
    :param x2: x coordinate of aircraft 2
    :param y2: y coordinate of aircraft 2
    :param z2: x coordinate of aircraft 2
    :return: a bool variable, indicating whether they can exchange messages or not
    """
    if abs(x1-x2) <= (q_exc/2.0) and abs(y1-y2) <= (q_exc/2.0) and abs(z1-z2) <= (q_exc/2.0):
        return True
    else:
        return False


def will_collision(cx1, cy1, cz1, cx2, cy2, cz2, dir1, dir2):
    """
    judge whether two aircraft are close enough to collision in the following minute
    :param cx1: current x coordinate of aircraft 1
    :param cy1: current y coordinate of aircraft 1
    :param cz1: current z coordinate of aircraft 1
    :param cx2: current x coordinate of aircraft 2
    :param cy2: current y coordinate of aircraft 2
    :param cz2: current z coordinate of aircraft 2
    :param dir1: direction of aircraft 1
    :param dir2: direction of aircraft 2
    :return: a bool variable, indicating whether they will collision or not
    """
    new_cx1, new_cy1, new_cz1 = update_position(dir1, cx1, cy1, cz1)
    new_cx2, new_cy2, new_cz2 = update_position(dir2, cx2, cy2, cz2)
    if abs(new_cx1-new_cx2) <= (d_col/2.0) and abs(new_cy1-new_cy2) <= (d_col/2.0) and abs(new_cz1-new_cz2) <= (d_col/2.0):
        # two aircraft will go to the same position
        return True

    if new_cx1 == cx2 and new_cy1 == cy2 and new_cx2 == cx1 and new_cy2 == cy1 and new_cz1 == cz1 and new_cz2 == cz2:
        # two aircraft will exchange position,
        # that current x/y different from each other only 1
        # and their position is opposite
        return True

    return False


def manhattan_distance(x1, y1, z1, x2, y2, z2):
    """
    calculate the Manhatten distance between aircraft 1 and aircraft 2
    :param x1: x coordinate of aircraft 1
    :param y1: y coordinate of aircraft 1
    :param z1: z coordinate of aircraft 1
    :param x2: x coordinate of aircraft 2
    :param y2: y coordinate of aircraft 2
    :param z2: z coordinate of aircraft 2
    :return: the Manhatten distance
    """
    return abs(x1-x2) + abs(y1-y2) + abs(z1-z2)


def euclidean_distance(x1, y1, x2, y2, z1, z2):
    """
    calculate the Euclidean distance between aircraft 1 and aircraft 2
    :param x1: x coordinate of aircraft 1
    :param y1: y coordinate of aircraft 1
    :param x2: x coordinate of aircraft 2
    :param y2: y coordinate of aircraft 2
    :return: the Euclidean distance
    """
    return np.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)


def output_status(time, cx1, cy1, cz1, tx1, ty1, tz1, cx2, cy2, cz2, tx2, ty2, tz2, dir1, dir2, bias_x, bias_y, bias_z, next=True, completed=False):
    """
    output the status of the two aircrafts
    :param time: current time
    :param dir1/2: last direction to this current position, or the next direction starting from the current position
    :param next: False for A* algorithm, True for others
                    if True, outputs direction as the next direction starting from the current position
                    if False, outputs the last direction to this current position
    :param completed: whether both of the two aircraft reach their target
    :return:
    """
    dir_list = ['UP', 'RIGHT', 'ABOVE', 'DOWN', 'LEFT', 'LOW']
    pos_string = 'next' if next else 'last'
    print("Time =", time)
    if dir1 is None and cx1 == tx1 and cy1 == ty1 and cz1 == tz1:
        route1.append((tx1 + bias_x, ty1 + bias_y, tz1 + bias_z))
        print("aircraft 1: current position (%d, %d, %d), target position (%d, %d, %d)," %
              (cx1 + bias_x, cy1 + bias_y, cz1+ bias_z, tx1 + bias_x, ty1 + bias_y, tz1 + bias_z), "completed!")
    elif dir1 is None:
        print("aircraft 1: current position (%d, %d, %d), target position (%d, %d, %d)," %
              (cx1 + bias_x, cy1 + bias_y, cz1+ bias_z, tx1 + bias_x, ty1 + bias_y, tz1 + bias_z), "no direction(initialized)")
    else:
        route1.append((cx1 + bias_x, cy1+bias_y, cz1+ bias_z))
        print("aircraft 1: current position (%d, %d, %d), target position (%d, %d, %d)," %
              (cx1 + bias_x, cy1 + bias_y, cz1+ bias_z, tx1 + bias_x, ty1 + bias_y, tz1 + bias_z), pos_string, "direction", dir_list[dir1])

    if dir2 is None and cx2 == tx2 and cy2 == ty2 and cz2 == tz2:
        route2.append((tx2 + bias_x, ty2 + bias_y, tz2 + bias_z))
        print("aircraft 2: current position (%d, %d, %d), target position (%d, %d, %d)," %
              (cx2 + bias_x, cy2 + bias_y, cz2+ bias_z, tx2 + bias_x, ty2 + bias_y, tz2 + bias_z), "completed!")
    elif dir2 is None:
        print("aircraft 2: current position (%d, %d, %d), target position (%d, %d, %d)," %
              (cx2 + bias_x, cy2 + bias_y, cz2+ bias_z, tx2 + bias_x, ty2 + bias_y, tz2 + bias_z), "no direction(initialized)")
    else:
        route2.append((cx2+bias_x, cy2+bias_y, cz2+ bias_z))
        print("aircraft 2: current position (%d, %d, %d), target position (%d, %d, %d)," %
              (cx2+bias_x, cy2+bias_y, cz2+ bias_z, tx2 + bias_x, ty2 + bias_y, tz2 + bias_z), pos_string, "direction", dir_list[dir2])

    if completed:
    #     print()
    #     print("Completed!")
    #     print("Total time cost =", time)
        x1 = [x[0] for x in route1]
        y1 = [x[1] for x in route1]
        z1 = [x[2] for x in route1]
        x2 = [x[0] for x in route2]
        y2 = [x[1] for x in route2]
        z2 = [x[2] for x in route2]
        ax = plt.axes(projection='3d')
        ax.plot3D(x1,y1,z1, label  = "line 1")
        ax.plot3D(x2,y2,z2, label  = "line 2")
        plt.legend()
        plt.show()
    # else:
    #     print()


def update_position(dir, cx, cy, cz):
    """
    update the position of the aircraft
    :param dir: direction
    :param cx: current x
    :param cy: current y
    :param tx: target x
    :param ty: target y
    :param cz: current z
    :param tz: target z
    :return: the updated position
    """
    if dir is None:
        return cx, cy, cz
    elif dir == UP:
        return cx, cy+VEL, cz
    elif dir == DOWN:
        return cx, cy-VEL, cz
    elif dir == LEFT:
        return cx-VEL, cy, cz
    elif dir == RIGHT:
        return cx+VEL, cy, cz
    elif dir == ABOVE:
        return cx, cy, cz+VEL
    else:
        return cx, cy, cz-VEL


def find_direction(cx, cy, cz, tx, ty, tz):
    """
    find a suitable direction,
    the aircraft always fly in the direction that deviates the most from the target
    :param cx: current x
    :param cy: current y
    :param cz: current z
    :param tx: target x
    :param ty: target y
    :param tz: target z
    :return: UP, DOWN, LEFT, RIGHT, ABOVE or LOW
    """
    if cx == tx and cy == ty and cz == tz:
        return None

    # if abs(cx - tx) >= abs(cy - ty):
    #     return LEFT if cx > tx else RIGHT
    # else:
    #     return DOWN if cy > ty else UP
    if max(abs(cx - tx), max(abs(cy - ty), abs(cz-tz))) == abs(cx - tx):
        return LEFT if cx > tx else RIGHT
    elif max(abs(cx - tx), max(abs(cy - ty), abs(cz-tz))) == abs(cy - ty):
        return DOWN if cy > ty else UP
    else:
        return LOW if cz > tz else ABOVE


def get_next_valid_direction(cur_dir):
    """
    get the next valid direction choises
    :param cur_dir:
    :return: a list consisting of each direction the aircraft can change
    """
    if cur_dir is None:
        return [UP, RIGHT, ABOVE, DOWN, LEFT, LOW]
    else:
        return [(cur_dir-1)%6, cur_dir, (cur_dir+1)%6]

