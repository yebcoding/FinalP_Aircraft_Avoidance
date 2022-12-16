import numpy as np
from matplotlib import pyplot as plt


# definition in the problem
d_col = 1  # 2d = 1km
q_exc = 4  # 2q = 4km
# velocity
VEL = 1
# direction
UP = 0
RIGHT = 1
DOWN = 2
LEFT = 3


route1= []
route2 = []


def calculate_bias(sx1, sy1, tx1, ty1, sx2, sy2, tx2, ty2):
    """
    to simplify the problem, move s/t+x/y+1/2 to the rectangle start from (0, 0)
    :param: Source/Target + x/y coordinate + aircraft 1/2
    :return: bias_x_min, bias_y_min, bias_x_max, bias_y_max
    """
    bias_x_min = min(sx1, tx1, sx2, tx2)
    bias_x_max = max(sx1, tx1, sx2, tx2)
    bias_y_min = min(sy1, ty1, sy2, ty2)
    bias_y_max = max(sy1, ty1, sy2, ty2)
    return bias_x_min, bias_y_min, bias_x_max, bias_y_max


def direction_change_valid(dir1, dir2):
    """
    judge whether one aircraft can change its direction from dir1 to dir2
    :param dir1: the origin direction
    :param dir2: the target direction
    :return: a bool variable, indicating whether it is valid or not
    """
    if abs(dir1 - dir2) % 4 == 2:
        return False
    elif dir1 is None or dir2 is None:
        return True
    else:
        return True


def can_exchange_message(x1, y1, x2, y2):
    """
    judge whether two aircraft are close enough to exchange messages
    :param x1: x coordinate of aircraft 1
    :param y1: y coordinate of aircraft 1
    :param x2: x coordinate of aircraft 2
    :param y2: y coordinate of aircraft 2
    :return: a bool variable, indicating whether they can exchange messages or not
    """
    if abs(x1-x2) <= (q_exc/2.0) and abs(y1-y2) <= (q_exc/2.0):
        return True
    else:
        return False


def will_collision(cx1, cy1, cx2, cy2, dir1, dir2):
    """
    judge whether two aircraft are close enough to collision in the following minute
    :param cx1: current x coordinate of aircraft 1
    :param cy1: current y coordinate of aircraft 1
    :param cx2: current x coordinate of aircraft 2
    :param cy2: current y coordinate of aircraft 2
    :param dir1: direction of aircraft 1
    :param dir2: direction of aircraft 2
    :return: a bool variable, indicating whether they will collision or not
    """
    new_cx1, new_cy1 = update_position(dir1, cx1, cy1)
    new_cx2, new_cy2 = update_position(dir2, cx2, cy2)
    if abs(new_cx1-new_cx2) <= (d_col/2.0) and abs(new_cy1-new_cy2) <= (d_col/2.0):
        # two aircraft will go to the same position
        return True

    if new_cx1 == cx2 and new_cy1 == cy2 and new_cx2 == cx1 and new_cy2 == cy1:
        # two aircraft will exchange position,
        # that current x/y different from each other only 1
        # and their position is opposite
        return True

    return False


def manhattan_distance(x1, y1, x2, y2):
    """
    calculate the Manhatten distance between aircraft 1 and aircraft 2
    :param x1: x coordinate of aircraft 1
    :param y1: y coordinate of aircraft 1
    :param x2: x coordinate of aircraft 2
    :param y2: y coordinate of aircraft 2
    :return: the Manhatten distance
    """
    return abs(x1-x2) + abs(y1-y2)


def euclidean_distance(x1, y1, x2, y2):
    """
    calculate the Euclidean distance between aircraft 1 and aircraft 2
    :param x1: x coordinate of aircraft 1
    :param y1: y coordinate of aircraft 1
    :param x2: x coordinate of aircraft 2
    :param y2: y coordinate of aircraft 2
    :return: the Euclidean distance
    """
    return np.sqrt((x1-x2)**2 + (y1-y2)**2)


def output_status(time, cx1, cy1, tx1, ty1, cx2, cy2, tx2, ty2, dir1, dir2, bias_x, bias_y, next=True, completed=False):
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
    dir_list = ['UP', 'RIGHT', 'DOWN', 'LEFT']
    pos_string = 'next' if next else 'last'
    print("Time =", time)
    if dir1 is None and cx1 == tx1 and cy1 == ty1:
        route1.append((tx1+bias_x,ty1+bias_y))
        print("aircraft 1: current position (%d, %d), target position (%d, %d)," %
              (cx1 + bias_x, cy1 + bias_y, tx1 + bias_x, ty1 + bias_y), "completed!")
    elif dir1 is None:
        print("aircraft 1: current position (%d, %d), target position (%d, %d)," %
              (cx1 + bias_x, cy1 + bias_y, tx1 + bias_x, ty1 + bias_y), "no direction(initialized)")
    else:
        route1.append((cx1+bias_x, cy1+bias_y))
        print("aircraft 1: current position (%d, %d), target position (%d, %d)," %
              (cx1+bias_x, cy1+bias_y, tx1+bias_x, ty1+bias_y), pos_string, "direction", dir_list[dir1])

    if dir2 is None and cx2 == tx2 and cy2 == ty2:
        route2.append((tx2+bias_x,ty2+bias_y))
        print("aircraft 2: current position (%d, %d), target position (%d, %d)," %
              (cx2 + bias_x, cy2 + bias_y, tx2 + bias_x, ty2 + bias_y), "completed!")
    elif dir2 is None:
        print("aircraft 2: current position (%d, %d), target position (%d, %d)," %
              (cx2 + bias_x, cy2 + bias_y, tx2 + bias_x, ty2 + bias_y), "no direction(initialized)")
    else:
        route2.append((cx2+bias_x, cy2+bias_y))
        print("aircraft 2: current position (%d, %d), target position (%d, %d)," %
              (cx2+bias_x, cy2+bias_y, tx2+bias_x, ty2+bias_y), pos_string, "direction", dir_list[dir2])

    if completed:
        print()
        print("Completed!")
        print("Total time cost =", time)
        x1 = [x[0] for x in route1]
        y1 = [x[1] for x in route1]
        x2 = [x[0] for x in route2]
        y2 = [x[1] for x in route2]
        plt.plot(x2,y2, label = "line 2")
        plt.plot(x1,y1, label  = "line 1")
        plt.legend()
        plt.show()
        # print(route1)
    else:
        print()


def update_position(dir, cx, cy):
    """
    update the position of the aircraft
    :param dir: direction
    :param cx: current x
    :param cy: current y
    :param tx: target x
    :param ty: target y
    :return: the updated position
    """
    if dir is None:
        return cx, cy
    elif dir == UP:
        return cx, cy+VEL
    elif dir == DOWN:
        return cx, cy-VEL
    elif dir == LEFT:
        return cx-VEL, cy
    else:
        return cx+VEL, cy


def find_direction(cx, cy, tx, ty):
    """
    find a suitable direction,
    the aircraft always fly in the direction that deviates the most from the target
    :param cx: current x
    :param cy: current y
    :param tx: target x
    :param ty: target y
    :return: UP, DOWN, LEFT, or RIGHT
    """
    if cx == tx and cy == ty:
        return None

    if abs(cx - tx) >= abs(cy - ty):
        return LEFT if cx > tx else RIGHT
    else:
        return DOWN if cy > ty else UP


def get_next_valid_direction(cur_dir):
    """
    get the next valid direction choises
    :param cur_dir:
    :return: a list consisting of each direction the aircraft can change
    """
    if cur_dir is None:
        return [UP, RIGHT, DOWN, LEFT]
    else:
        return [(cur_dir-1)%4, cur_dir, (cur_dir+1)%4]

