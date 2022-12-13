import preliminaries2
from preliminaries2 import *

# velocity
VEL = preliminaries2.VEL
# direction
UP = preliminaries2.UP
RIGHT = preliminaries2.RIGHT
DOWN = preliminaries2.DOWN
LEFT = preliminaries2.LEFT


def complex_controller(time, cx1, cx2, cy1, cy2, tx1, tx2, ty1, ty2, dir1, dir2, bias_x, bias_y, last_dir1, last_dir2):
    """
    design a controller, and call the function recursively until both aircraft 1 and 2 arrive their targets
    :param time: current time
    :param cx1, cx2, cy1, cy2: Current x/y of aircraft 1/2
    :param tx1, tx2, ty1, ty2: Target x/y of aircraft 1/2
    :param dir1: current direction of aircraft 1
    :param dir2: current direction of aircraft 2
    :param bias_x: bias for input x coordinates
    :param bias_y: bias for input y coordinates
    :param last_dir1: direction for aircraft 1, for the last time
    :param last_dir2: direction for aircraft 2, for the last time
    :return: no return,
    """
    if cx1==tx1 and cy1==ty1 and cx2==tx2 and cy2==ty2:
        output_status(time, cx1, cy1, tx1, ty1, cx2, cy2, tx2, ty2, dir1, dir2, bias_x, bias_y, completed=True)
        return

    if can_exchange_message(cx1, cy1, cx2, cy2) == False:
        # if they are far away from each other, then do what simple strategy ask to do
        output_status(time, cx1, cy1, tx1, ty1, cx2, cy2, tx2, ty2, dir1, dir2, bias_x, bias_y)
        new_cx1, new_cy1 = update_position(dir1, cx1, cy1)
        new_cx2, new_cy2 = update_position(dir2, cx2, cy2)
        new_dir1 = find_direction(new_cx1, new_cy1, tx1, ty1)
        new_dir2 = find_direction(new_cx2, new_cy2, tx2, ty2)
        if direction_change_valid(dir1, new_dir1) and direction_change_valid(dir2, new_dir2):
            complex_controller(time + 1, new_cx1, new_cx2, new_cy1, new_cy2, tx1, tx2, ty1, ty2, new_dir1, new_dir2,
                              bias_x, bias_y, dir1, dir2)
        return

    # ------------------------------------
    # if it does NOT satisfy both two conditions above (else the function will return)
    # that is not both aircrafts arrives the target
    # AND they are close enough to exchange messages
    if will_collision(cx1, cy1, cx2, cy2, dir1, dir2) == False:
        # if they will not collision according to the direction chosen by the simple strategy
        output_status(time, cx1, cy1, tx1, ty1, cx2, cy2, tx2, ty2, dir1, dir2, bias_x, bias_y)
        new_cx1, new_cy1 = update_position(dir1, cx1, cy1)
        new_cx2, new_cy2 = update_position(dir2, cx2, cy2)
        new_dir1 = find_direction(new_cx1, new_cy1, tx1, ty1)
        new_dir2 = find_direction(new_cx2, new_cy2, tx2, ty2)
        if direction_change_valid(dir1, new_dir1) and direction_change_valid(dir2, new_dir2):
            complex_controller(time + 1, new_cx1, new_cx2, new_cy1, new_cy2, tx1, tx2, ty1, ty2,
                               new_dir1, new_dir2, bias_x, bias_y, dir1, dir2)
    else:
        # if they will collision, then their direction must be changed
        if dir1 is None or dir2 is None:
            # one of aircraft 1/2 arrive the target
            new_cx1, new_cy1 = update_position(dir1, cx1, cy1)
            new_cx2, new_cy2 = update_position(dir2, cx2, cy2)
            new_dir1 = find_direction(new_cx1, new_cy1, tx1, ty1)
            new_dir2 = find_direction(new_cx2, new_cy2, tx2, ty2)
            output_status(time, cx1, cy1, tx1, ty1, cx2, cy2, tx2, ty2, dir1, dir2, bias_x, bias_y)
            complex_controller(time + 1, new_cx1, new_cx2, new_cy1, new_cy2, tx1, tx2, ty1, ty2, new_dir1, new_dir2,
                               bias_x, bias_y, dir1, dir2)

        elif abs(dir1 - dir2) % 2 == 0:
            # if the two aircraft fly towards each other
            dir1 = (dir1 + 1) % 4  # turn right
            dir2 = (dir2 + 1) % 4  # turn right
            new_cx1, new_cy1 = update_position(dir1, cx1, cy1)
            new_cx2, new_cy2 = update_position(dir2, cx2, cy2)
            new_dir1 = find_direction(new_cx1, new_cy1, tx1, ty1)
            new_dir2 = find_direction(new_cx2, new_cy2, tx2, ty2)
            if direction_change_valid(dir1, new_dir1) and direction_change_valid(dir2, new_dir2):
                output_status(time, cx1, cy1, tx1, ty1, cx2, cy2, tx2, ty2, dir1, dir2, bias_x, bias_y)
                complex_controller(time + 1, new_cx1, new_cx2, new_cy1, new_cy2, tx1, tx2, ty1, ty2, new_dir1, new_dir2,
                                   bias_x, bias_y, dir1, dir2)
            else:
                dir1 = (dir1 - 1) % 4  # turn left
                dir2 = (dir2 + 1) % 4  # turn left
                new_cx1, new_cy1 = update_position(dir1, cx1, cy1)
                new_cx2, new_cy2 = update_position(dir2, cx2, cy2)
                new_dir1 = find_direction(new_cx1, new_cy1, tx1, ty1)
                new_dir2 = find_direction(new_cx2, new_cy2, tx2, ty2)
                output_status(time, cx1, cy1, tx1, ty1, cx2, cy2, tx2, ty2, dir1, dir2, bias_x, bias_y)
                complex_controller(time + 1, new_cx1, new_cx2, new_cy1, new_cy2, tx1, tx2, ty1, ty2, new_dir1, new_dir2,
                                   bias_x, bias_y, dir1, dir2)

        else:
            # if the one aircraft is flying vertically, and another horizontally, and they will collision
            # then my strategy is that:
            # let the aircraft which is further to its target to change direction
            # it change its direction to another aircraft, that is it fly 'behind' another aircraft
            dist1 = manhattan_distance(cx1, cy1, tx1, ty1)
            dist2 = manhattan_distance(cx2, cy2, tx2, ty2)

            if dist1 <= dist2:
                # aircraft 2 should change its position
                if direction_change_valid(last_dir2, (dir1 + 2) % 4):
                    dir2 = (dir1 + 2) % 4
                else:
                    dir2 = last_dir2
                output_status(time, cx1, cy1, tx1, ty1, cx2, cy2, tx2, ty2, dir1, dir2, bias_x, bias_y)
                new_cx1, new_cy1 = update_position(dir1, cx1, cy1)
                new_cx2, new_cy2 = update_position(dir2, cx2, cy2)
                new_dir1 = find_direction(new_cx1, new_cy1, tx1, ty1)
                new_dir2 = find_direction(new_cx2, new_cy2, tx2, ty2)
                complex_controller(time + 1, new_cx1, new_cx2, new_cy1, new_cy2, tx1, tx2, ty1, ty2,
                                   new_dir1, new_dir2, bias_x, bias_y, dir1, dir2)

            else:
                # aircraft 1 should change its position
                if direction_change_valid(last_dir1, (dir2 + 2) % 4):
                    dir1 = (dir2 + 2) % 4
                else:
                    dir1 = last_dir1
                output_status(time, cx1, cy1, tx1, ty1, cx2, cy2, tx2, ty2, dir1, dir2, bias_x, bias_y)
                new_cx1, new_cy1 = update_position(dir1, cx1, cy1)
                new_cx2, new_cy2 = update_position(dir2, cx2, cy2)
                new_dir1 = find_direction(new_cx1, new_cy1, tx1, ty1)
                new_dir2 = find_direction(new_cx2, new_cy2, tx2, ty2)
                complex_controller(time + 1, new_cx1, new_cx2, new_cy1, new_cy2, tx1, tx2, ty1, ty2,
                                   new_dir1, new_dir2, bias_x, bias_y, dir1, dir2)


if __name__ == '__main__':
    sx1, sy1 = 10, 0  # source x, y coordinate of aircraft 1
    tx1, ty1 = 0, 10  # target x, y coordinate of aircraft 1
    sx2, sy2 = 0, 0  # source x, y coordinate of aircraft 2
    tx2, ty2 = 0, 10    # target x, y coordinate of aircraft 2

    bias_x_min, bias_y_min, bias_x_max, bias_y_max = calculate_bias(sx1, sy1, tx1, ty1, sx2, sy2, tx2, ty2)
    sx1, tx1, sx2, tx2 = sx1 - bias_x_min, tx1 - bias_x_min, sx2 - bias_x_min, tx2 - bias_x_min
    sy1, ty1, sy2, ty2 = sy1 - bias_y_min, ty1 - bias_y_min, sy2 - bias_y_min, ty2 - bias_y_min

    complex_controller(time=0, cx1=sx1, cx2=sx2, cy1=sy1, cy2=sy2, tx1=tx1, tx2=tx2, ty1=ty1, ty2=ty2,
                      dir1=find_direction(sx1, sy1, tx1, ty1), dir2=find_direction(sx2, sy2, tx2, ty2),
                      bias_x=bias_x_min, bias_y=bias_y_min, last_dir1=None, last_dir2=None)
