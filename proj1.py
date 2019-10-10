
import racetrack as rt
import math
import sys


# Global variables

infinity = float('inf')

g_fline = False
g_walls = False
grid = []
straight = False
first_time = True
start_x = 0
start_y = 0
reached_fline = False


def h_proj1(state, fline, walls):
    """
    This first time this function called will test if there is a striaght line from start to
    finish line. If straight line exists will use diagonal distance as heuristic. Otherwise will
    use straight line distance as heuristic.

    :param state:
    :param fline:
    :param walls:
    :return: hval
    """
    global g_fline, g_walls, first_time, start_x, start_y, reached_fline,straight

    ((x, y), (u, v)) = state

    # find closest point from finish line to the current state
    x_f,y_f = cloest_point((x,y),fline)

    # calculate diagonal distance if no walls
    if not rt.crash(((x,y),(x_f,y_f)),walls) and first_time:
        edist_grid((x,y),fline,walls)
        start_x = x
        start_y = y
        straight=True
        first_time = False

    # calculate straight line distance if wall exist
    if fline != g_fline or walls != g_walls or grid == []:
        edist_grid_1(fline, walls)
        first_time=False

    hval = float(grid[x][y])

    # add a small penalty to favor short stopping distances
    au = abs(u);
    av = abs(v);
    sdu = au * (au - 1) / 2.0
    sdv = av * (av - 1) / 2.0
    sd = max(sdu, sdv)
    penalty = sd / 10.0
    # compute location after fastest stop, and add a penalty if it goes through a wall
    if u < 0: sdu = -sdu
    if v < 0: sdv = -sdv
    sx = x + sdu
    sy = y + sdv
    if rt.crash([(x, y), (sx, sy)], walls):
        penalty += math.sqrt(au ** 2 + av ** 2)

    hval = max(hval + penalty, sd)

    if not straight:
        # add reward to favor points that are closer to goal if not go through wall
        sdist = edist_diagonal((x - u, y - v), fline)
        tdist = edist_diagonal((x, y), fline)
        reward = 0
        if not rt.crash(((x, y), (x-u, y-v)), walls) and sdist > tdist:
            reward = -math.sqrt(2)
            hval += reward

        if not reached_fline:
            if rt.intersect(((x-u,y-v),(x,y)),fline):
                reward = -math.sqrt(2)
                hval += reward

    elif straight:   # use straight line if no walls in between
        # tie breaking for multiple similar heuristics
        cloest_x, cloest_y = cloest_point((x, y), fline)
        dx1 = x - cloest_x
        dy1 = y - cloest_y
        dx2 = start_x - cloest_x
        dy2 = start_y - cloest_y
        cross = abs(dx1 * dy2 - dx2 * dy1)
        hval += cross * 0.001

        # if straight line reached goal expand other nodes will have penalty for early stopping
        if not reached_fline:
            if point_on_line((x-u,y-v), fline):
                reached_fline = True
        if reached_fline:
            return hval+math.sqrt(2)

    return hval


def edist_grid(point, fline, walls):
    """
    this calculate diagonal distance from start to goal in straight line distance exist.
    It will only store the distance in restricted area surround the straight line.
    Other points will be infinity.

    :param point:
    :param fline:
    :param walls:
    :return: grid
    """
    global grid, g_fline, g_walls, xmax, ymax

    xmax = max([max(x, x1) for ((x, y), (x1, y1)) in walls])
    ymax = max([max(y, y1) for ((x, y), (x1, y1)) in walls])

    (x, y) = point
    cloest_x, cloest_y = cloest_point(point, fline)

    print('computing edist grid_straight', end=' ');
    sys.stdout.flush()
    grid = [[infinity for y in range(ymax + 1)] for x in range(xmax + 1)]

    # calculate diagonal distance in restricted area
    for i in range(min(x, cloest_x), max(x, cloest_x) + 1):
        for j in range(min(y, cloest_y), max(y, cloest_y) + 1):
            print('.', end='');
            sys.stdout.flush()
            dist_line = dist_to_line((i, j), ((x, y), (cloest_x, cloest_y)))
            # tdraw.draw_dot((i,j))
            if dist_line < 0.4 and dist_line >= 0:
                grid[i][j] = edist_diagonal((i, j), fline)

    print(' done')
    g_fline = fline
    g_walls = walls
    return grid


def dist_to_line(point, line):
    """
    This method calculates the distance from a point to a given line.

    :param point:
    :param line:
    :return: dist
    """
    (x, y) = point
    ((x1, y1), (x2, y2)) = line
    # calculate wall's line function for distance calculation with the given point y=mx+b
    dist = (float)(
        (abs((y2 - y1) * x - (x2 - x1) * y + x2 * y1 - y2 * x1)) / math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2))

    return dist


def point_on_line(point, line):
    """
    This method tests if a point is on given line or not.

    :param point:
    :param line:
    :return: True/False
    """
    (i, j) = point
    ((x1, y1), (x2, y2)) = line

    # avoid division by 0
    if x1 == x2:
        return i == x1

    # calculate y=mx+b
    m = (y2 - y1) / (x2 - x1)
    b = y1 - m * x1

    # test if points on the line
    tmp_y = m * i + b
    if tmp_y == j:
        return True

    return False


def edist_diagonal(point, fline):
    """
    This method calculate diagonal distance from a point to finish line

    :param point:
    :param fline:
    :return: value of distance
    """
    (x, y) = point
    x1, y1 = cloest_point(point, fline)

    dx = abs(x - x1)
    dy = abs(y - y1)

    val = float(1 * (dx + dy) + (1 - 2) * min(dx, dy))

    return val


def point_dist(point1, point2):
    """
    This method calculate straight line distance between two points

    :param point1:
    :param point2:
    :return: dist
    """
    (x1, y1) = point1
    (x2, y2) = point2

    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    return dist


def cloest_point(point, fline):
    """
    This method calculate shortest distance from a given point to finish line.

    :param point:
    :param fline:
    :return: x, y coordinate
    """
    (x, y) = point
    ((x1, y1), (x2, y2)) = fline
    # tesst if points on forms 90 degree angle with line
    if x1 == x2:  # fline vertical
        cloest_x = x1
        if y > min(y1, y2) and y < max(y1, y2):
            # find x points on fline which is the min distance
            cloest_y = y
        else:
            if (y - y1) < (y - y2):
                cloest_y = y1
            else:
                cloest_y = y2
    if y1 == y2:  # fline horizontal
        cloest_y = y1
        if x > min(x1, x2) and x < max(x1, x2):
            # find x points on fline which is the min distance
            cloest_x = x
        else:
            if (x - x1) < (x - x2):
                cloest_x = x1
            else:
                cloest_x = x2

    return cloest_x, cloest_y


def edistw_to_finish(point, fline, walls):
    """
    straight-line distance from (x,y) to the finish line ((x1,y1),(x2,y2)).
    Return infinity if there's no way to do it without intersecting a wall
    """

    (x, y) = point
    ((x1, y1), (x2, y2)) = fline
    # make a list of distances to each reachable point in fline
    if x1 == x2:  # fline is vertical, so iterate over y
        ds = [math.sqrt((x1 - x) ** 2 + (y3 - y) ** 2) \
              for y3 in range(min(y1, y2), max(y1, y2) + 1) \
              if not rt.crash(((x, y), (x1, y3)), walls)]
    else:  # fline is horizontal, so iterate over x
        ds = [math.sqrt((x3 - x) ** 2 + (y1 - y) ** 2) \
              for x3 in range(min(x1, x2), max(x1, x2) + 1) \
              if not rt.crash(((x, y), (x3, y1)), walls)]
    ds.append(infinity)  # for the case where ds is empty
    return min(ds)


def edist_grid_1(fline,walls):
    """
    This method stores the grid with straight line distance to finish line. This heurstic is used
    when there exists a wall between start point to finish line.

    :param fline:
    :param walls:
    :return: grid
    """
    global grid, g_fline, g_walls, xmax, ymax
    xmax = max([max(x,x1) for ((x,y),(x1,y1)) in walls])
    ymax = max([max(y,y1) for ((x,y),(x1,y1)) in walls])
    grid = [[edistw_to_finish((x,y), fline, walls) for y in range(ymax+1)] for x in range(xmax+1)]

    flag = True
    print('computing edist grid_cross', end=' '); sys.stdout.flush()
    while flag:
        print('.', end=''); sys.stdout.flush()
        flag = False
        for x in range(xmax+1):
            for y in range(ymax+1):
                for y1 in range(max(0,y-1),min(ymax+1,y+2)):
                    for x1 in range(max(0,x-1),min(xmax+1,x+2)):
                        if grid[x1][y1] != infinity and not rt.crash(((x,y),(x1,y1)),walls):
                            if x == x1 or y == y1:
                                d = grid[x1][y1] + 1
                            else:
                                d = grid[x1][y1] + 1.4142135623730951
                            if d < grid[x][y]:
                                grid[x][y] = d
                                flag = True
    print(' done')
    g_fline = fline
    g_walls = walls

    return grid
