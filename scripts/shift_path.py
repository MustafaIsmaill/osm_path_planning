from collections import OrderedDict
import math

from shapely.geometry import Point
from shapely.geometry import LineString

def get_eucl_distance(p1x, p1y, p2x, p2y):
    return math.sqrt( (p2x-p1x)**2 + (p2y-p1y)**2 )

def rm_duplicates(arr_x, arr_y):
    length = len(arr_x) - 1
    idx = 0
    while idx < length:
        if get_eucl_distance(arr_x[idx], arr_y[idx], arr_x[idx+1], arr_y[idx+1]) < 0.5:
            arr_x.remove(arr_x[idx])
            arr_y.remove(arr_y[idx])   
            length -= 1
        idx += 1
    
    return arr_x, arr_y

def parallel_line_shift(start, end, distance):
    (x1,y1) = start
    (x2,y2) = end

    theta = math.atan((x1-x2)/(y1-y2))
    if y2 < y1:
        x1 -= distance*math.cos(theta)
        y1 += distance*math.sin(theta)
        x2 -= distance*math.cos(theta)
        y2 += distance*math.sin(theta)
    else:
        x1 += distance*math.cos(theta)
        y1 -= distance*math.sin(theta)
        x2 += distance*math.cos(theta)
        y2 -= distance*math.sin(theta)
    return ((x1,y1), (x2,y2))

def shift_path(x, y, distance):
    lines = []
    for idx in range(len(x)-1):
        curr_line = LineString( [[x[idx], y[idx]], [x[idx+1], y[idx+1]]] )
        curr_x, curr_y = curr_line.xy
        for c in range(len(curr_x)-1):
            p1, p2 = parallel_line_shift((curr_x[c], curr_y[c]),(curr_x[c+1], curr_y[c+1]), distance)
            l = LineString([p1, p2])
            lines.append(l)
    return lines

def remove_intersects(lines):
    new_x = []
    new_y = []
    for line in range(len(lines)-1):
        x_one, y_one = lines[line].xy
        x_two, y_two = lines[line+1].xy

        inter = lines[line].intersection(lines[line+1])

        if inter:
            lines[line] = LineString([(x_one[0], y_one[0]), (inter.x, inter.y)])
            lines[line+1] = LineString([(inter.x, inter.y), (x_two[len(x_two)-1], y_two[len(y_two)-1])])

    return lines