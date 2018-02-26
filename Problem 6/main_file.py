from Map import Map
from Parameters import Parameters

from graphics import *

import json
import random

def main_function() :
    random.seed(0)

    with open("P26.json") as json_file:
        desc = json.load(json_file)

    bounding_poly_positions = desc['bounding_polygon']
    formation_positions = desc['formation_positions']
    start_positions = desc['start_positions']
    dt = desc['vehicle_dt']
    v_max = desc['vehicle_v_max']
    a_max = desc['vehicle_a_max']

    with open("P25_26_traj.json") as json_file:
        desc = json.load(json_file)

    traj_vec = list(zip(desc['x'],desc['y'],desc['theta']))

    graphics_scale = 5
    graphics_add = 20

    win = GraphWin("map",400,700)

    parameters = Parameters(bounding_poly_positions,formation_positions,start_positions,dt,v_max,a_max,graphics_scale,graphics_add)

    parameters.draw_obstacles(win)

    map = Map(parameters,traj_vec,win)

    map.solve_problem()

    win.getMouse()
    win.close()





main_function()