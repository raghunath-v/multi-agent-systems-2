from Map import Map
from Parameters import Parameters

from graphics import *

import json
import random

def main_function() :
    random.seed(0)

    N = 36

    with open("P21_X.json") as json_file:
        desc = json.load(json_file)

    bounding_poly_positions = desc['bounding_polygon']
    obstacles = [desc[key] for key, val in desc.items() if key.startswith('obs')]
    orig_start_positions = desc['start_positions']
    start_positions = orig_start_positions[:N]
    orig_goal_positions = desc['goal_positions']
    goal_positions = orig_goal_positions[:N]
    dt = desc['vehicle_dt']
    v_max = desc['vehicle_v_max']
    a_max = desc['vehicle_a_max']

    #with open("P25_26_traj.json") as json_file:
    #    desc = json.load(json_file)

    #traj_vec = list(zip(desc['x'],desc['y'],desc['theta']))

    graphics_scale = 10
    graphics_add = 50

    win = GraphWin("map",400,500)
    win.yUp()

    parameters = Parameters(bounding_poly_positions,obstacles,start_positions,goal_positions,dt,v_max,a_max,graphics_scale,graphics_add)

    parameters.draw_obstacles(win)

    map = Map(parameters, win)

    map.solve_problem()

    win.getMouse()
    win.close()





main_function()