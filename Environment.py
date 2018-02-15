import json
import sys
#from Node import Node
from Goal import Goal
from graphics import GraphWin
from KinematicPoint import KinematicPoint
from Obstacle import Obstacle
from RRT import RRT
from BoundingArea import BoundingArea


class Environment:
    '''
    Environment represents takes care of running the
    models, generating obstacles and visualizing it.
    '''

    def __init__(self, obs, bounding, players, goals, win, quick_draw=False):
        '''
        m_type : [0,1,2,3] = [the models available]
        obs = list of lists of [x,y], each representing an obstacle
        bounding = list of [x,y] defining the bounding area
        '''
        self.win = win
        self.win.yUp()
        self.obstacles = [Obstacle(o, self.win) for o in obs]
        self.bounding_area = BoundingArea(bounding, self.win)
        self.players = players
        self.goals = goals
        #if len(players) != len(goals):
        #    raise ValueError('Number of Players and Goals are not same')
        self.rrt = None
        self.quick_draw = quick_draw



    def gen_rrt(self, rrt_setup):
        self.rrt = RRT(self.bounding_area, self.obstacles, self.player, self.goal,
                       rrt_setup, self.win, goal_rate=10)
        self.rrt.generate()
        self.player.add_path(self.rrt.optimal_path)
        self.rrt.set_graphicals(self.quick_draw)

    def simulate(self, rrt_setup, player, goal, idx):
        self.gen_rrt(rrt_setup)
        player.set_velocity(goal)
        player.set_graphicals()
        #self.win.getMouse()
        #self.win.close()
        #return player.total_time

    def run(self, rrt_setup):
        for idx in range(len(self.players))
            self.init_draw()
            self.gen_rrt(rrt_setup)
                if not players[idx].finished:
                    self.simulate(rrt_setup, self.players[idx], self.goals[idx], idx)
            self.player.set_graphicals()
            print("Is finshed")
            print("Position:")
            print("goal: ", self.goal.pos_x, ",", self.goal.pos_y)
            print("player: ", self.player.pos_x, ",", self.player.pos_y)
            print("Velocity:")
            print("goal: ", self.goal.vel_x, ",", self.goal.vel_y)
            if isinstance(self.player, DynamicPoint):
                print("player: ", self.player.current_vel[0], ",", self.player.current_vel[1])
            else:
                print("player: ", self.player.vel_x, ",", self.player.vel_y)
            print("Time taken to reach goal (sec): ", player.total_time)
            # comment line below to record time
            self.win.getMouse()
            self.win.close()
        return player.total_time



    def init_draw(self):
        # draw everything initially
        self.bounding_area.set_graphicals()
        for obs in self.obstacles:
            obs.set_graphicals()
        self.player.set_graphicals()
        self.goal.set_graphicals()


if __name__ == "__main__":

    # Parse problem setup
    with open("Problem1/" + str(env_name) + ".json") as json_file:
        desc = json.load(json_file)
    bounding_poly = desc['bounding_polygon']
    obstacles = [desc[key] for key, val in desc.items() if key.startswith('obs')]
    pos_start = desc['start_positions'][:0]
    pos_goal = desc['goal_positions'][:0]
    dt = desc['vehicle_dt']
    v_max = desc['vehicle_v_max']
    a_max = desc['vehicle_a_max']
    omega_max = desc['vehicle_omega_max']
    phi_max = desc['vehicle_phi_max']
    vehicle_length = desc['vehicle_L']

    # Parse problem strategy
    with open("Problem1/current_strategy.json") as json_file:
        curr_strat = json.load(json_file)
    env_name = curr_strat['env_name']
    mdl_name = curr_strat['mdl_name']
    delta_q = curr_strat['delta_q']
    k = curr_strat['k']
    rrt_strat = curr_strat['rrt_strategy']

    # Area setup for visualisation
    canvas_width = 800
    canvas_height = 800
    win = GraphWin("area", canvas_width, canvas_height)

    # Setup Initial Conditions
    players = []
    goals = []
    if mdl_name == 'kinematic_point':
        for pos in pos_start:
            players.append(KinematicPoint(pos, dt, v_max, win))
    else:
        print('Invalid model name, exiting')
        sys.exit(0)
    for pos in pos_goal:
        goal = Goal(pos, win)

    # Generate paths and run environment
    env = Environment(obstacles, bounding_poly, players, goals, win, quick_draw=True)
    rrt_setup = {'delta_q': delta_q, 'k': k, 'strategy': rrt_strat, 'x_range': [-2, 60], 'y_range': [-2, 60]}
    new_time = env.run(rrt_setup)
    win.getMouse()
    win.close()
