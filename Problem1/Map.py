from graphics import *
import matplotlib.colors as colors
import time

import numpy as np
from miscFunctions import get_N_HexCol, intersect, if_between, distance, reached
import math
import time
from Agent import Agent


class Map:

    def __init__(self,parameters,win):
        self.parameters = parameters
        self.win = win

        self.agents = [Agent(x, [self.parameters.goal_positions[x][0], self.parameters.goal_positions[x][1]])
                       for x in range(0,self.parameters.nr_agents)]
        for i in range(0,self.parameters.nr_agents) :
            self.agents[i].x = self.parameters.start_positions[i][0]
            self.agents[i].y = self.parameters.start_positions[i][1]

        self.agent_graphics_points = [Circle(Point(0,0),3) for x in range(0,self.parameters.nr_agents)]
        self.draw_agent_positions()

        # color_vec = ['red','green','blue','yellow','black','cyan','magenta']
        color_vec = get_N_HexCol(self.parameters.nr_agents)
        step = len(colors.cnames)
        for i in range(0,len(self.agent_graphics_points)) :
            # val = math.ceil(255/(i+1))
            # color = color_rgb(val, val, val)
            self.agent_graphics_points[i].setFill(color_vec[i])
            self.agent_graphics_points[i].setOutline(color_vec[i])
            self.agent_graphics_points[i].draw(win)

        # Game variables
        self.allDone = False
        self.vision = 2
        self.discrete_div = 10
        self.total_time = 0
        self.finished_agents = 0

    def nearby_agents(self, agent):
        count = 0
        for i in range(0, self.parameters.nr_agents):
            dist = agent.dist_to(self.agents[i])
            if dist < self.vision:
                count += 1
        return count

    def get_scaled_velocity(self, agent):
        nearby_agents = self.nearby_agents(agent)
        vel = self.parameters.v_max/math.sqrt(2**(nearby_agents/2))
        # TODO HARD CODED Forced vel to vmax
        vel = self.parameters.v_max
        self.get_goal_direction(agent)
        agent.v_x = agent.dir_x * vel
        agent.v_y = agent.dir_y * vel


    def get_goal_direction(self, agent):
        dir_x = self.parameters.goal_positions[agent.id][0]-agent.x
        dir_y = self.parameters.goal_positions[agent.id][1]-agent.y
        scale = math.sqrt(dir_x**2+dir_y**2)
        agent.dir_x = dir_x / scale
        agent.dir_y = dir_y / scale

    def get_new_direction(self, agent):
        dir_x = self.parameters.goal_positions[agent.id][0] - agent.x
        dir_y = self.parameters.goal_positions[agent.id][1] - agent.y
        scale = math.sqrt(dir_x ** 2 + dir_y ** 2)
        agent.dir_x = dir_x / scale
        agent.dir_y = dir_y / scale

    def get_velocities(self):
        for i in range(0, self.parameters.nr_agents):
            if self.agents[i].goal_reached is False:
                self.get_scaled_velocity(self.agents[i])
            else:
                self.agents[i].v_x = 0
                self.agents[i].v_y = 0

    def update_agent_positions(self):
        self.finished_agents = 0
        for i in range(0,self.parameters.nr_agents):
            self.agents[i].x += self.agents[i].v_x * self.parameters.dt
            self.agents[i].y += self.agents[i].v_y * self.parameters.dt
            if reached(self.agents[i]):
                self.agents[i].goal_reached = True
                self.finished_agents += 1
        if self.finished_agents == self.parameters.nr_agents:
            self.allDone = True
        self.total_time = self.total_time + self.parameters.dt

    def draw_agent_positions(self):
        for i in range(0,len(self.agent_graphics_points)):
            old_x_i = self.agent_graphics_points[i].getCenter().x
            old_y_i = self.agent_graphics_points[i].getCenter().y

            dx_i = self.agents[i].x * self.parameters.graphics_scale + self.parameters.graphics_add - old_x_i
            dy_i = self.agents[i].y * self.parameters.graphics_scale + self.parameters.graphics_add - old_y_i

            self.agent_graphics_points[i].move(dx_i,dy_i)

    def get_best_velocity(self,agent):
        velocities = agent.discrete_velocities(self.discrete_div)
        for i in range(0, self.parameters.nr_agents):
            if i is not agent.id:
                for vel in velocities:
                    pass
                    #if vel1 and vel2 collide
                    #note time and vel
                    #if time is max
                    #opt_vel
        pass

    def RVO_update(self):
        for i in range(self.parameters.nr_agents):
            agentA = self.agents[i]
            RVO_all = []
            if agentA.goal_reached is False:
                for j in range(self.parameters.nr_agents):
                    if i is not j:
                        agentB = self.agents[j]
                        transl_vB_vA = [agentA.x + 0.5*(agentB.v_x + agentA.v_x),
                                        agentA.y + 0.5*(agentB.v_y + agentA.v_y)]
                        dist = agentA.dist_to(agentB)
                        min_dist = agentA.radius+agentB.radius
                        theta = agentA.angle_to(agentB)
                        if dist < min_dist:
                            dist = min_dist
                        theta_ort = math.asin(min_dist/dist)
                        theta_ort_left = theta + theta_ort
                        bound_left = [math.cos(theta_ort_left), math.sin(theta_ort_left)]
                        theta_ort_right = theta - theta_ort
                        bound_right = [math.cos(theta_ort_right), math.sin(theta_ort_right)]
                        RVO = [transl_vB_vA, bound_left, bound_right, dist, min_dist]
                        RVO_all.append(RVO)
                new_vel = intersect(agentA, RVO_all)
                self.agents[i].v_x = new_vel[0]
                self.agents[i].v_y = new_vel[1]



    def solve_problem(self) :

        while self.allDone is not True:
            start = time.clock()
            self.get_velocities()
            #print("Time to get Velocities :", time.clock() - start)

            start = time.clock()
            self.RVO_update()
            #print("Time to update RVO :", time.clock() - start)

            self.update_agent_positions()
            self.draw_agent_positions()


            #time.sleep(0.0001)

        print("All agents have reached their goals")
        print("Total time taken is : ", self.total_time)

