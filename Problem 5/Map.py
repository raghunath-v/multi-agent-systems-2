from graphics import *

import numpy as np
import math
import time

class Agent_Position :
    def __init__(self):
        self.a_x = 0
        self.a_y = 0
        self.v_x = 0
        self.v_y = 0
        self.x = 0
        self.y = 0

class Map :

    def __init__(self,parameters,traj_vec,win):
        self.parameters = parameters
        self.traj_vec = traj_vec
        self.win = win

        self.agent_positions = [Agent_Position() for x in range(0,self.parameters.nr_agents)]
        for i in range(0,self.parameters.nr_agents) :
            self.agent_positions[i].x = self.parameters.start_positions[i][0]
            self.agent_positions[i].y = self.parameters.start_positions[i][1]

        self.agent_graphics_points = [Circle(Point(0,0),3) for x in range(0,self.parameters.nr_agents + 1)]
        self.draw_agent_positions(0)

        color_vec = ['red','green','blue','yellow','black','cyan','magenta']
        for i in range(0,len(self.agent_graphics_points)) :
            self.agent_graphics_points[i].setFill(color_vec[i])
            self.agent_graphics_points[i].setOutline(color_vec[i])
            self.agent_graphics_points[i].draw(win)

        self.old_errors = []
        for i in range(0,self.parameters.nr_agents) :
            self.old_errors.append([0,0])


    def get_desired_positions(self,iteration):
        x_i = self.traj_vec[iteration][0]
        y_i = self.traj_vec[iteration][1]
        theta_i = self.traj_vec[iteration][2]

        theta_i -= math.pi / 2

        formation_positions_real = []
        for i in range(0,self.parameters.nr_agents) :
            formation_positions_real_i_0 = self.parameters.formation_positions_relative[i][0] * math.cos(theta_i) - self.parameters.formation_positions_relative[i][1] * math.sin(theta_i)
            formation_positions_real_i_1 = self.parameters.formation_positions_relative[i][0] * math.sin(theta_i) + self.parameters.formation_positions_relative[i][1] * math.cos(theta_i)

            formation_positions_real.append([formation_positions_real_i_0,formation_positions_real_i_1])

        for i in range(0,self.parameters.nr_agents) :
            formation_positions_real[i][0] += x_i
            formation_positions_real[i][1] += y_i

        return formation_positions_real


    def update_agent_positions(self,desired_positions):
        alpha = 5.0
        beta = 10.0

        for i in range(0,self.parameters.nr_agents) :
            error_i_x = desired_positions[i][0] - self.agent_positions[i].x
            error_i_y = desired_positions[i][1] - self.agent_positions[i].y

            d_error_i_x = error_i_x - self.old_errors[i][0]
            d_error_i_y = error_i_y - self.old_errors[i][1]

            self.old_errors[i][0] = error_i_x
            self.old_errors[i][1] = error_i_y

            a_i_x = alpha * error_i_x + beta * d_error_i_x
            a_i_y = alpha * error_i_y + beta * d_error_i_y

            a_i = math.sqrt(math.pow(a_i_x,2) + math.pow(a_i_y,2))
            if (a_i > self.parameters.a_max) :
                # sqrt((a * x)^2 + (a * y)^2 ) = b
                # solve for a
                #
                # a^2x^2 + a^2y^2 = b^2
                #
                # a^2 = b^2 / (x^2 + y^2)

                factor = math.sqrt(math.pow(self.parameters.a_max,2) / (math.pow(a_i_x,2) + math.pow(a_i_y,2)))

                a_i_x *= factor
                a_i_y *= factor

                #a_i = math.sqrt(math.pow(a_i_x, 2) + math.pow(a_i_y, 2))

            self.agent_positions[i].a_x = a_i_x
            self.agent_positions[i].a_y = a_i_y

            v_i_x = self.agent_positions[i].v_x + self.agent_positions[i].a_x * self.parameters.dt
            v_i_y = self.agent_positions[i].v_y + self.agent_positions[i].a_y * self.parameters.dt

            v_i = math.sqrt(math.pow(v_i_x,2) + math.pow(v_i_y,2))
            if (v_i > self.parameters.v_max):
                factor = math.sqrt(math.pow(self.parameters.v_max, 2) / (math.pow(v_i_x, 2) + math.pow(v_i_y, 2)))

                v_i_x *= factor
                v_i_y *= factor

            self.agent_positions[i].v_x = v_i_x
            self.agent_positions[i].v_y = v_i_y

            self.agent_positions[i].x += self.agent_positions[i].v_x * self.parameters.dt
            self.agent_positions[i].y += self.agent_positions[i].v_y * self.parameters.dt


    def draw_agent_positions(self,iteration):
        a = 2

        for i in range(0,len(self.agent_graphics_points)) :
            old_x_i = self.agent_graphics_points[i].getCenter().x
            old_y_i = self.agent_graphics_points[i].getCenter().y

            if (i < self.parameters.nr_agents) :
                dx_i = self.agent_positions[i].x * self.parameters.graphics_scale + self.parameters.graphics_add - old_x_i
                dy_i = self.agent_positions[i].y * self.parameters.graphics_scale + self.parameters.graphics_add- old_y_i
            else :
                dx_i = self.traj_vec[iteration][0] * self.parameters.graphics_scale + self.parameters.graphics_add - old_x_i
                dy_i = self.traj_vec[iteration][1] * self.parameters.graphics_scale + self.parameters.graphics_add - old_y_i

            self.agent_graphics_points[i].move(dx_i,dy_i)



    def solve_problem(self) :
        traj_len = len(self.traj_vec)

        for i in range(0,traj_len) :
            desired_positions = self.get_desired_positions(i)
            self.update_agent_positions(desired_positions)
            self.draw_agent_positions(i)

            time.sleep(0.005)

