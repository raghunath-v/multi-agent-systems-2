from graphics import *

import math
import time
import numpy as np

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

        self.agent_form_positions = [Agent_Position() for x in range(0, self.parameters.nr_agents)]
        for i in range(0, self.parameters.nr_agents):
            self.agent_positions[i].x = self.parameters.start_positions[i][0]
            self.agent_positions[i].y = self.parameters.start_positions[i][1]

        self.agent_graphics_points = [Circle(Point(0,0),3) for x in range(0,self.parameters.nr_agents + 1)]
        self.agent_graphics_points[10] = Circle(Point(0,0),5) #Text(Point(0,0), "R")

        self.draw_agent_positions(0)

        color_vec = ['green','blue','yellow','red','cyan','black','white','brown','pink','violet','magenta']
        for i in range(0,len(self.agent_graphics_points)-1) :
            self.agent_graphics_points[i].setFill(color_vec[i])
            self.agent_graphics_points[i].setOutline(color_vec[i])
            self.agent_graphics_points[i].draw(win)
        #self.agent_graphics_points[0].setFill(color_vec[0])
        self.agent_graphics_points[10].setOutline(color_vec[10])
        self.agent_graphics_points[10].draw(win)

        self.old_errors = []
        for i in range(0,self.parameters.nr_agents) :
            self.old_errors.append([0,0])



    def get_formation_delta(self,agent):
        return [[self.parameters.formation_min_x - self.parameters.formation_positions[agent][0],
                 self.parameters.formation_min_y - self.parameters.formation_positions[agent][1]],
                [self.parameters.formation_max_x - self.parameters.formation_positions[agent][0],
                 self.parameters.formation_max_y - self.parameters.formation_positions[agent][1]]]

    def formation_is_in_pitch(self,agent,iteration):
        formation_delta = self.get_formation_delta(agent)

        position_ronaldo = self.traj_vec[iteration]

        if position_ronaldo[0] + formation_delta[0][0] < self.parameters.pitch_min_x:
            return False
        if position_ronaldo[1] + formation_delta[0][1] < self.parameters.pitch_min_y:
            return False
        if position_ronaldo[0] + formation_delta[1][0] > self.parameters.pitch_max_x:
            return False
        if position_ronaldo[1] + formation_delta[1][1] > self.parameters.pitch_max_y:
            return False

        return True


    def get_distance_between(self,p1,p2):
        return math.sqrt(math.pow(p1[0] - p2[0],2) + math.pow(p1[1] - p2[1],2))

    def get_cover_agent(self,iteration):
        distances_to_ronaldo = []

        for i in range(0,self.parameters.nr_agents) :
            #distances_to_ronaldo.append(self.get_distance_between([self.agent_positions[i].x,self.agent_positions[i].y],
            #                                                     [self.traj_vec[iteration][0],self.traj_vec[iteration][1]]))
            distances_to_ronaldo.append(self.get_distance_between([self.agent_form_positions[i].x,self.agent_form_positions[i].y],
                                                                  [self.traj_vec[iteration][0],self.traj_vec[iteration][1]]))

        distances_to_ronaldo_sort = np.argsort(distances_to_ronaldo)

        for i in distances_to_ronaldo_sort :
            if self.formation_is_in_pitch(i,iteration) :
                return i

        return 0


    def get_desired_positions(self,cover_agent,iteration):

        formation_positions_real = []
        for i in range(0,self.parameters.nr_agents) :
            cover_agent_i_delta_x = self.parameters.formation_positions[i][0] - self.parameters.formation_positions[cover_agent][0]
            cover_agent_i_delta_y = self.parameters.formation_positions[i][1] - self.parameters.formation_positions[cover_agent][1]

            formation_positions_real_i_0 = self.agent_positions[cover_agent].x + cover_agent_i_delta_x
            formation_positions_real_i_1 = self.agent_positions[cover_agent].y + cover_agent_i_delta_y

            if (i == cover_agent) :
                formation_positions_real_i_0 = self.traj_vec[iteration][0]
                formation_positions_real_i_1 = self.traj_vec[iteration][1]

            formation_positions_real.append([formation_positions_real_i_0,formation_positions_real_i_1])

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
            cover_agent = self.get_cover_agent(i)
            desired_positions = self.get_desired_positions(cover_agent,i)
            self.update_agent_positions(desired_positions)
            self.draw_agent_positions(i)

            time.sleep(0.005)

