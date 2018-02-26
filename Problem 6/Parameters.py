from graphics import *

import numpy as np

class Obstacle :

    def __init__(self, bounding_poly):
        self.bounding_poly = bounding_poly
        self.nr_corners = len(bounding_poly)

    def draw_obstacle(self,win,graphics_scale,grapchis_add):
        j = self.nr_corners - 1
        for i in range(0,self.nr_corners) :
            if (i > 0): j = i -1

            p1 = Point(self.bounding_poly[j][0] * graphics_scale + grapchis_add,self.bounding_poly[j][1] * graphics_scale + grapchis_add)
            p2 = Point(self.bounding_poly[i][0] * graphics_scale + grapchis_add, self.bounding_poly[i][1] * graphics_scale + grapchis_add)

            line = Line(p1,p2)
            line.setFill('red')
            line.draw(win)

    def is_in_bounding_poly(self,point):
        c = False;

        j = self.nr_corners - 1
        for i in range(0,self.nr_corners) :
            if (i > 0) : j = i - 1

            if (((self.bounding_poly[i][1] >= point[1]) != (self.bounding_poly[j][1] >= point[1])) and
                (point[0] <= (self.bounding_poly[j][0] - self.bounding_poly[i][0]) * (point[1] - self.bounding_poly[i][1]) / (self.bounding_poly[j][1] - self.bounding_poly[i][1]) +
                 self.bounding_poly[i][0])) :

                    c =  not(c)

        return c


class Parameters :

    def __init__(self,
                 bounding_poly_positions,
                 formation_positions,
                 start_positions,
                 dt,
                 v_max,
                 a_max,
                 graphics_scale,
                 graphics_add):

        self.bounding_poly_positions = bounding_poly_positions
        self.formation_positions = formation_positions
        self.start_positions = start_positions
        self.dt = dt
        self.v_max = v_max
        self.a_max = a_max

        self.graphics_scale = graphics_scale
        self.graphics_add = graphics_add

        self.nr_agents = len(start_positions)

        self.bounding_poly = Obstacle(bounding_poly_positions)

        self.formation_max_x = max(np.array(formation_positions)[:,0])
        self.formation_min_x = min(np.array(formation_positions)[:,0])
        self.formation_max_y = max(np.array(formation_positions)[:,1])
        self.formation_min_y = min(np.array(formation_positions)[:,1])

        self.pitch_max_x = max(np.array(bounding_poly_positions)[:,0])
        self.pitch_min_x = min(np.array(bounding_poly_positions)[:,0])
        self.pitch_max_y = max(np.array(bounding_poly_positions)[:,1])
        self.pitch_min_y = min(np.array(bounding_poly_positions)[:,1])

    def draw_obstacles(self,win):
        self.bounding_poly.draw_obstacle(win,self.graphics_scale,self.graphics_add)


    def is_in_obstacle(self,point):
        if not(self.bounding_poly.is_in_bounding_poly(point)) :
            return True

        return False