from graphics import *
import math as math
from g_tools import gen_point_list,scale_points
from Agent import Agent


class Bounding:

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
        c = False

        j = self.nr_corners - 1
        for i in range(0,self.nr_corners) :
            if (i > 0) : j = i - 1

            if (((self.bounding_poly[i][1] >= point[1]) != (self.bounding_poly[j][1] >= point[1])) and
                (point[0] <= (self.bounding_poly[j][0] - self.bounding_poly[i][0]) * (point[1] - self.bounding_poly[i][1]) / (self.bounding_poly[j][1] - self.bounding_poly[i][1]) +
                 self.bounding_poly[i][0])):
                    c = not c

        return c

class Obstacle:
    """
        An obstacle has both a graphical and a
        numerical representation
    """
    def __init__(self, bounding_poly):
        self.bounding_poly = bounding_poly
        self.nr_corners = len(bounding_poly)
        # Get dummy agent
        centroid = [0, 0]
        for i in range(0, self.nr_corners):
            centroid = [centroid[0] + self.bounding_poly[i][0], centroid[1] + self.bounding_poly[i][1]]
        centroid = [centroid[0] / self.nr_corners, centroid[1] / self.nr_corners]
        max_rad = 0
        for i in range(0, self.nr_corners):
            dist = math.sqrt((centroid[0] - self.bounding_poly[i][0]) ** 2 +
                             (centroid[1] - self.bounding_poly[i][1]) ** 2)
            if dist > max_rad:
                max_rad = dist
        self.dummy_agent = Agent(1000, centroid, max_rad)

    def draw_obstacle(self,win,graphics_scale,grapchis_add):
        j = self.nr_corners - 1
        for i in range(0,self.nr_corners) :
            if (i > 0): j = i -1

            p1 = Point(self.bounding_poly[j][0] * graphics_scale + grapchis_add,self.bounding_poly[j][1] * graphics_scale + grapchis_add)
            p2 = Point(self.bounding_poly[i][0] * graphics_scale + grapchis_add, self.bounding_poly[i][1] * graphics_scale + grapchis_add)

            line = Line(p1,p2)
            line.setFill('blue')
            line.draw(win)



class Parameters:

    def __init__(self,
                 bounding_poly_positions,
                 obstacles,
                 start_positions,
                 goal_positions,
                 dt,
                 v_max,
                 a_max,
                 graphics_scale,
                 graphics_add):

        self.bounding_poly_positions = bounding_poly_positions
        self.goal_positions = goal_positions
        self.start_positions = start_positions
        self.dt = dt
        self.v_max = v_max
        self.a_max = a_max

        self.graphics_scale = graphics_scale
        self.graphics_add = graphics_add

        self.nr_agents = len(start_positions)

        self.bounding_poly = Bounding(bounding_poly_positions)
        self.obstacles = [Obstacle(obs) for obs in obstacles]


    def draw_obstacles(self, win):
        self.bounding_poly.draw_obstacle(win,self.graphics_scale,self.graphics_add)
        for i in range(len(self.obstacles)):
            self.obstacles[i].draw_obstacle(win,self.graphics_scale,self.graphics_add)


    def is_in_obstacle(self,point):
        if not(self.bounding_poly.is_in_bounding_poly(point)) :
            return True

        return False