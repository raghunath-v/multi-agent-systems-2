import math as math

class Agent :
    def __init__(self, identity, goal):
        self.id = identity
        self.a_x = 0
        self.a_y = 0
        self.v_x = 0
        self.v_y = 0
        self.x = 0
        self.y = 0
        self.dir_x = 0
        self.dir_y = 0
        self.allowed_velocities = []
        self.radius = 3
        self.goal_x = goal[0]
        self.goal_y = goal[1]
        self.goal_reached = False

    def dist_to(self, anotherAgent):
        dist = math.sqrt((self.x-anotherAgent.x)**2 + (self.y-anotherAgent.y)**2)
        return dist

    def angle_to(self, anotherAgent):
        angle = math.atan2(anotherAgent.y-self.y, anotherAgent.x-self.x)
        return angle

    def stop(self):
        self.v_x = 0
        self.v_y = 0

    def discrete_velocities(self, div):
        theta = 2 * math.pi / div
        velocities = []
        for i in range(div):
            new_vx = self.v_x * math.cos(i * theta) - self.v_x * math.sin(i * theta)
            new_vy = self.v_y * math.sin(i * theta) + self.v_y * math.cos(i * theta)
            velocities.append([new_vx, new_vy, 1])
        self.allowed_velocities = velocities

    def discrete_velocities_symmetric(self, div):
        theta = math.pi / div
        velocities = []
        for i in range(div):
            new_vx = self.v_x * math.cos(i * theta) - self.v_x * math.sin(i * theta)
            new_vy = self.v_y * math.sin(i * theta) + self.v_y * math.cos(i * theta)
            velocities.append([new_vx, new_vy, 1])
            new_vx = self.v_x * math.cos(-i * theta) - self.v_x * math.sin(-i * theta)
            new_vy = self.v_y * math.sin(-i * theta) + self.v_y * math.cos(-i * theta)
            velocities.append([new_vx, new_vy, 1])
        self.allowed_velocities = velocities

    def dist_to_goal(self):
        dist = math.sqrt((self.x - self.goal_x) ** 2 + (self.y - self.goal_y) ** 2)
        return dist
