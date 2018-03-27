import math as math
import numpy as np
import colorsys
from graphics import color_rgb


def get_N_HexCol(N=5):

    HSV_tuples = [(x*1.0/N, 1, 1) for x in range(N)]
    colors = []
    for rgb in HSV_tuples:
        rgb = colorsys.hsv_to_rgb(*rgb)
        colors.append(color_rgb(math.ceil(255*rgb[0]), math.ceil(255*rgb[1]), math.ceil(255*rgb[2])))
    return colors


def if_between(theta_right, theta, theta_left):
    if abs(theta_right - theta_left) <= math.pi:
        if theta_right <= theta <= theta_left:
            return True
        else:
            return False
    else:
        if (theta_left < 0) and (theta_right > 0):
            theta_left += 2 * math.pi
            if theta < 0:
                theta += 2 * math.pi
            if theta_right <= theta <= theta_left:
                return True
            else:
                return False
        if (theta_left > 0) and (theta_right < 0):
            theta_right += 2 * math.pi
            if theta < 0:
                theta += 2 * math.pi
            if theta_left <= theta <= theta_right:
                return True
            else:
                return False


def distance(u, v):
    return math.sqrt((u[0]-v[0])**2 + (u[1]-v[1])**2)


def intersect(agent, RVO_all):
    vel_mag = math.sqrt(agent.v_x**2 + agent.v_y**2)
    suitable_V = []
    unsuitable_V = []
    for theta in np.arange(0, 2 * math.pi, 0.1):
        for rad in np.arange(0.02, vel_mag + 0.02, vel_mag / 5.0):
            new_v = [rad * math.cos(theta), rad * math.sin(theta)]
            suit = True
            for RVO in RVO_all:
                p_0 = RVO[0]
                left = RVO[1]
                right = RVO[2]
                dif = [new_v[0] + agent.x - p_0[0], new_v[1] + agent.y - p_0[1]]
                theta_dif = math.atan2(dif[1], dif[0])
                theta_right = math.atan2(right[1], right[0])
                theta_left = math.atan2(left[1], left[0])
                if if_between(theta_right, theta_dif, theta_left):
                    suit = False
                    break
            if suit is True:
                suitable_V.append(new_v)
            else:
                unsuitable_V.append(new_v)
    new_v = [agent.v_x, agent.v_y]
    suit = True
    for RVO in RVO_all:
        p_0 = RVO[0]
        left = RVO[1]
        right = RVO[2]
        dif = [new_v[0] + agent.x - p_0[0], new_v[1] + agent.y - p_0[1]]
        theta_dif = math.atan2(dif[1], dif[0])
        theta_right = math.atan2(right[1], right[0])
        theta_left = math.atan2(left[1], left[0])
        if if_between(theta_right, theta_dif, theta_left):
            suit = False
            break
    if suit:
        suitable_V.append(new_v)
    else:
        unsuitable_V.append(new_v)

    if suitable_V:
        # print 'Suitable found'
        vA_post = min(suitable_V, key=lambda v: distance(v, [agent.v_x, agent.v_y]))
        new_v = vA_post[:]
        for RVO in RVO_all:
            p_0 = RVO[0]
            left = RVO[1]
            right = RVO[2]
            dif = [new_v[0] + agent.x - p_0[0], new_v[1] + agent.y - p_0[1]]
            theta_dif = math.atan2(dif[1], dif[0])
            theta_right = math.atan2(right[1], right[0])
            theta_left = math.atan2(left[1], left[0])
    else:
        # print 'Suitable not found'
        tc_V = dict()
        for unsuit_v in unsuitable_V:
            tc_V[tuple(unsuit_v)] = 0
            tc = []
            for RVO in RVO_all:
                p_0 = RVO[0]
                left = RVO[1]
                right = RVO[2]
                dist = RVO[3]
                rad = RVO[4]
                dif = [unsuit_v[0] + agent.x - p_0[0], unsuit_v[1] + agent.y - p_0[1]]
                theta_dif = math.atan2(dif[1], dif[0])
                theta_right = math.atan2(right[1], right[0])
                theta_left = math.atan2(left[1], left[0])
                if if_between(theta_right, theta_dif, theta_left):
                    small_theta = abs(theta_dif - 0.5 * (theta_left + theta_right))
                    if abs(dist * math.sin(small_theta)) >= rad:
                        rad = abs(dist * math.sin(small_theta))
                    big_theta = math.asin(abs(dist * math.sin(small_theta)) / rad)
                    dist_tg = abs(dist * math.cos(small_theta)) - abs(rad * math.cos(big_theta))
                    if dist_tg < 0:
                        dist_tg = 0
                    tc_v = dist_tg / distance(dif, [0, 0])
                    tc.append(tc_v)
            tc_V[tuple(unsuit_v)] = min(tc) + 0.001
        WT = 0.2
        vA_post = min(unsuitable_V, key=lambda v: ((WT / tc_V[tuple(v)]) + distance(v, [agent.v_x, agent.v_y])))
    return vA_post


def reached(agent, slack=0.5):
    if agent.dist_to_goal() < slack:
        return True
    else:
        return False
