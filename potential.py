import numpy as np
import math


class Potential:
    def __init__(self):

        a = 1
        self.nazo = 0.02
        self.omomi_K = 500
        self.shogaibutu_kyoyou_l = 0.02
        self.target_omomi = 800

    #@staticmethod


        





    def v_obstacle(self, p, p_obstacles):
        v = 0.0
        K = self.omomi_K
        l = self.shogaibutu_kyoyou_l

        for p_obs in p_obstacles:
            d = (p - p_obs.pos)
            r = np.sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2])
            r = r-l

            # if r > 0.4:
            #     continue
            # else:
            #     #v += 1. / (0.0002 * r * r + 0.01)
            #v += 2.5 / (1. * r * r + 0.02) 3d

            v += 2.5 / (K * r * r + self.nazo)
            #v += 2.0 / (500. * r * r + 0.02)
        return v
        

    #@staticmethod
    def v_target(self, p, p_target):
        T = self.target_omomi
        if len(p_target) == 0:
            return 0.0

        d = (p - p_target)
        r = np.sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2])

        # return r * 1000.
        #return np.exp(r) * 500.
        return np.exp(r) * T

    def grad_v_target(self, dp, p, p_target):
        dx = np.array([dp, 0.0, 0.0])
        dy = np.array([0.0, dp, 0.0])
        dz = np.array([0.0, 0.0, dp])

        gvx = (self.v_target(p + dx, p_target) - self.v_target(p - dx, p_target)) * 0.5 / dp
        gvy = (self.v_target(p + dy, p_target) - self.v_target(p - dy, p_target)) * 0.5 / dp
        gvz = (self.v_target(p + dz, p_target) - self.v_target(p - dz, p_target)) * 0.5 / dp

        return np.array([gvx, gvy, gvz])

    def grad_v_obstacle(self, dp, p, p_obstacles):
        dx = np.array([dp, 0.0, 0.0])
        dy = np.array([0.0, dp, 0.0])
        dz = np.array([0.0, 0.0, dp])

        gvx = (self.v_obstacle(p + dx, p_obstacles) - self.v_obstacle(p - dx, p_obstacles)) * 0.5 / dp
        gvy = (self.v_obstacle(p + dy, p_obstacles) - self.v_obstacle(p - dy, p_obstacles)) * 0.5 / dp
        gvz = (self.v_obstacle(p + dz, p_obstacles) - self.v_obstacle(p - dz, p_obstacles)) * 0.5 / dp

        return np.array([gvx, gvy, gvz])







