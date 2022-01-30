from re import T
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import math




class Simulation:
    def __init__(self):
        pi = np.pi

        L = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        theta = [0, 0, 0, 0, 0, 0]
        dof = len(theta)
    
        arpha_i1 = [0, pi/2, 0, pi/2, -pi/2, pi/2]
        a_i1     = [0, 0, L[1], 0, 0, 0]
        d_i      = [L[0], 0, 0, L[2]+L[3], 0, L[4]+L[5]]
        theta_i  = [theta[0], 3*pi/4+theta[1], -pi/4+theta[2], theta[3], -pi/2+theta[4], theta[5]]


        self.dof = dof

        self.arpha_i1 = arpha_i1
        self.a_i1 = a_i1
        self.d_i = d_i
        self.theta_i = theta_i


    # DHパラメータ
    def dh_parameter(self):
        pi = np.pi
        dof = self.dof
        arpha_i1 = self.arpha_i1
        a_i1 = self.a_i1
        d_i = self.d_i
        theta_i = self.theta_i

        dh = np.zeros((6, 4))

        for i in range(dof):
            dh[i] = np.array([
                arpha_i1[i], a_i1[i], d_i[i], theta_i[i]
            ])
            
            #print(dh[i])

        #print(dh)




        return dh

    def Screw_x(a,arpha):
        Screw_x = np.array([

            [1,             0,              0, a],
            [0, np.cos(arpha), -np.sin(arpha), 0],
            [0, np.cos(arpha), np.cos(arpha),  0],
            [0,             0,              0, 1]

        ])
        return Screw_x

    def Screw_z(d,theta):
        Screw_z = np.array([

            [0, np.cos(theta), -np.sin(theta), 0],
            [0, np.cos(theta),  np.cos(theta), 0],
            [0,             0,              1, d],
            [0,             0,              0, 1]

        ])

        return Screw_z

    def doujihenkan_gyouretu(dh_arpha, dh_a, dh_d, dh_theta, L, theta):

        T = []
    
        for i in range(len(L)):
            T = np.dot( Screw_x(dh_a[i], dh_arpha[i]), Screw_z(dh_d[0], dh_theta[0]) )
            np.append(T[i], T)

        T = np.dot()

    def gyouretu_T(self):
        
        arpha_i1 = self.arpha_i1
        a_i1 = self.a_i1
        d_i = self.d_i
        theta_i = self.theta_i

        T = np.zeros((4,4))

        for i in range(4):
            T = np.array([

                [np.cos(theta_i[i]), -np.sin(theta_i[i]), 0, a_i1[i]],
                [np.sin(theta_i[i])*np.cos(arpha_i1[i]), np.cos(theta_i[i])*np.cos(arpha_i1[i]), -np.sin(arpha_i1[i]),  -np.sin(arpha_i1[i])*d_i[i]],
                [np.sin(theta_i[i])*np.sin(arpha_i1[i]), np.cos(theta_i[i])*np.sin(arpha_i1[i]),  np.cos(arpha_i1[i]),   np.cos(arpha_i1[i])*d_i[i]],
                [0, 0, 0, 1]

            ])

        print(T)

        return T

   


    def main(self):
        self.dh_parameter()

        self.gyouretu_T()
        
        

        # リンク1, 2の長さ
        



    # 順運動学の計算
    #dh = dh_parameter(L, theta)
    #print(dh)



if __name__ == '__main__':
    sim = Simulation()
    sim.main()