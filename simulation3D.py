# -*- coding: UTF-8 -*-
from ntpath import join
from os import path

from numpy import r_
from camera import Camera
from arm3D import Arm2
from object import Object
import numpy as np
import matplotlib.pyplot as plt
import json
import csv
import copy
from decide_obstacle_position import obstacle_pos
from potential import Potential



class Simulation:
    def __init__(self):

        # 初期setting.jsonを使う場合
        #json_file = open('setting.json', 'r')

        #json_file = open('setting_jisaku6def_arm.json', 'r')
        json_file = open('setting_kaiten_gosa_arm.json', 'r')

        
        #json_file = open('setting_shifted_jisaku6dof_arm.json', 'r')

        #json_file = open('setting_7def.json', 'r')


        setting_json = json.load(json_file)

        self.world_size = [-.5, -.5, 0.0, .5, .5, .5]  # [x_min, y_min, z_min, x_max, y_max, z_max]
        # self.arm = Arm()
        self.arm = Arm2(link_info=setting_json["link_info"], interaction_info=setting_json["interaction_info_ur"])
        self.cam = Camera()
        self.rotation, self.translation = self.cam.generate_camera_pose()
        self.target = Object(self.world_size, .0)
        self.potential = Potential()

        # 初期位置を最初の目標にする
        self.target.pos[:] = [0.187172718, 0.0, 0.2121513]
        # self.fig, self.ax1, self.ax2, self.ax3 = self.plot_initialize()
        self.fig, self.ax1, self.ax3 = self.plot_initialize()

        #self.num_obstacle = 0
        #self.obstacle = [Object(self.world_size, 0.0) for i in range(self.num_obstacle)]

        self.obstacle = obstacle_pos().check_obs()
        self.num_obstacle = len(self.obstacle)
        #print("\n0~{}: {}個の障害物\n" .format(self.num_obstacle-1, self.num_obstacle))
        self.plan_counting = 0
        


        self.main_loop()

    @staticmethod
    def plot_initialize():
        """グラフを初期化

        　　グラフを初期化し、ハンドルを返します

            Args:
                None

            Returns:
                fig (fig object): matplotlib の figオブジェクト
                ax (ax object): グラフ１のaxesオブジェクト
                ax2 (ax object): グラフ2のaxesオブジェクト
                ax3 (ax object): グラフ3のaxesオブジェクト
                ax4 (ax object): グラフ4のaxesオブジェクト

            Examples:

                現在の各関節角度に加算したい角度[deg.]を引数として渡し、関数を呼び出します

                >>> fig, ax, ax2, ax3  = plot_initialize()

        """

        fig = plt.figure(figsize=plt.figaspect(0.3))
        ax = fig.add_subplot(1, 2, 1, projection='3d', xlabel='x', ylabel='y', zlabel='z')
        ax2 = fig.add_subplot(1, 2, 2, xlabel='x', ylabel='y')
        #ax3 = fig.add_subplot(1, 2, 3, xlabel='x', ylabel='y')
        return fig, ax, ax2

    def plot_robot_in_world_coord(self):

        self.ax1.cla()
        self.ax1.set_xlim([self.world_size[0], self.world_size[3]])
        self.ax1.set_ylim([self.world_size[1], self.world_size[4]])
        self.ax1.set_zlim([self.world_size[2], self.world_size[5]])
        self.ax1.set_title("world")
        world = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        self.ax1.plot([0.0, world[0, 0]], [0.0, world[0, 1]], [0.0, world[0, 2]], color="red")
        self.ax1.plot([0.0, world[1, 0]], [0.0, world[1, 1]], [0.0, world[1, 2]], color="green")
        self.ax1.plot([0.0, world[2, 0]], [0.0, world[2, 1]], [0.0, world[2, 2]], color="blue")

    def plot_camera_pose_in_world_coord(self):
        """world座標系上にカメラ座標をplot

             world座標系上にカメラ座標を表す単位ベクトルをplotします

            Args:
                None

            Returns:
                None

            Examples:

                >>> plot_camera_pose_in_world_coord()

        """
        world = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        camera = (np.dot(self.rotation, world))

        self.ax1.plot([self.translation[0, 0], self.translation[0, 0] + camera[0, 0]],
                      [self.translation[1, 0], self.translation[1, 0] + camera[0, 1]],
                      [self.translation[2, 0], self.translation[2, 0] + camera[0, 2]], color="blue")
        self.ax1.plot([self.translation[0, 0], self.translation[0, 0] + camera[1, 0]],
                      [self.translation[1, 0], self.translation[1, 0] + camera[1, 1]],
                      [self.translation[2, 0], self.translation[2, 0] + camera[1, 2]], color="green")
        self.ax1.plot([self.translation[0, 0], self.translation[0, 0] + camera[2, 0]],
                      [self.translation[1, 0], self.translation[1, 0] + camera[2, 1]],
                      [self.translation[2, 0], self.translation[2, 0] + camera[2, 2]], color="red")
    

    def draw_world_hokan(self):

        offset_all = self.arm.offset()
        #print(offset_all)

        p0 = [0, 0, 0]
        self.ax1.plot(p0[0], p0[1], p0[2], marker='.', markersize=8, color="blue")
        p1 = offset_all[0]
        self.ax1.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]], color="blue")

        for i in range(1):
            p0 = offset_all[i]
            p1 = offset_all[i] + offset_all[i+1]
            self.ax1.plot(p0[0], p0[1], p0[2], marker='.', markersize=8, color="blue")
            self.ax1.plot(p1[0], p1[1], p1[2], marker='.', markersize=8, color="blue")
            self.ax1.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]], color="blue")


    def draw_world(self):

        self.ax1.plot(self.target.pos[0], self.target.pos[1], self.target.pos[2], marker='.', markersize=12, color="red")

        # 障害物
        size = 35
        for i in range(self.num_obstacle):
            self.ax1.plot(self.obstacle[i].pos[0], self.obstacle[i].pos[1], self.obstacle[i].pos[2],
                          marker='.', markersize=size, color="green")

        # アーム
        for i in range(self.arm.num_link-1):

            p0 = self.arm.joint3d(i)
            p1 = self.arm.joint3d(i + 1)

            self.ax1.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]], color="blue")

        # joint
        for i in range(self.arm.num_link-1):
            p1 = self.arm.joint3d(i)
            self.ax1.plot(p1[0], p1[1], p1[2], marker='.', markersize=8, color="blue")

        count = 1

        self.draw_world_hokan()
        
        for index in range(self.arm.num_link * self.arm.num_force_point + 1):
            if index >= self.arm.num_link * self.arm.num_force_point:
                pos = self.arm.pos3d()
                self.ax1.plot(pos[0], pos[1], pos[2], marker='.', markersize=4, color="cyan")
            else:
                link = index//self.arm.num_force_point
                
                # ベース（固定リンク）は力点がないので 3d plot では力点は移さない，代わりに青い棒だけ写す
                if link != 0:
                #ベースが二個ある時
                #if link > 1:
                    pos = self.arm.pos3d(link_index=link, t=1. / (self.arm.num_force_point + 1.) * count)
                    self.ax1.plot(pos[0], pos[1], pos[2], marker='.', markersize=4, color="cyan")
            count += 1
            if count > self.arm.num_force_point:
                count = 1

    def draw_world_2d(self):
        #self.arm.joint_angle[0] = self.arm.arpha
        self.ax3.cla()
        self.ax3.grid()
        self.ax3.set_title("2d image")
        self.ax3.set_xlim([self.world_size[0], self.world_size[3]])
        self.ax3.set_ylim([self.world_size[1], self.world_size[4]])

        self.ax3.plot(self.target.pos[0], self.target.pos[1], marker='.', markersize=12, color="red")

        # 障害物
        for i in range(self.num_obstacle):
            self.ax3.plot(self.obstacle[i].pos[0], self.obstacle[i].pos[1],
                          marker='.', markersize=12, color="green")

        # アーム
        p0 = [0, 0, 0]
        p1 = self.arm.joint3d(0)
        self.ax3.plot([p0[0], p1[0]], [p0[1], p1[1]], color="blue")

        for i in range(self.arm.num_link-1):

            p0 = self.arm.joint3d(i)
            p1 = self.arm.joint3d(i + 1)

            self.ax3.plot([p0[0], p1[0]], [p0[1], p1[1]], color="blue")

        # joint
        for i in range(self.arm.num_link-1):
            p1 = self.arm.joint3d(i)
            self.ax3.plot(p1[0], p1[1], marker='.', markersize=6, color="blue")
            #self.ax3.quiver(p1[0], p1[1], self.arm.d[i][0], self.arm.d[i][1],
                                #angles='xy', scale_units='xy', scale=50)

        count = 1

        # 力のベクトル
        for index in range(self.arm.num_link * self.arm.num_force_point + 1):
            if index >= self.arm.num_link * self.arm.num_force_point:
                pos = self.arm.pos3d()
                self.ax3.plot(pos[0], pos[1], marker='.', markersize=4, color="cyan")
                self.ax3.quiver(pos[0], pos[1], self.arm.d[index][0], self.arm.d[index][1],
                                angles='xy', scale_units='xy', scale=50)
                # print(np.linalg.norm(self.arm.d[index], ord=2))
                # print(index, self.arm.u[index])
                # print(self.arm.joint_angle*180./np.pi)
            else:
                link = index//self.arm.num_force_point
                if link != 0:
                    pos = self.arm.pos3d(link_index=link, t=1. / (self.arm.num_force_point + 1.) * count)
                    self.ax3.plot(pos[0], pos[1], marker='.', markersize=4, color="cyan")
                    self.ax3.quiver(pos[0], pos[1], self.arm.d[index][0], self.arm.d[index][1], angles='xy',
                                    scale_units='xy', scale=50)
                count += 1
            if count > self.arm.num_force_point:
                count = 1

    # def draw_heatmap(self, potential_target=True):
    #
    #     # 解像度
    #     xn = 40
    #     yn = 40
    #
    #     dx = int((self.world_size[3] - self.world_size[0]) / xn)
    #     dy = int((self.world_size[4] - self.world_size[1]) / yn)
    #     xs = np.linspace(self.world_size[0], self.world_size[3], xn)
    #     ys = np.linspace(self.world_size[1], self.world_size[4], yn)
    #
    #     XX, YY = np.meshgrid(xs, ys)
    #
    #     # ZZ = self.get_potential(XX, YY, target=potential_target)
    #     ZZ = self.get_potential_from_image(XX, YY, target=potential_target)
    #     self.ax2.imshow(ZZ, interpolation='nearest', cmap='jet')
    #     self.ax2.set_xticks(XX)
    #     self.ax2.set_yticks(YY)

    def draw_camera_image(self):
        self.ax2.cla()
        self.ax2.grid()
        self.ax2.set_title("image")
        self.ax2.set_xlim([0.0, 0.64])
        self.ax2.set_ylim([0.48, 0.0])

        target_pos_in_camera = self.cam.convert_world_to_camera_coord(self.target.pos.T, self.rotation, self.translation)
        target_pos_in_image = self.cam.pixel_at_xyz(target_pos_in_camera.T)
        self.ax2.plot(target_pos_in_image[-1][0],target_pos_in_image[-1][1], marker='.', markersize=12, color="red")

        # 障害物
        for i in range(self.num_obstacle):
            obstacle_pos_in_camera = \
                self.cam.convert_world_to_camera_coord(self.obstacle[i].pos.T, self.rotation, self.translation)
            obstacle_pos_in_image = self.cam.pixel_at_xyz(obstacle_pos_in_camera.T)
            self.ax2.plot(obstacle_pos_in_image[-1][0], obstacle_pos_in_image[-1][1], marker='.', markersize=12, color="green")

        # アーム
        for i in range(self.arm.num_link):
            p0 = self.cam.convert_world_to_camera_coord(self.arm.joint3d(i).T, self.rotation, self.translation)
            p1 = self.cam.convert_world_to_camera_coord(self.arm.joint3d(i + 1).T, self.rotation, self.translation)
            p0_in_image = self.cam.pixel_at_xyz(p0.T)
            p1_in_image = self.cam.pixel_at_xyz(p1.T)
            lines = [[p0_in_image[-1][0], p1_in_image[-1][0]], [p0_in_image[-1][1], p1_in_image[-1][1]]]
            self.ax2.plot([p0_in_image[-1][0], p1_in_image[-1][0]], [p0_in_image[-1][1], p1_in_image[-1][1]], color="blue")

        # joint
        for i in range(self.arm.num_link):
            p1 = self.cam.convert_world_to_camera_coord(self.arm.joint3d(i).T, self.rotation, self.translation)
            p1_in_image = self.cam.pixel_at_xyz(p1.T)
            self.ax2.plot(p1_in_image[-1][0], p1_in_image[-1][1], marker='.', markersize=12, color="blue")

        count = 1
        for index in range(self.arm.num_link * self.arm.num_force_point + 1):
            if index >= self.arm.num_link * self.arm.num_force_point:
                pos = self.cam.convert_world_to_camera_coord(self.arm.pos().T, self.rotation, self.translation)
                pos_image = self.cam.pixel_at_xyz(pos.T)
                self.ax2.plot(pos_image[-1][0], pos_image[-1][1], marker='.', markersize=8, color="cyan")
            else:
                link = index//self.arm.num_force_point
                pos = self.cam.convert_world_to_camera_coord(self.arm.pos(arm_index=link,
                                                                          t=1./(self.arm.num_force_point + 1.) * count).T, self.rotation, self.translation)
                pos_image = self.cam.pixel_at_xyz(pos.T)
                self.ax2.plot(pos_image[-1][0], pos_image[-1][1], marker='.', markersize=8, color="cyan")
            count += 1
            if count > self.arm.num_force_point:
                count = 1

    def distance(self):
        d = np.linalg.norm(self.target.pos[:] - self.arm.joint3d(self.arm.num_link-1))
        
        print("現在位置{}" .format(self.arm.joint3d(self.arm.num_link-1)))
        print("target距離{}". format(d))
        
        return d

    def get_joint_pos_angle(self):
        joint_plan_first = [self.arm.joint_angle]
        self.joint_plan_first = joint_plan_first
        angle_rad = copy.deepcopy(self.arm.joint_angle)
        
        angle_rad = np.delete(angle_rad, 0, 0)
        #angle_rad = np.append(angle_rad, [0])

        joint_num = len(angle_rad)
        #print(joint_num)

        angle_deg = []

        for i in range(joint_num):
            #print(np.rad2deg(angle_rad[i]))
            angle_deg = np.append(angle_deg, np.rad2deg(angle_rad[i]))
        
        print(angle_deg)

        pos = []
        for i in range(self.arm.num_link):
            pos = self.arm.joint3d(i)
            if i == 0:
                print('Base  link{}間 (x, y, z)={}'.format(i+1, pos))
                #print('               angle={} rad.' .format(angle[i]))
            elif i == self.arm.num_link-1:
                print('手先位置      (x, y, z)={}'.format(pos))
                #print('               angle={} rad.' .format(angle[self.arm.num_link-1]))
            else:
                print('link{} link{}間 (x, y, z)={}' .format(i, i+1, pos))
                #print('               angle={} rad.' .format(angle[i]))

        


    def update_obstacle_position(self):
        #self.target.move()

        self.num_obstacle = 3
        self.obstacle = [Object(self.world_size, 0.0) for i in range(self.num_obstacle)]

        self.obstacle[0].pos[:] = [-.1, .0, 0.]
        self.obstacle[1].pos[:] = [-.1, .1, 0.]
        self.obstacle[2].pos[:] = [-.1, .2, 0.]
        #self.obstacle[3].pos[:] = [-.1, .3, 0.]
        #self.obstacle[4].pos[:] = [-.1, .4, 0.]
        #self.obstacle[5].pos[:] = [-.2, .25, 0.]
        #self.obstacle[6].pos[:] = [-.2, .3, 0.]
        #self.obstacle[7].pos[:] = [-.15, .15, 0.]
    

    def update_target_position(self, position):
        self.target.pos[:] = position


    
    
    def reserve_plan(self):
        joint_angle = self.arm.joint_angle

        joint_angles = copy.deepcopy(joint_angle)    
        if self.plan_counting ==0:
            
            joint_plan = np.append(self.joint_plan_first, [joint_angles], axis=0)
            self.plan_counting = 1

        else:
            joint_plan = self.joint_plan
            joint_plan = np.append(joint_plan, [joint_angles], axis=0)



        #print(joint_plan)
        self.joint_plan = joint_plan

    

    def jitu_sensa_tesaki_pos_distance(self):
        jitukukan_tesaki_pos = copy.deepcopy(self.arm.jitu_kukan_tesaki_pos())
        #self.arm.joint_angle[0] = self.arm.arpha
        target_pos = self.target.pos[:]
        vector = [target_pos[0]-jitukukan_tesaki_pos[0], target_pos[1]-jitukukan_tesaki_pos[1], target_pos[2]-jitukukan_tesaki_pos[2]]

        print("実空間手先位置{}" .format(jitukukan_tesaki_pos))
        d = np.linalg.norm(target_pos - jitukukan_tesaki_pos)
        print("実空間手先位置とtarget距離{}". format(d))
        
        return [d, vector]

    def main_loop(self):
        count = 0
        #print("実空間座標系での開始手先位置\n{}" .format(self.arm.jitu_kukan_tesaki_pos()))

        self.get_joint_pos_angle()
        path_pos = self.arm.Reading_File()
        num_path_pos = len(path_pos)
        print("経路点[0]~[{}]の{}個\n\n" .format(num_path_pos-1, num_path_pos))
        potential_omomi_count = 0

        while True:

            if count == 0:
                self.update_target_position(path_pos[count])
                print("経路点[{}]: {}\n" .format(count, self.target.pos[:]))
                self.arm.move2(self.target.pos, self.obstacle, count)
                count += 1
                self.plot_robot_in_world_coord()
                self.plot_camera_pose_in_world_coord()
                self.draw_world()

                self.draw_world_2d()

                plt.pause(5.0)
            
            elif count < num_path_pos:
                if self.distance() < 0.03:
                    self.update_target_position(path_pos[count])
                    print("target更新! 経路点[{}]->[{}]" .format(count-1, count))
                    print("経路点[{}]: {}\n" .format(count, self.target.pos[:]))
                    count += 1
                    potential_omomi_count = 0
                    self.potential.omomi_K = 2.5
                    self.potential.shogaibutu_kyoyou_l = 0.02
                    if count == num_path_pos:
                        print("target更新終了")
                        print("最終目標経路点[{}]（最終目標手先位置）:{}\n" .format(count-1, self.target.pos[:]))
                        # ここで，count == num_path_pos
            elif count == num_path_pos:
                #self.distance()
                if self.distance() < 0.03:
                    count += 1
                    # ここで，count == num_path_pos+1
            else:
                if self.distance() <=  0.02:
                    print("目標に到達成功")
                    count += 1
                    # ここで，count == num_path_pos+2
            #"""
            i = potential_omomi_count//15
            #print(i)
            #print("i")

            if i == 1:
                if potential_omomi_count == 15:
                    print(potential_omomi_count)
                    self.potential.shogaibutu_kyoyou_l -= 0.02
                    print("行き詰まっているので v_obstacle の障害物距離許容範囲を小さく変更")
                #print(self.potential.shogaibutu_kyoyou_l)

                self.potential.nazo = 200
                self.potential.target_omomi = 1000
                print("行き詰まっているので v_obstacle の重みを小さく変更")
                print("行き詰まっているので v_target の重みを大きく変更")

            if i >= 2:
                if count < num_path_pos:
                    self.update_target_position(path_pos[count])
                    print("target更新! 経路点[{}]->[{}]" .format(count-1, count))
                    print("経路点[{}]: {}\n" .format(count, self.target.pos[:]))
                    count += 1
                    self.potential.omomi_K = 2.5
                    self.potential.shogaibutu_kyoyou_l = 0.02
                    potential_omomi_count = 0
                else:
                    self.arm.v_obstacle_flag = 0
            
                

                
            if count < num_path_pos+2:
                self.arm.move2(self.target.pos, self.obstacle, count)
                potential_omomi_count += 1
                print("count-potential_omomi_count", potential_omomi_count)
            
            

            # if count%50 == 0:
            self.arm.joint_angle[0] = 0
            #self.arm.test()
            self.plot_robot_in_world_coord()
            self.plot_camera_pose_in_world_coord()
            self.draw_world()
            #self.arm.joint_angle[0] = self.arm.arpha

            self.draw_world_2d()
                #self.draw_heatmap()
                #self.draw_camera_image()
            

            #self.reserve_plan()
            if self.arm.shototu_hantei(self.obstacle) == False:
                print("衝突発生")
                plt.pause(100)
            elif count == num_path_pos+3:
                plt.pause(100)
            else:
                plt.pause(0.001)
            #print(self.arm.jitu_kukan_tesaki_pos())
            #self.arm.joint_angle[0] = self.arm.arpha

    def update_object_position(self):
        self.target.move()

        # for i in range(self.num_obstacle):
        #     self.obstacle[i].move()


if __name__ == '__main__':

    sim = Simulation()
    sim.main_loop()
