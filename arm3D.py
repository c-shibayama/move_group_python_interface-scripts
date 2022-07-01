# -*- coding: UTF-8 -*-
import numpy as np
import copy
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation
import numpy.linalg as LA
from math import sqrt, cos, sin, acos
import csv
from decide_obstacle_position import obstacle_pos

import potential as pot

class Link:

    def __init__(self, *args, **kwargs):
        self.name = "None"
        self.pos = np.zeros(3)                                # ジョイントの位置
        self.omega_local = np.zeros(3)                        # ローカル座標における回転軸を表すベクトルの初期値
        self.omega = np.zeros(3)                              # ローカル座標における回転軸を表すベクトル
        self.offset = np.zeros(3)                             # １つ前のローカル座標に対するオフセット [m]
        self.rot = Rotation.from_euler('xyz', np.array([0.0, 0.0, 0.0]))
        self.joint = False                                    # False:ジョイントのない固定リンク, True:ジョイントのある稼働リンク
        self.joint_index = -1                                 # ジョイントインデックス, 非ジョイントは-1

class Arm2:

    def __init__(self, **kwargs):
        link_info = kwargs.get('link_info')
        self.num_link = len(link_info) - 1                    # リンク数
        # 初期setting.jsonを使う場合
        #self.joint_angle = [0., 0., np.deg2rad(60.), np.deg2rad(30.)]                 # 初期姿勢[rad.]
        
        # setting_jisaku6def_arm.json
        #self.joint_angle = [0., 0., np.deg2rad(45), np.deg2rad(-135), np.deg2rad(0), np.deg2rad(-90), np.deg2rad(0)]

        # setting_shifted_jisaku6dof_arm.json
        #self.joint_angle = [0., 0., 0., np.deg2rad(45), np.deg2rad(-135), np.deg2rad(0), np.deg2rad(-90), np.deg2rad(0)]

        # setting_kaiten_gosa_arm.json
        arpha = np.deg2rad(0)
        self.joint_angle = [arpha, np.deg2rad(0), np.deg2rad(45), np.deg2rad(-135), np.deg2rad(0), np.deg2rad(-90), np.deg2rad(0)]


        # 試しに4軸アームを用いる時
        # setting_7de.json
        #self.joint_angle = [0., 0., np.deg2rad(60.), np.deg2rad(30.)]

        self.delta_theta = np.zeros(self.num_link)            # 算出される角速度格納用変数[deg.]
        self.num_force_point = 5                              # 各リンク上に配置される力点の数
        self.link = [Link() for i in range(len(link_info))]   # Linkクラスのインスタンス

        # setting.jsonに記載された情報を代入
        joint_index = 0
        offset_all = []
        for i, val in enumerate(link_info):
            self.link[i].name = val["name"]
            self.link[i].omega_local = np.array(val["omega"])
            self.link[i].offset = np.array(val["offset"])
            offset_all.append(val["offset"])
            self.link[i].joint = val["joint"]

            if self.link[i].joint:
                self.link[i].joint_index = joint_index
                joint_index += 1

        self.interaction_info = list(kwargs.get('interaction_info'))
        self.pot = pot.Potential()                            # potential用のインスタンス
        self.d = np.empty((0))                                # ポテンシャルから各力点が受ける力ベクトル（ロボット座標系）
        self.u = np.empty((0))                                # 関節トルク格納用変数
        self.u_old = np.zeros(self.num_link)                  # 1ステップ前の関節トルク格納用変数
        self.offset_all = offset_all
        self.arpha = arpha
        self.v_obstacle_flag = 1
    
    def offset(self):
        #print(offset)
        offset_all = self.offset_all
        return offset_all

            
    def joint3d(self, joint_index):
        """ジョイントの位置を計算する関数

        　　引数で指定されたジョイントの位置を算出します

            Args:
                joint_index: ジョイントのインデックス（0～）

            Returns:
                pos: ロボット座標系におけるジョイント位置[x,y,z] [m]

            Examples:

                位置を算出したいジョイントのインデックスを引数として、関数を呼び出します

                >>> [x,y,z]  = joint3d(2)

        """
        pos = np.r_[0.0, 0.0, 0.0]
        rot = Rotation.from_euler('xyz', np.array([0.0, 0.0, 0.0]))

        for i in range(joint_index+1):
            rot = rot * Rotation.from_euler('xyz', self.link[i].omega_local * self.joint_angle[i], degrees=False)
            offset = rot.apply(self.link[i].offset)
            pos = pos + offset

        return pos

    def pos3d(self, link_index=None, t=1.0):
        """力点の位置を計算する関数

        　　引数で指定されたジョイントの位置を算出します

            Args:
                link_index: 力点の位置するリンクのインデックス（0～）
                t: リンク長を1とした時の力点の間隔

            Returns:
                pos: ロボット座標系における指定された力点の位置[x,y,z] [m]

            Examples:

                位置を算出したいジョイントのインデックスを引数として、関数を呼び出します

                >>> [x,y,z]  = pos3d(1,0.5)

        """
        if link_index is None:
            link_index = self.num_link - 1

        l = np.empty((0,3), float)
        for i in range(len(self.link) - 1):
            l = np.append(l, np.array([self.link[i].offset]), axis=0)

        l[link_index] = l[link_index] * t

        pos = np.r_[0.0, 0.0, 0.0]
        rot = Rotation.from_euler('xyz', np.array([0.0, 0.0, 0.0]))

        for i in range(link_index + 1):
            rot = rot * Rotation.from_euler('xyz', self.link[i].omega_local * self.joint_angle[i], degrees=False)
            offset = rot.apply(l[i])
            pos = pos + offset

        return pos

    
    

    def jitu_kukan_tesaki_pos(self):
        self.joint_angle[0] = 0
        jitu_kukan_tesaki_pos = self.joint3d(self.num_link-1)
        return jitu_kukan_tesaki_pos


    def Reading_File(self):
        # trajectory_position_list_0211_???.csvを選ぶ
        with open('/home/cshiba/kyoudoukenkyu/autonomous-control-simulation-master/trajectory_position_list/trajectory_position_list_0211_second.csv') as f:
            #print("reading file", f)
            reader = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
    
            trajectory_position_list = [trajectory_position for trajectory_position in reader]
            l = len(trajectory_position_list)
            #for i in range(l):

                #trajectory_position_list[i][0] += 1.5-0.03
                #trajectory_position_list[i][1] += -0.02
                #print(trajectory_position_list[i])

            #print(trajectory_position_list)
        
        return trajectory_position_list

    def move2(self, p_target, obstacles, count):
        self.update_transform()
        p_joint, omega_joint, p_action, action_condition = self.set_condition()
        #print(p_action)
            

        mix_const = 1
        dangles = self.get_delta_joints(p_action, action_condition, p_joint, omega_joint, p_target, obstacles)
        if count == 0:
            return
        self.delta_theta = (1. - mix_const) * self.delta_theta + mix_const * np.deg2rad(dangles)
        self.joint_angle = self.joint_angle + self.delta_theta * 0.01
        #self.joint_angle[0] = 0
        #print(dangles)

    def shototu_hantei_distance(self, p_some, obstacle):
        num_obstacle = len(obstacle)

        for i in range(num_obstacle):
            d = np.linalg.norm(obstacle[i].pos - p_some)
            if d > 0.02:
                i
            else:
                print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n\n\n\n\n\AAAAAAAAAAAAAA")
                return False
        return True


    def shototu_hantei(self, obstacle):
        self.update_transform()
        p_joint, omega_joint, p_action, action_condition = self.set_condition()
        num_p_joint = len(p_joint)
        num_p_action = len(p_action)
        for i in range(num_p_joint):
            if self.shototu_hantei_distance(p_joint[i], obstacle):
                #print("joint衝突なし")
                i
            else:
                print("joint衝突あり{}" .format(p_joint[i]))
                return False
        for i in range(num_p_action):
            if self.shototu_hantei_distance(p_action[i], obstacle):
                #print("力点衝突なし")
                i
            else:
                print("力点衝突あり{}"  .format(p_action[i]))
                
                return False
        return True

    def test(self):
        for i in range(3):
            d = 0.01
            if d > 0.02:
                i
            else:
                print("終わるはず")
                return False
        return True
            



        


    def get_delta_joints(self, p_action, potential_conf, p_node, omega, p_target, obstacles):
        num_joints = len(p_node)

        # 行列とベクトル用意
        J = np.empty((0, num_joints))
        d = np.empty((0))

        t = []

        for index, (p_act, pot_conf) in enumerate(zip(p_action, potential_conf)):
            J = np.r_[J, self.get_J(p_act, index, p_node, omega, num_joints)]
            d = np.r_[d, self.get_d(p_act, pot_conf, p_target, obstacles)]
            t.append(np.array(self.get_d(p_act, pot_conf, p_target, obstacles)))

        self.d = t

        res = lsq_linear(J, d, tol=1.0e-0)

        # この係数を変えれば大きく動くかも
        # （大きくすると，障害物が近いときにはぶれたときに障害物に衝突するため目標に近づけない）
        delta_angle = res.x * 1.
        #delta_angle = res.x * 2.
        # print(delta_angle)

        return np.r_[0., delta_angle]

    def get_J(self, p_action, index, p_joint, omega, num_joints):

        idx = index // self.num_force_point

        if idx >= self.num_link:
            J = np.array([np.cross(omega[i], p_action - p_joint[i]) for i in range(idx-1)])
            J = np.r_[J, np.zeros([num_joints - (idx-1), 3])]
        elif idx == 0:
            J = np.zeros((num_joints, 3))
            
        # shifted_baseを作成し、不要なjoint数が一つ増えた時、エラー発生
        else:
            J = np.array([np.cross(omega[i], p_action - p_joint[i]) for i in range(idx)])
            J = np.r_[J, np.zeros([num_joints - idx, 3])]

        J = J.T

        return J

    def get_d(self, p_action, potential_conf, p_target, p_obstacles):
        d = np.zeros(3)
        v_obstacle_flag = self.v_obstacle_flag

        

        # メールで教えていただいたところ
        # 一個下の def set_condition() における
        # condition に記載される

        if potential_conf[0]:
            d += -1.0 * self.pot.grad_v_target(0.01, p_action, p_target)

        if potential_conf[1]:
            if self.v_obstacle_flag:
                d += -1.0 * self.pot.grad_v_obstacle(0.01, p_action, p_obstacles)
            else:
                d += 0

        return d

    def set_condition(self):
        p_joint = np.array([])  # ジョイントの位置 RB座標系
        omega_joint = np.array([])  # ジョイントの回転ベクトル　RB座標系
        p_action = np.array([])  # 力を受ける力点の位置
        # joint_index_action = np.array([], dtype=np.int)  # 力が作用するジョイントのインデックス
        # # p_joint[0], ..., p_joint[joint_index_action]　->>>　作用する
        # # p_joint[joint_index_action+1], ... ->>>　作用しない
        action_condition = np.array([], dtype=np.bool)  # 力の種別

        # 作用点
        for i, link in enumerate(self.link):
            if link.joint:
                p_joint = np.append(p_joint, link.pos).reshape(-1, 3)
                omega_joint = np.append(omega_joint, link.omega).reshape(-1, 3)

        # 力点
        count = 1


        for index in range(self.num_link * self.num_force_point + 1):
            if index >= self.num_link * self.num_force_point:
                pos = self.pos3d()
                # print(pos)
            else:
                link = index//self.num_force_point
                pos = self.pos3d(link_index=link, t=1. / (self.num_force_point + 1.) * count)
            count += 1
            if count > self.num_force_point:
                count = 1
            

            p_action = np.append(p_action, pos).reshape(-1, 3)

            # メールで教えていただいたところ↓
            # [引力を受けるかのフラグ, 斥力を受けるかのフラグ] 
            # [True, True]: 引力斥力の両方を受ける手先
            # [False, True]: 斥力のみを受けるリンク上の力点
            if index >= self.num_link * self.num_force_point:
                condition = [True, True]
            # 固定リンク（ベース）は力を受けないように変更
            elif index < self.num_force_point:
                condition = [False, False]
            
            else:
                condition = [False, True]

            action_condition = np.append(action_condition, condition).reshape(-1, 2)

        return p_joint, omega_joint, p_action, action_condition

    def update_transform(self):

        #angles = np.array([self.joint_angle[0], self.joint_angle[1], self.joint_angle[2], self.joint_angle[3]])
        angles = []
        joint_angle_num = len(self.joint_angle)
        for i in range(joint_angle_num):
            angles = np.append(angles, self.joint_angle[i])
            
        rot = Rotation.from_euler('xyz', np.array([0.0, 0.0, 0.0]))

        for i in range(len(self.link) - 1):
            rot = rot * Rotation.from_euler('xyz', self.link[i].omega_local * angles[i], degrees=False)
            offset = rot.apply(self.link[i].offset)
            self.link[i + 1].pos = self.link[i].pos + offset
            self.link[i].omega = rot.apply(self.link[i].omega_local)
            self.link[i].rot = rot
