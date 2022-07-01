# coding: UTF-8
import numpy as np


class Camera:
    """Camera class

        カメラの位置・姿勢変更、透視投影変換を定義したクラス

        Attributes:
            fx (float): 焦点距離 fx [m]
            fy (float): 焦点距離 fy [m]
            ppx (float): 画像原点 ppx [m]
            ppy (float): 画像原点 ppy [m]
            camera_matrix (float): 内部パラメータ行列
            resolution (float): キー入力時の単位変位角度 [deg.]
            th1 (float): ロボット座標系に対するカメラ座標の姿勢を表すx-y-z系オイラー角 x [deg.]
            th2 (float): ロボット座標系に対するカメラ座標の姿勢を表すx-y-z系オイラー角 y [deg.]
            th3 (float): ロボット座標系に対するカメラ座標の姿勢を表すx-y-z系オイラー角 z [deg.]
            r (float): ロボット座標系に対するカメラ座標原点位置を極座標で表した際の球の半径 r [m]
            theta (flaot): ロボット座標系に対するカメラ座標原点位置を極座標で表した際の偏角Θ [deg.]
            phi (flaot): ロボット座標系に対するカメラ座標原点位置を極座標で表した際の偏角Φ [deg.]
    """
    def __init__(self):
        """コンストラクタ

         カメラの内部パラメータ、カメラ座標を初期化します

        Args:
            None

        Returns:
            None
        """
        self.fx = 0.612928    # 612.928 [mm]
        self.fy = 0.611466    # 611.466 [mm]
        self.ppx = 0.317672   # 317.672 [mm]
        self.ppy = 0.24075    # 240.75 [mm]
        self.camera_matrix = np.array([[self.fx, 0.0, self.ppx],
                                       [0.0, self.fy, self.ppy],
                                       [0.0, 0.0, 1.0]])
        self.resolution = 5.0
        self.th1 = 90.0
        self.th2 = 0.0
        self.th3 = 0.0
        self.r = 5.0
        self.theta = 0.0 # 70,80,85
        self.phi = -90.0

    def pixel_at_xyz(self, xyz_cam):
        """カメラ座標系上の位置を画像座標系上に投影します

         x = f * X / Z, y = f * X / Z

        Args:
            xyz_cam (float): カメラ座標系上の位置 [x, y, z] [m]

        Returns:
            xy_pixs (flaot): カメラ座標系上の位置 [x, y, z]を画像座標系上に投影した位置 [u,v] [m]

        Examples:

            現在の各関節角度に加算したい角度[deg.]を引数として渡し、関数を呼び出します

            >>> pixel_at_xyz([x, y, z])
                [u,v]

        Note:
            戻り値が[pixel]ではなく[m]であることに注意

        """
        z_cam = xyz_cam[:,2]

        z_cam[z_cam < 1.0e-12] = 1.0e-12

        xy_pixs = np.c_[xyz_cam[:,0] * self.fx / z_cam + self.ppx,
                        xyz_cam[:,1] * self.fy / z_cam + self.ppy]
        return xy_pixs

    def generate_camera_pose(self):

        x = self.r * np.sin(np.deg2rad(self.theta)) * np.cos(np.deg2rad(self.phi))
        y = self.r * np.sin(np.deg2rad(self.theta)) * np.sin(np.deg2rad(self.phi))
        z = self.r * np.cos(np.deg2rad(self.theta))
        t = np.array([[x, y, z]]).T

        Rz1 = np.array([[np.cos(np.pi / 2. + np.deg2rad(self.phi)), np.sin(np.pi / 2. + np.deg2rad(self.phi)), 0.],
              [-np.sin(np.pi / 2. + np.deg2rad(self.phi)), np.cos(np.pi / 2. + np.deg2rad(self.phi)), 0.],
              [0., 0., 1.]])
        Rx = np.array([[1., 0., 0.],
              [0., np.cos(-(np.pi - np.deg2rad(self.theta))), np.sin(-(np.pi - np.deg2rad(self.theta)))],
              [0., -np.sin(-(np.pi - np.deg2rad(self.theta))), np.cos(-(np.pi - np.deg2rad(self.theta)))]])
        R = np.dot(Rx, Rz1)

        return R, t

    def update_camera_pose_with_euler(self):

        cos1 = np.cos(np.radians(self.th1))
        cos2 = np.cos(np.radians(self.th2))
        cos3 = np.cos(np.radians(self.th3))
        sin1 = np.sin(np.radians(self.th1))
        sin2 = np.sin(np.radians(self.th2))
        sin3 = np.sin(np.radians(self.th3))

        Rotx = np.array([[1.0, 0.0, 0.0],
                         [0.0, cos1, -sin1],
                         [0.0, sin1, cos1]])
        Roty = np.array([[cos2, 0.0, sin2],
                         [0.0, 1.0, 0.0],
                         [-sin2, 0.0, cos2]])
        Rotz = np.array([[cos3, -sin3, 0.0],
                         [sin3, cos3, 0.0],
                         [0.0, 0.0, 1.0]])
        R = Rotx.dot(Roty).dot(Rotz)

        return R

    def update_camera_pose_with_polar(self):

        cos_theta = np.cos(np.radians(self.theta))
        cos_phi = np.cos(np.radians(self.phi))
        sin_theta = np.sin(np.radians(self.theta))
        sin_phi = np.sin(np.radians(self.phi))

        x = self.r * sin_theta * cos_phi
        y = self.r * sin_theta * sin_phi
        z = self.r * cos_theta
        t = np.array([[x, y, z]]).T

        return t

    @staticmethod
    def convert_world_to_camera_coord(pos_robot, rotation_mat, t):
        return np.dot(rotation_mat, (pos_robot - t.T).T)

