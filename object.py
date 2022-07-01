import numpy as np


class Object:
    def __init__(self, world_size, speed):
        if 1:
            self.pos = np.r_[
                np.random.uniform(world_size[0], world_size[3]), np.random.uniform(world_size[1], world_size[4]), 0.0]
        else:
            self.pos = np.r_[0.0, 0.0, 0.0]

        self.dir = np.r_[0.0, 0.0, 0.0]
        self.pos_pre = np.r_[0.0, 0.0, 0.0]
        self.dir_pre = np.r_[0.0, 0.0, 0.0]
        self.speed = speed
        self.world_size = world_size

    def move(self):
        alpha = 0.99
        self.dir = alpha * self.dir + (1.0 - alpha) * np.random.randn(1, 2).flatten() * self.speed
        self.pos = self.pos + self.dir

        if self.pos[0] < self.world_size[0] + 0.1:
            self.pos[0] = self.world_size[0] + 0.1

        if self.pos[0] > self.world_size[2] - 0.1:
            self.pos[0] = self.world_size[2] - 0.1

        if self.pos[1] < self.world_size[1] + 0.1:
            self.pos[1] = self.world_size[1] + 0.1

        if self.pos[1] > self.world_size[3] - 0.1:
            self.pos[1] = self.world_size[3] - 0.1