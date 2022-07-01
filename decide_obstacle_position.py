from object import Object
import numpy as np

class obstacle_pos:
    def __init__(self):
        self.world_size = [-1., -1., 0.0, 1., 1., 1.]
        self.num_obstacle = 100
        self.obstacle = [Object(self.world_size, 0.0) for i in range(self.num_obstacle)]

    def check_obs(self):

        height = 0.2
        height_ver2 = 0.44
        height_ver3 = 0.24
        radius =0.02
        x_1 = 0.3
        y_1 = -0.22
        x_2 = x_1 -0.2

        box1_pos = [x_1, y_1, radius]
        box2_pos = [x_1-x_1/4, y_1/2, radius]
        box3_pos = [x_1/2, y_1/2, radius]
        box4_pos = [x_2, 0-height_ver2/2+radius, height_ver2+radius]
        box5_pos = [x_1, y_1, height_ver2+radius]
        

        self.obstacle[0].pos[:] = box1_pos
        self.obstacle[1].pos[:] = box2_pos
        self.obstacle[2].pos[:] = box3_pos
        self.obstacle[3].pos[:] = box4_pos
        self.obstacle[4].pos[:] = box5_pos
        counting = 5
        for i in range(counting):
            print("{}: {}" .format(i, self.obstacle[i].pos))

        for i in range(counting, self.num_obstacle):
            self.obstacle[i].pos[:] = box1_pos
            self.obstacle[i].pos[:][2] += (i+1-counting)*0.04
            if self.obstacle[i].pos[:][2] > height:
                counting = i
                # counting が obstacle の個数（0~counting-1）
                print("{}: {}" .format(i, self.obstacle[i].pos))
                print("{}: 却下 box1終了" .format(counting))
                break
            print("{}: {}" .format(i, self.obstacle[i].pos))

        for i in range(counting, self.num_obstacle):
            self.obstacle[i].pos[:] = box2_pos
            self.obstacle[i].pos[:][2] += (i+1-counting)*0.04
            if self.obstacle[i].pos[:][2] > height_ver2:
                counting = i
                # counting が obstacle の個数（0~counting-1）
                print("{}: {}" .format(i, self.obstacle[i].pos))
                print("{}: 却下 box2終了" .format(counting))
                break
            print("{}: {}" .format(i, self.obstacle[i].pos))

        for i in range(counting, self.num_obstacle):
            self.obstacle[i].pos[:] = box3_pos
            self.obstacle[i].pos[:][2] += (i+1-counting)*0.04
            if self.obstacle[i].pos[:][2] > height_ver2:
                counting = i
                # counting が obstacle の個数（0~counting-1）
                print("{}: {}" .format(i, self.obstacle[i].pos))
                print("{}: 却下 box3終了" .format(counting))
                break
            print("{}: {}" .format(i, self.obstacle[i].pos))
        
        for i in range(counting, self.num_obstacle):
            self.obstacle[i].pos[:] = box4_pos
            self.obstacle[i].pos[:][1] += (i+1-counting)*0.04
            if self.obstacle[i].pos[:][1] >= box4_pos[1]+height_ver2:
                counting = i
                # counting が obstacle の個数（0~counting-1）
                print("{}: {}" .format(i, self.obstacle[i].pos))
                print("{}: 却下 box4終了" .format(counting))
                break
            print("{}: {}" .format(i, self.obstacle[i].pos))
            
        for i in range(counting, self.num_obstacle):
            self.obstacle[i].pos[:] = box5_pos
            self.obstacle[i].pos[:][1] += (i+1-counting)*0.04
            if self.obstacle[i].pos[:][1] >= box5_pos[1]+height_ver3:
                counting = i
                # counting が obstacle の個数（0~counting-1）
                print("{}: {}" .format(i, self.obstacle[i].pos))
                print("{}: 却下 box5終了" .format(counting))
                break
            print("{}: {}" .format(i, self.obstacle[i].pos))

        #self.num_obstacle = counting
        print("\n0~{}: {}個の障害物\n" .format(counting-1, counting))

        result_obstacle = [Object(self.world_size, 0.0) for i in range(counting)]



        for i in range(counting):
            result_obstacle[i] = self.obstacle[i]
            #print("{}: {}" .format(i, result_obstacle[i].pos[:]))

    
        real_obstacle = result_obstacle
        return real_obstacle

sim = obstacle_pos()
#result_obs = sim.check_obs()
#print(result_obs)