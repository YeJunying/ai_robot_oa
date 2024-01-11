# 环境模拟器

import numpy as np
import cv2
import matplotlib.pyplot as plt
import IPython

from .vmap_sim   import *
from .map_tools  import *

# 环境模拟器，保存机器人当前位置、方向以及环视图
class robo_env_sim_c:
    def __init__(self,vmap,x=0,y=0,a=0):
        self.vmap=vmap
        self.set_pos(x,y,a)
        self.get_round_dist()
    
    def set_pos(self,x=0,y=0,a=0):
        self.x,self.y,self.a=x,y,a
    
    # 将角度限定在-180～180之间
    def angle_norm(self,a):
        a%=360
        return a if a<180 else a-360
        
    # 得到机器人当前视角下的环视数据
    def get_round_dist(self, x=None, y=None, a=None):
        self.round_dist = self.vmap.get_round_dist(self.x if x is None else x,
                                                   self.y if y is None else y,
                                                   self.a if a is None else a)
        return self.round_dist

    # 移动机器人在环境中的位置
    # ds是移动步长
    # da是角度增量
    def move(self,ds=0,da=0):
        self.a =self.angle_norm(self.a+da)
        self.x+=np.cos(np.deg2rad(self.a))*ds
        self.y+=np.sin(np.deg2rad(self.a))*ds
        return self.get_round_dist()

    # 得到机器人视角的环视图片
    def get_round_view_img(self):
        return round_dist_to_img(self.round_dist,self.vmap.R)

    # 得到机器人在环境中的显示图片
    def get_env_img(self):
        print(f'[INF] coord:({self.x},{self.y})')
        print(f'[INF]   dir:{self.a}')
        img=self.vmap.map.repeat(3).reshape(*self.vmap.map.shape,3).astype(np.uint8)

        cv2.circle(img,(int(self.x),int(self.y)),35,(255,255,0),20)
        cv2.line  (img,(int(self.x),int(self.y)),(int(self.x+np.cos(np.radians(self.a))*40),int(self.y+np.sin(np.radians(self.a))*40)),(255,255,0),20)
        return img

