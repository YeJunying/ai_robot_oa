# 地图模拟
from pathlib import Path

import numpy as np
import cv2
import matplotlib.pyplot as plt

from .map_tools    import *

import IPython

# 虚拟地图，生成特定观察点的环视图
class vmap_c:
    def __init__(self, fname='vmap.bmp',R=200,show=True):
        self.R=R                    # 最大环视探测距离
        fpath = Path(__file__).parent / fname
        self.load_map(str(fpath),show)   # 加载地图

        # 速查表，给出每个角度对应的探测射线的坐标
        self.range_det_LUT = {a:np.round(np.kron(np.array((np.cos(np.radians(a)),np.sin(np.radians(a)))),
                                                 np.arange(self.R).reshape(-1,1))).astype(int)
                              for a in range(360)}

    # 加载地图
    def load_map(self,fname, show=True):
        self.map=cv2.imread(fname)[:,:,::-1]    # 读取图像后按RGB格式存储
        self.map=(self.map.sum(axis=2)==0)*255  # 改为单色图
        self.hgt,self.wid=self.map.shape[:2]

        # 扩大图片，用于简化视觉探测效率
        self.map_ext=np.vstack((np.zeros((self.R,self.wid)), self.map, np.zeros((self.R,self.wid))))
        self.map_ext=np.hstack((np.zeros((self.hgt+self.R*2,self.R)), self.map_ext, np.zeros((self.hgt+self.R*2,self.R))))

        if show: self.show_map(title=fname)

	# 显示地图
    def show_map(self,title=''):
        plt.clf()
        plt.imshow(self.map,origin='lower',cmap='gray')
        if title: plt.title(title)
        plt.show()

    # 得到360环视距离, (x,y)是观测点坐标，rot是观测角
    def get_round_dist(self,x,y,rot=0):
        if x>=self.wid or x<0 or y>=self.hgt or y<0:
            print('[INF] error, out of range')
            return None, None
        dist=get_round_dist(self.map_ext,x+R,y+R)

        # 观察角
        rot%=360
        if rot>0: dist=np.roll(dist,-np.round(rot).astype(int))
        return dist

    # 得到环视数据，返回可显示图像
    def get_view_img(self,x,y,rot=0):
        d=self.get_round_dist(x,y,rot)
        return round_dist_to_img(d,self.R)
	
    
    # 随机环视图生成
    def random_view(self,min_dist=10):
        while True:
            x=np.random.randint(0,self.wid)
            y=np.random.randint(0,self.hgt)
            a=np.random.randint(0,360)
            dist=self.get_round_dist(x,y,a)
            if min(dist)>=min_dist:
                return dist, x, y, a


if __name__=='__main__':
    if False:
        img = cv2.imread('vmap_lab_ori.bmp')[:,:,::-1]    # 读取图像后按RGB格式存储
        img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)<200
        img = cv2.dilate(img.astype(np.uint8),np.ones((3,3)))
        img = cv2.erode(img.astype(np.uint8),np.ones((3,3)),iterations=2)
        img = cv2.dilate(img.astype(np.uint8),np.ones((3,3)))
        img = np.stack((img,img,img)).transpose((1,2,0))*255
        img = (cv2.resize(img,dsize=None,fx=3.6,fy=3.6,interpolation=cv2.INTER_LINEAR)==0)*255
        
        hgt,wid,_=img.shape
        plt.imshow(img,cmap='gray')
        plt.title(f'wid:{wid}, hgt:{hgt}')
        plt.show()
        cv2.imwrite('vmap_lab.bmp', img)
        
        exit()

    vmap=vmap_c('vmap_lab.bmp')

    # 显示特定位置环视图
    if True: 
        x,y,r=145,230,30
        plt.clf()
        plt.imshow(vmap.get_view_img(x,y,r),origin='lower')
        plt.title(f'view on ({x}, {y}), dir: {r}')
        plt.show()
