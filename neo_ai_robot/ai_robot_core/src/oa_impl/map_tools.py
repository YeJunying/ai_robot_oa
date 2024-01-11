import numpy as np
import cv2
import IPython
import matplotlib.pyplot as plt

# 角度快查表
cos_deg_lut=np.cos(np.deg2rad(range(360)))
sin_deg_lut=np.sin(np.deg2rad(range(360)))

# 射线坐标速查表
#   每一行对应一个角度的射线(x,y)坐标，射线长度R
R=200
half_line_coords=[np.array((cos_deg_lut[a]*np.arange(R),
                            sin_deg_lut[a]*np.arange(R))) for a in range(360)]

# 从栅格地图计算环视距离
def get_round_dist(gmap,cx=None,cy=None):
    h,w=gmap.shape
    if cx is None: cx=w//2
    if cy is None: cy=h//2
    
    dist =np.full(360,np.inf)
    for a,(x,y) in enumerate(half_line_coords):
        x=np.round(x+cx).astype(int)        # 从图中心出发的射线
        y=np.round(y+cy).astype(int)
        
        mask=(x>=0)*(x<w)*(y>=0)*(y<h)==1   # 屏蔽图像外的射线坐标
        sample=gmap[y[mask],x[mask]]        # 采样栅格地图
        
        d=np.nonzero(sample)[0]             # 沿射线遇到的障碍物距离
        if d.size: 
            dist[a] = min(dist[a],d[0])     # 最近障碍物的距离
    #IPython.embed()
    return dist

# 环视距离转成图像
def round_dist_to_img(dist,R=200, draw_center=True):
    gmap=round_dist_to_gmap(dist,R)
    gmap=cv2.dilate(gmap, np.ones((3,3)))
    img=gmap.repeat(3).reshape(2*R+1,2*R+1,3)*255
    
    if draw_center:
        cv2.circle(img,(int(R),int(R)),35,[255,255,0],3)
        cv2.line(img,(int(R),int(R)),(int(R+40),int(R)),[255,255,0],3)
    
    return img

# 环视距离转成栅格地图
def round_dist_to_gmap(dist,R=200):
    mask=~np.isinf(dist)
    x=np.round(cos_deg_lut[mask]*dist[mask]+R).astype(int)  # 计算扫描距离对应的图像坐标
    y=np.round(sin_deg_lut[mask]*dist[mask]+R).astype(int)
    
    mask=(x>=0)*(x<=R*2)*(y>=0)*(y<=R*2)==1                 # 滤除图像外的坐标
    coords=tuple(np.array((y[mask],x[mask])))
    
    gmap=np.zeros((2*R+1,2*R+1)).astype(np.uint8)
    gmap[coords]=1
    return gmap

