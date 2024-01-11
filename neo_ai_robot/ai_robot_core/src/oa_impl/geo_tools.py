# 几何计算工具
import numpy as np

import matplotlib.pyplot as plt
import cv2
import IPython

##############################
#   计算直线模型参数：p=dt+m
# 返回
#   d：  方向
#   m：  直线上一点
##############################
def det_line_param(pts):
    m=np.mean(pts,axis=0)                                               # 直线上一点（点云中心）
    pts0=pts-m                                                          # 去均值
    C=np.dot(pts0.T,pts0)                                               # 协方差阵
    E,V=np.linalg.eig(C)                                                # E: 特征值, V: 特征向量
    idx=np.argmax(E)                                                    # 排序
    d=V[:,idx].ravel()                                                  # 最大特征值(对应直线方向)
    return d,m

##############################
# 计算点集pts到直线(p1,p2)的距离
##############################
def pts_to_line_dist(p1,p2,pts):
    S=0.5*np.abs(p1[0]*p2[1]+p2[0]*pts[:,1]+pts[:,0]*p1[1]-p1[0]*pts[:,1]-p2[0]*p1[1]-pts[:,0]*p2[1])   # 三角形面积
    return S/np.linalg.norm(p1-p2)

##############################
# 计算点v在p1-p2直线(及其延长线)上的投影坐标
##############################
def pts_to_line_proj(v,p1,p2):
    d=p1-p2
    m=(p1+p2)*0.5
    return m+np.sum((v-m)*d)*d/np.sum(d**2)

##############################
# 直线搜索
# 输入：
#   pts：点集，每一行是一个点的x/y坐标
#   K：判定直线的点数下限
#   h：判定点在直线上的距离下限（单位是cm）
#   d：用于搜寻直线的两个端点距离下限（单位是cm）
#   it_max：迭代检测次数上限
# 返回：
#   res列表，每行是直线参数(p1,p2,n)，其中
#       p1,p2为构成直线的两个端点
#       n是pts中，在p1-p2直线（及其延长线）上的点的数目
##############################
def det_line(pts, K=50, h=2, d=50, it_max=500):        
    res=[]
    p3=pts.copy()
    for it in range(it_max):        
        if len(p3)<K+2: break                                           # 点过少
        
        # 抽取2个随机点(p1,p2)作为待定直线端点
        p1,p2=p3[np.random.randint(low=0, high=len(p3), size=2)]
        if np.linalg.norm(p1-p2)<d: continue                            # 滤除距离过近的端点对

        # 计算所有点到直线(p1,p2)的距离，并标识出到直线距离小于h的点
        sel=pts_to_line_dist(p1,p2,p3)<h
        
        # 直线判定
        if (sel.sum()>K+2):                                             # 检出直线的点数条件（扣除作为直线端点的两点)         
            pd,p0=det_line_param(p3[sel])                               # 收集直线上的所有点，重新计算直线参数
            res.append((p0+pd,p0-pd,sel.sum()-2))                       # 保存直线参数
            p3=p3[~sel]                                                 # 删除直线上的点
    print('[INF] it: ',it)
    return res


##############################
# 得到边墙的直线参数
# 输入：
#   min_line_dist   允许的最近距离直线（单位是cm, 更近距离的直线将被滤除）
# 返回：dict，里面元素是
#   'left' : (直线到原点距离，由原点指向直线的法向量方向角)
#   'right': (距离，方向角)
#   'up'   : (距离，方向角)
#   'down' : (距离，方向角)
# 注意：角度单位是度，距离单位是cm
##############################
def get_walls(line_param, min_line_dist=5):    
    # 计算原点在每条直线上的投影
    proj_pts=[pts_to_line_proj((0,0),p1,p2) for p1,p2,_ in line_param]
    
    # 滤除太近距离的直线
    proj_pts=[p for p in proj_pts if np.linalg.norm(p)>min_line_dist]
    
    # 根据投影位置分为 |x|>|y|:    x>0 right, x<0 left
    #                 |y|>|x|:    y>0 up   , y<0 down
    proj_pts_partion={'up':[], 'down':[], 'left':[], 'right':[]}
    for x,y in proj_pts:
        if abs(x)>abs(y): 
            proj_pts_partion['right' if x>0 else 'left'].append((x,y))
        else:
            proj_pts_partion['up'    if y>0 else 'down' ].append((x,y))

    # 计算每个集合中距离原点最近的直线，给出其距离和法向量角度
    res={}
    for key,pts in proj_pts_partion.items():
        if len(pts)>0:
            x,y=pts[0] if len(pts)==1 else \
                pts[np.argmin([np.linalg.norm(p) for p in pts])]
            res[key]=(np.sqrt(x*x+y*y),
                      np.rad2deg(np.arctan2(y,x)))
    return res

##############################
# 把Lidar扫描数据转成0-360环视数据
# 输入：
#   scan    原始Lidar距离数据，单位是M，扫描角范围是-180～180
#   num     原始Lidar数据长度
# 输出：
#   dist    0～360度环视距离数据，单位是cm
##############################
def scan_data_to_round_dist(scan, num=1153):
    res=np.full(360,np.inf)
    # for a,d in zip(np.linspace(-180,180,num+1)[:-1],scan):
    for a,d in zip(np.linspace(-180,180,num),scan):
        ai=int(round(a))%360
        res[ai]=min(res[ai],d*100)
    return res

##############################
# 从环视距离数据得到四面墙的方向和距离
# 输入：
#   dist    360元素数组，对应0-359度的距离，单位是cm
#   R       探测视线距离（忽略距离大于R的点）
#   K       判定直线的点数下限
#   h       判定点在直线上的距离下限（单位是cm）
#   d       用于搜寻直线的两个端点距离下限（单位是cm）
#   min_line_dist   允许的最近距离直线（单位是cm, 更近距离的直线将被滤除）
# 返回dict，里面元素是：
#   {'left' :(距离，方向角), 'right' :(距离，方向角), 'up' :(距离，方向角), 'down' :(距离，方向角) }
#   对应4个方位墙壁的参数（未探测到的墙壁，相应的key-value不出现在字典里）
##############################
def get_walls_from_round_dist(dist, R=300, K=50, h=2, d=50, min_line_dist=5):
    # 计算半径R范围内的点坐标
    pts=np.array([(np.cos(np.deg2rad(a))*r,np.sin(np.deg2rad(a))*r) for a,r in enumerate(dist) if r<R])
    # 直线检测
    line_param=det_line(pts, K, h, d, it_max=len(dist))
    # 墙面检测
    return get_walls(line_param,min_line_dist)

##############################
# 从原始Lidar数据数据得到四面墙的方向和距离
# （参考get_walls_from_round_dist）
##############################
def get_walls_from_scan(scan, num=1153, R=300, K=50, h=2, d=50, min_line_dist=5):
    return get_walls_from_round_dist(scan_data_to_round_dist(scan, num), R, K, h, d, min_line_dist)

##############################
# 单元测试
##############################
if __name__=='__main__':
    from map_tools import *

    # 配置参数
    R=300                                                               # 探测视线距离（忽略距离大于R的点）
    K=50                                                                # 判定直线的点数下限
    h=2                                                                 # 判定点在直线上的距离下限（单位是cm）
    d=50                                                                # 用于搜寻直线的两个端点距离下限（单位是cm）
    min_line_dist=5                                                     # 允许的最近距离直线（单位是cm, 更近距离的直线将被滤除）
    NUM_ROW=1153                                                        # Lidar每轮扫描的点数
    FNAME_BIN='./Lidar/laserscan_ranges.bin'                            # 原始Lidar数据文件
    
    # 读取存储的Lidar数据
    scan_data=np.fromfile(FNAME_BIN,dtype=np.float32).reshape(-1,NUM_ROW)
    
    # 检测直线
    for n,scan in enumerate(scan_data):     
        # 获取墙面直线
        param_walls=get_walls_from_scan(scan, NUM_ROW, R, K, h, d, min_line_dist)
        print(param_walls)
        
        # 扫描数据格式转换
        dist=scan_data_to_round_dist(scan)  # 扫描数据按0-360度排列，距离单位改为cm
        
        # 生成栅格地图图像
        img=round_dist_to_img(dist,R)
        
        # 绘图并显示
        for k,(r,a) in param_walls.items():
            color_list={'left':[255,255,0], 'right':[0,255,255], 'up':[255,0,255], 'down':[255,0,0]}
            dx,dy=np.cos(np.deg2rad(a)),np.sin(np.deg2rad(a))
            cv2.line(img, (int(R),int(R)),(int(R+dx*r),int(R+dy*r)), color_list[k], 1)
            cv2.line(img, (int(R+dx*r-dy*R),int(R+dy*r+dx*R)),(int(R+dx*r+dy*R),int(R+dy*r-dx*R)),color_list[k],1)
            
        if True:
            plt.clf()
            plt.imshow(img,origin='lower')
            plt.title(str(n))
            if False:
                plt.show()
            else:
                plt.show(block=False)
                plt.pause(0.01)

        
