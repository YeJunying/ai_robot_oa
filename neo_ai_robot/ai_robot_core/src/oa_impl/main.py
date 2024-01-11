﻿import numpy as np
import cv2
import matplotlib.pyplot as plt
import IPython

from .vmap_sim     import *
from .robo_env_sim import *
from .robo_ctrl    import *

np.random.seed(1234)

# 地图模拟器对象
vmap=vmap_c(fname='vmap_lab.bmp', show=False)

# 机器人环境模拟器，以及机器人地点和目的地设置
robo_env=robo_env_sim_c(vmap)

########### 测试用例选择 ###########
TEST_CASE=16
if TEST_CASE==0:
    robo_env.set_pos(410*3.6,560*3.6,90)
    target_x,target_y=500*3.6,70*3.6
if TEST_CASE==1:
    robo_env.set_pos(410*3.6,560*3.6,90)
    target_x,target_y=420*3.6,70*3.6
if TEST_CASE==2:
    robo_env.set_pos(515*3.6,270*3.6,90)
    target_x,target_y=60*3.6,150*3.6
if TEST_CASE==3:
    robo_env.set_pos(515*3.6,270*3.6,90)
    target_x,target_y=350*3.6,100*3.6
if TEST_CASE==4:
    robo_env.set_pos(50*3.6,780*3.6,90) 
    target_x,target_y=280*3.6,135*3.6
if TEST_CASE==5:
    robo_env.set_pos(515*3.6,270*3.6,90)    # north-west
    target_x,target_y=50*3.6,780*3.6
if TEST_CASE==6:
    robo_env.set_pos(50*3.6,780*3.6,90)     # south-east
    target_x,target_y=650*3.6,100*3.6
if TEST_CASE==7:
    robo_env.set_pos(100*3.6,675*3.6,90)
    target_x,target_y=70*3.6,215*3.6
if TEST_CASE==8:
    robo_env.set_pos(420*3.6,520*3.6,-90)
    target_x,target_y=500*3.6,70*3.6
if TEST_CASE==9:
    robo_env.set_pos(1120,740,-165)
    target_x,target_y=60*3.6,150*3.6
if TEST_CASE==10:
    robo_env.set_pos(1659,1237,5)
    target_x,target_y=420*3.6,70*3.6
if TEST_CASE==11:
    robo_env.set_pos(200,2500,90)
    target_x,target_y=490,215
if TEST_CASE==12:
    robo_env.set_pos(1445,2253,90)  # too close to wall
    target_x,target_y=500*3.6,70*3.6
if TEST_CASE==13:    
    robo_env.set_pos(200,2500,90)   # trap by room
    target_x,target_y=1000,250 
if TEST_CASE==14:                   # trap in room
    robo_env.set_pos(200,2500,90)
    target_x,target_y=1488,226    
if TEST_CASE==15:
    robo_env.set_pos(550,1000,90)
    target_x,target_y=550,1500    
if TEST_CASE==16:
    robo_env.set_pos(2000,900,0)
    target_x,target_y=550,900


# 地图、机器人初始位置、目的地显示
plt.clf()
img=robo_env.get_env_img()

cv2.circle(img,(int(target_x),int(target_y)),35,(0,255,0),20)
cv2.line(img,(int(robo_env.x),int(robo_env.y)),(int(target_x),int(target_y)),(0,255,0),20)
plt.imshow(img, origin='lower')
plt.title(f'test case: {TEST_CASE}') 
plt.show()
    
# 机器人控制器对象
robo_ctrl = robo_ctrl_c()

# 机器人移动循环
print('[INF] run')
PLOT_STEP=0
trace=[]
for t in range(500):
    
    # 计算目标引导方向guide_angle(相对于机器人的头朝向，单位是度)
    dx, dy= target_x-robo_env.x,target_y-robo_env.y
    ta=np.arctan2(dy,dx)-np.deg2rad(robo_env.a)                         
    guide_angle=np.rad2deg(ta)                                          
    print(f'[INF] robo_env.x:{robo_env.x}, robo_env.y:{robo_env.y}')
    print(f'[INF] target_x:{target_x}, target_y:{target_y}')
    print(f'[INF] dx:{dx}, dy:{dy}, guide_angle:{guide_angle}')
    
    # 取得360度环视障碍距离round_dist（360个距离元素和角度一一对应）
    round_dist=robo_env.get_round_dist()
    
    # 距离测量值加扰
    round_dist+=np.random.randint(-2,2,len(round_dist))
    
    # 是否到达终点
    if (dx*dx+dy*dy<80**2): 
        break  # 到达终点
    else:
        trace.append((robo_env.x,robo_env.y,np.min(round_dist)))
        
    # 计算线速度ds和角速度da
    ds,da,*_=robo_ctrl.step(round_dist, guide_angle)

    # 移动加扰
    ds*=(1.0+(np.random.rand()-0.5)*0.5)
    da+=np.random.randint(-3,3)
    
    # 机器人移动
    robo_env.move(ds,da)
    
    if PLOT_STEP>0:
        if t%PLOT_STEP==0:
            # 取得全局俯视图
            img_env =robo_env.get_env_img()
            cv2.circle(img_env,(int(target_x),int(target_y)),35,(0,255,0),10)   # 绘制目标
            
            # 取得机器人局部环视图
            img_round_view=robo_env.get_round_view_img()
            h,w=img_round_view.shape[:2]
            cv2.line  (img_round_view,                                          # 绘制导向线
                       (w//2,h//2),
                       (np.round(np.cos(ta)*40+w//2).astype(int),np.round(np.sin(ta)*40+h//2).astype(int)),
                       (0,255,0),3)
            plt.clf()
            plt.subplot(1,2,1)
            plt.imshow(img_env, origin='lower')
            plt.subplot(1,2,2)
            plt.imshow(img_round_view, origin='lower')
            plt.title(f'coord:({int(robo_env.x)},{int(robo_env.y)}), dir:{int(robo_env.a)}')
            
            if False:
                plt.show()
            else:
                #plt.show()
                #IPython.embed()
                plt.show(block=False)
                plt.pause(1e-6)

# 绘制移动轨迹
img_env =robo_env.get_env_img()
cv2.circle(img_env,(int(target_x),int(target_y)),35,(0,255,0),10)       # 绘制目标
for (x1,y1,_),(x2,y2,_) in zip(trace[:-1],trace[1:]):
    cv2.line(img_env,(int(x1),int(y1)),(int(x2),int(y2)),(255,255,0),20)# 绘制轨迹
plt.clf()
plt.subplot(1,2,1)
plt.imshow(img_env, origin='lower')
plt.title(f'trace of test case: {TEST_CASE}') 
plt.subplot(1,2,2)
plt.plot([d for _,_,d in trace])
plt.plot(np.full(len(trace),robo_ctrl.collision_dist))
plt.title(f'minimal distance in test case: {TEST_CASE}') 
plt.show(block=True)
