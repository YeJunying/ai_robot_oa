import numpy as np
import cv2
import matplotlib.pyplot as plt
# import IPython

from .vmap_sim     import *
from .robo_env_sim import *
from .robo_ctrl    import *

def run_sim():
    np.random.seed(1234)

    # 地图模拟器对象
    vmap=vmap_c(fname='vmap_lab.bmp', show=False)

    # 机器人环境模拟器，以及机器人地点和目的地设置
    robo_env=robo_env_sim_c(vmap)

    if True:  # 茶水间 
        robo_env.set_pos(580,145,180)
        target_x,target_y=220,550
    if False: # 办公室
        robo_env.set_pos(580,145,180)
        target_x,target_y=650,380
    if False: # fail
        robo_env.set_pos(580,145,180)
        target_x,target_y=450,450
    if False:
        robo_env.set_pos(410,560,90)
        target_x,target_y=500,70
    if False:
        robo_env.set_pos(515,270,90)
        target_x,target_y=60,150
    if False:
        robo_env.set_pos(515,270,90)
        target_x,target_y=280,135
    if False:
        robo_env.set_pos(50,780,90)
        target_x,target_y=280,135
    if False:
        robo_env.set_pos(515,270,90)
        target_x,target_y=50,780
    if False:
        robo_env.set_pos(50,780,90)
        #robo_env.set_pos(630,800,90)
        target_x,target_y=560,100
        
    # 地图、机器人初始位置、目的地显示
    plt.clf()
    img=robo_env.get_env_img()
    cv2.circle(img,(int(target_x),int(target_y)),10,(0,255,0),5)
    plt.imshow(img, origin='lower')
    plt.show()
        
    # 机器人控制器对象
    robo_ctrl = robo_ctrl_c()

    # 键盘事件处理函数用以退出循环
    def press(event):
        if event.key == 'q':
            print('Quit requested')
            global quit_flag
            quit_flag = True

    # 注册键盘事件处理函数
    fig, ax = plt.subplots()
    fig.canvas.mpl_connect('key_press_event', press);

    # 机器人移动循环
    print('[INF] run')
    quit_flag = False
    for _ in range(2400):
        if quit_flag:
            break
        
        # 计算目标引导方向guide_angle(相对于机器人的头朝向，单位是度)
        dx, dy= target_x - robo_env.x, target_y - robo_env.y
        ta = np.arctan2(dy,dx) - np.deg2rad(robo_env.a)
        guide_angle = np.rad2deg(ta)                                          
        
        # 取得360度环视障碍距离 round_dist（360个距离元素和角度一一对应）
        round_dist=robo_env.get_round_dist()
        
        # 计算线速度 ds 和角速度 da
        ds,da=robo_ctrl.step(round_dist, guide_angle)          
        
        # 机器人移动
        robo_env.move(ds, da)
        
        # 取得全局俯视图
        img_env =robo_env.get_env_img()
        cv2.circle(img_env, (int(target_x), int(target_y)), 10, (0,255,0), 5)
        
        # 取得机器人局部环视图
        img_round_view = robo_env.get_round_view_img()
        
        plt.clf()
        ax1 = fig.add_subplot(121)
        ax1.imshow(img_env, origin='lower')
        ax2 = fig.add_subplot(122)
        ax2.imshow(img_round_view, origin='lower')
        ax2.set_title(f'coord:({robo_env.x.astype(int)},{robo_env.y.astype(int)}), dir:{robo_env.a.astype(int)}')
        if False:
            plt.show()
        else:
            #plt.show()
            #IPython.embed()
            plt.show(block=False)
            plt.pause(0.001)
