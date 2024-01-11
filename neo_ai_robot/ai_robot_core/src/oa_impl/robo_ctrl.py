##############################
# 近距离避障控制器
##############################
# 使用方法概述
#-----------------------------
# 1. 建立控制器对象：
#       robo_ctrl = robo_ctrl_c(cfg)
#    其中cfg是可选输入参数，如果不提供，则使用本文件里面的ROBO_CTRL_CFG_DFL作为缺省参数
#
# 2. 不断调用控制器的成员函数step得到运行的线速度ds和角速度da：
#       ds,da=robo_ctrl.step(round_dist, guide_angle)
#     其中输入参数：
#       round_dist： 360度环视障碍距离（360个距离元素和角度一一对应）
#       guide_angle：目标引导方向(相对于机器人的头朝向，单位是度)
#
#     注意：如果robo_ctrl.step(...)返回ds=da=0，则表示受阻无法移动
##############################

import numpy as np
import cv2

from .map_tools import *

##############################
# 调试开关
##############################
DEBUG_ROBO_CTRL=False

if DEBUG_ROBO_CTRL:
    import matplotlib.pyplot as plt
    import IPython

##############################
# 缺省配置
##############################
ROBO_CTRL_CFG_DFL={ 'ds'                   : 10,       # 移动线速度（cm/秒）
                    'local_range'          : 200,      # 局部避障视线关注距离（单位是cm，移动线速度越快，视线关注距离应该更大）
                    'collision_dist'       : 35,       # 最近碰撞距离（和车身物理尺寸相关）
                    'dir_keep'             : 30,       # 维持前进方向的门限（降低该门限会导致频繁掉头）
                    'dir_adj'              : 0,        # 为远离障碍的最大允许角度修改余量
                    'dist_mask_threshold'  : 20,       # 干扰点屏蔽距离（小于该门限的点被当成干扰消去）
                    'verbose'              : False     # 是否显示调试信息
                    }

##############################    
# 机器人近距离避障控制器
##############################
class robo_ctrl_c:
    def __init__(self, cfg=ROBO_CTRL_CFG_DFL):        
        # 控制参数
        self.ds                   = int(round(cfg['ds']))
        self.local_range          = int(round(cfg['local_range']))
        self.collision_dist       = int(round(cfg['collision_dist']))
        self.dist_mask_threshold  = int(round(cfg['dist_mask_threshold']))
        self.dir_keep             = int(round(cfg['dir_keep']))
        self.dir_adj              = int(round(cfg['dir_adj']))
        self.verbose              = int(round(cfg['verbose']))
        
        self.mode='go'
        self.time=0

    # 强行改变执行器状态
    def set_state(self,s='go'):
        self.mode=s

    ##############################
    # 计算移动的线速度和角速度
    # 输入：
    #   round_dist： 360度环视障碍距离（360个距离元素和角度一一对应）
    #   guide_angle：目标引导方向(相对于机器人的头朝向，单位是度)
    # 输出：
    #   ds：移动线速度
    #   da：移动角速度
    # 备注：
    #   ds=da=0表示受阻无法移动
    ##############################
    def step(self, round_dist, guide_angle=0.0,time_stamp=None):
        if self.verbose: 
            print(f'[INF] minimal distance: {np.min(round_dist)}')
            print(f'[INF] guide_angle: {guide_angle}')
            
        # 数据预处理，屏蔽干扰点云
        round_dist[round_dist<self.dist_mask_threshold]=np.inf
        
        # 计算前进角度和距离
        if self.mode=='go':
            da=self.find_path(round_dist, guide_angle)                  # 核心避障算法
            self.time=self.time+1 if time_stamp is None else time_stamp # 时间标签更新
            
            if da is None: 
                if self.verbose: print(f'[WRN] ########## stop at barrier')
                # self.mode='block' 不主动停止执行
                return (0,0,self.time)
            
            if self.verbose: print(f'[INF] return da: {da}, ds:{self.ds}')
            return (self.ds,da,self.time)

        if self.verbose: print(f'[WRN] ########## unknown state, stoppedr')
        return (0,0,self.time)

    ##############################
    # 基于可通行区域的避障
    #   根据障碍以及车身尺寸，技术和目标方位最接近的可通行方向角作为执行方向
    #   当有2个可通行方向时，优先选择和之前行进方向一致的方向
    # 输入：
    #   round_dist： 360度环视障碍距离（360个距离元素和角度一一对应）
    #   guide_angle：目标引导方向(相对于机器人的头朝向，单位是度)
    # 输出：
    #   da：机器人转角（单位是度）
    ##############################
    def find_path(self, round_dist, guide_angle=0):
        R=self.local_range                                              # 避障期间的局部视线距离
        ds=self.ds                                                      # 最大移动距离
        dmin=self.collision_dist
        
        # 距离信息转成本地栅格地图
        gmap=round_dist_to_gmap(round_dist,R=R) 
        gmap0=gmap.copy()                                               # 保存未加保护距离的栅格地图
        
        if DEBUG_ROBO_CTRL: 
            pass#IPython.embed()
        
        # 障碍加粗，屏蔽狭窄区域
        ker=cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dmin*2,dmin*2))
        gmap = cv2.dilate(gmap,ker)
        (gmap[R-ds//2:R+ds//2+1,:])[:,R-ds//2:R+ds//2+1]=0
        dist_map = cv2.distanceTransform(1-gmap, distanceType=cv2.DIST_L2, maskSize=5)  # 扣除碰撞距离后，各个空间点到障碍的距离
    
        # 计算可通行区域的环视距离
        dist=get_round_dist(gmap,R,R)
        
        # 车体已经在碰撞距离内，需要脱离碰撞区域
        if (dist>dmin).sum()==0:
            # 重新计算距离图，基于未加碰撞保护区的栅格地图
            dist_map = cv2.distanceTransform(1-gmap0, distanceType=cv2.DIST_L2, maskSize=5)  
            #if dist_map[R,R+dmin]>dist_map[R,R]: return 0               # 当前方向能够脱离碰撞区

            # 寻找脱离碰撞区的移动方向
            a=np.deg2rad(np.arange(-180,180))
            coord=tuple((np.array((np.sin(a),np.cos(a)))*self.ds).astype(int)+R)
            dist_step=dist_map[coord]
            da_pos= np.argmax(dist_step[180:])                          # 正角度方向逃离角
            da_neg=-np.argmax((dist_step[:180])[::-1])                  # 负角度方向逃离角
            da=da_pos if dist_step[da_pos+180]>dist_step[da_neg+180] else da_neg
            da%=360
            if DEBUG_ROBO_CTRL: IPython.embed()
            return da-360 if da>180 else da

        # 直接可达的情况
        if dist[int(round(guide_angle)) % 360]>R:
            da=guide_angle
            if self.dir_adj>0:
                pass    # FIXME! 加入角度余量
            da%=360
            return da-360 if da>180 else da
                                           
        # 不直接可达，需要转角
        dist0=np.roll(dist,-int(round(guide_angle)))                    # 对齐目标方向
        mark_inf=np.isinf(dist0)                                        # 视角范围内无障碍区域标志
        # IPython.embed()               
        dist0[mark_inf]=1e12                                            # 避免warning
        dist0=np.hstack((dist0,dist0[0]))   
        diff_dist0=np.diff(dist0)   
        mark_gap_pos=diff_dist0>dmin                                    # 可通行间隙标志
        mark_gap_pos[180:]=0    
        mark_gap_neg=diff_dist0<-dmin   
        mark_gap_neg[:180]=0    
        mark_gap=mark_gap_pos+mark_gap_neg  
            
        angle=np.nonzero(mark_inf+mark_gap)[0]                          # 所有可逃离障碍的方位角
        if not angle.size:  
            if DEBUG_ROBO_CTRL: IPython.embed() 
            return None                                                 # 无法前进
        angle_pos=angle[ 0]                                             # 相对目标向的正角度方向逃离角
        angle_neg=angle[-1]-360                                         # 相对目标向的负角度方向逃离角
        
        # 优先选择和之前角度接近的
        sel_pos=np.cos(np.deg2rad(angle_pos+guide_angle))
        sel_neg=np.cos(np.deg2rad(angle_neg+guide_angle))
        if max(sel_pos,sel_neg)>np.cos(np.deg2rad(self.dir_keep)):      # 待选角和之前移动方向基本一致？
            angle_sel=angle_pos if sel_pos>sel_neg else angle_neg
        else:   # 选择尽量朝向目标的逃离角
            angle_sel=angle_pos if abs(angle_pos)<abs(angle_neg) else angle_neg                                     

        # 相对目标位置的转角转换成相对机器人头朝向的转角
        da=angle_sel+guide_angle
        
        # 检查移动后距离边界距离，增加角度冗余量
        if self.dir_adj>0:
            a=np.deg2rad(da+np.arange(self.dir_adj)) if angle_sel==angle_pos else np.deg2rad(da-np.arange(self.dir_adj))    # 加不同角度偏转
            coord=tuple((np.array((np.sin(a),np.cos(a)))*self.ds).astype(int)+R)
            dist_step=dist_map[coord]                                       # 加额外的角度偏转后，行进方向离障碍的距离（扣除碰撞距离）
            if (dist_step>ds/2).sum()>0:                                    # 偏转区域内已经有满足余量的方向了
                dda=np.nonzero(dist_step)[0][0]                             # 找出满足余量的最小偏转
            else:
                dda=np.argmax(dist_step)                                    # 找出余量最大的偏转方向
            da+=dda if angle_sel==angle_pos else -dda     # 角度修正
        
        # 角度转成：180～180
        da%=360
        if da>180: da-=360
        
        if DEBUG_ROBO_CTRL: 
            print(f'[INF] guide_angle:{guide_angle}\n      angle_pos:{angle_pos}\n      angle_neg:{angle_neg}\n      angle_sel:{angle_sel}\n      da:{da}')
            gmap_comb=np.zeros((2*R+1,2*R+1,3),dtype=np.uint8)
            gmap_comb[:,:,1]=gmap *128
            gmap_comb[:,:,0]=gmap0*255
            cv2.line(gmap_comb,(R,R),(R+int(np.cos(np.radians(guide_angle))*R)          ,R+int(int(np.sin(np.radians(guide_angle))*R)))          ,(255,255,255)  ,2)
            cv2.line(gmap_comb,(R,R),(R+int(np.cos(np.radians(angle_pos+guide_angle))*R),R+int(int(np.sin(np.radians(angle_pos+guide_angle))*R))),(0,0,255)      ,5)
            cv2.line(gmap_comb,(R,R),(R+int(np.cos(np.radians(angle_neg+guide_angle))*R),R+int(int(np.sin(np.radians(angle_neg+guide_angle))*R))),(0,0,255)      ,5)
            cv2.line(gmap_comb,(R,R),(R+int(np.cos(np.radians(da))*R)                   ,R+int(int(np.sin(np.radians(da))*R)))                   ,(255,255,0)    ,2)
    
            plt.clf()
            plt.subplot(1,2,1)
            plt.plot(np.roll(dist,-round(guide_angle)))
            plt.plot(mark_gap*100)
            plt.plot(mark_inf*100)
            plt.grid(True)
            plt.subplot(1,2,2)
            plt.imshow(gmap_comb,origin='lower')
            plt.grid(True)
            plt.show()
            IPython.embed()

        return da


