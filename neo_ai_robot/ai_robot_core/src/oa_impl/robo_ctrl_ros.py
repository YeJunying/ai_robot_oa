import math

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion

from ai_robot_msgs.msg import OAVelocity, ai_robot_msgs
from ai_robot_msgs.srv import SetRobotPose, SetRobotPoseResponse, SetTargetPosition, SetTargetPositionResponse 

from .robo_ctrl import robo_ctrl_c
from .geo_tools import scan_data_to_round_dist


class RoboCtrlRos:
    def __init__(self):
        # pose
        self.initial_x = 0.0  # initial position x in map frame
        self.initial_y = 0.0  # initial position y in map frame
        self.initial_orient = 0.0  # initial orientation in map frame
        self.x = 0.0  # current position x in map frame
        self.y = 0.0  # current position y in map frame
        self.orient = 0.0  # orientation in map frame
        self.yaw = 0.0
        self.dyaw = 0.0  # orientation change
        # self.hdg = 0.0  # magetic orientation

        # target position
        self.tgt_x = None
        self.tgt_y = None

        # control flow
        self.pose_intialized = False
        self.ready = False

        # config
        self.sim_mode = False
        self.use_mag = False
        self.angle_nwu_to_map = 0.0
        self.cfg = {}
        self.get_params()
        self.controller = robo_ctrl_c(self.cfg)

        # navigation
        self.guide_angle = 0.0
        self.step_flag = False

        # sensors
        self.scan_data = None
        self.vel_msg = OAVelocity()

        # ros ipc
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_cb)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        if self.use_mag:
            self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_cb)
        self.vel_pub = rospy.Publisher("/oa_vel", OAVelocity, queue_size=1)
        self.pose_srv = rospy.Service( "/oa_robot_pose", SetRobotPose, self.set_pose)
        self.tgt_srv = rospy.Service( "/oa_target_pos", SetTargetPosition, self.set_target_position)
        # self.timer = rospy.Timer(rospy.Duration(1.0), self.compute_velocity)
        self.tfl = tf.TransformListener()

    def set_pose(self, req):
        self._set_pose(req.x, req.y, req.theta)
        return SetRobotPoseResponse()

    def set_target_position(self, req):
        self._set_target_position(req.x, req.y)
        return SetTargetPositionResponse()

    def _set_pose(self, x, y, orient):
        self.initial_x, self.initial_y = x, y
        self.yaw = self.initial_orient = orient
        self.pose_intialized = True

    def _set_target_position(self, x, y):
        self.tgt_x, self.tgt_y = x, y
        self._initialize_guide_angle()

    def _initialize_guide_angle(self):
        self.guide_angle = (
            math.atan2(self.tgt_y - self.initial_y, self.tgt_x - self.initial_x)
            - self.initial_orient
        )
        self.ready = True

    def _update_guide_angle(self):
        # dtheta = self.guide_angle - math.degrees(self.dyaw)
        # self.guide_angle = dtheta / math.fabs(dtheta) * (math.fabs(dtheta) % 360)
        self.guide_angle -= self.dyaw
        if self.cfg['verbose']:
            rospy.loginfo("guide_angle: %f", math.degrees(self.guide_angle))

    # def set_initial_data(self, req):
    #     self.guide_angle = req.guide_angle
    #     self.x = req.x
    #     self.y = req.y
    #     self.step_flag = True
    #     return SetGuideAngleResponse()

    def get_params(self):
        """Get settings from ROS parameter server. Currently the following parameters
        are supported:

        - ds: an integer representing the linear velocity (cm/s)
        - local_range: an integer representing the detection range (cm)
        - collision_dist: an integer representing the minimum gap kept between the robot
            and the obstale (cm)
        - dir_keep: an integer representing the minimum direction angle to keep (degrees)
        - dir_adj: an interger representing the maximum adjustment for the angle to long
          distance obstacle (degrees)
        - dist_mask_threshold: an integer representing the distance within which all
          obstacle points are ignored
        - use_mag: a boolean indicating whether to use magnetometer for orientation
        - angle_nwu_to_map: an angle from NMU frame (imu magnetometer) to map frame (degrees)
        - verbose: a boolean for switching on/off debugging output
        """
        cfg = self.cfg
        cfg["ds"] = rospy.get_param("~ds")
        cfg["local_range"] = rospy.get_param("~local_range")
        cfg["collision_dist"] = rospy.get_param("~collision_dist")
        cfg["dir_keep"] = rospy.get_param("~dir_keep")
        cfg["dir_adj"] = rospy.get_param("~dir_adj")
        cfg["dist_mask_threshold"] = rospy.get_param("~dist_mask_threshold")
        cfg["verbose"] = rospy.get_param("~verbose")
        self.sim_mode = rospy.get_param("~sim")
        self.use_mag = rospy.get_param("~use_mag")
        self.angle_nwu_to_map = math.radians(rospy.get_param("~angle_nwu_to_map"))

    # def downsample_laserscan(self, seq, res, n=360):
    #     def get_min(i):
    #         rem = i % res
    #         j = round(i / res)
    #         return min(seq[j], seq[j+1]) if rem != 0 else seq[j]
    #
    #     return np.asarray([get_min(i) for i in range(n)])

    def lidar_cb(self, msg):
        self.scan_data = msg.ranges

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # rospy.loginfo("cur_yaw: %f, old_yaw: %f", math.degrees(yaw), math.degrees(self.yaw))
        self.dyaw = yaw - self.yaw  # [-360, 360]
        self.yaw = yaw

        # pose_odom = PoseStamped()
        # pose_odom.header = msg.header
        # pose_odom.pose = msg.pose.pose
        # tfl = self.tfl
        # try:
        #     tfl.waitForTransform("map", "odom", rospy.Time(0), rospy.Duration(5))
        #     pose_map = tfl.transformPose("map", pose_odom)
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        #     # rospy.logerr("Transforming from odom pose to map pose failed")
        #     raise(e)
        # else:
        #     self.x = pose_map.pose.position.x
        #     self.y = pose_map.pose.position.y
        #     q = pose_map.pose.orientation
        #     _, _, self.orient = euler_from_quaternion((q.x, q.y, q.z, q.w))

        self._update_guide_angle()

    def imu_cb(self, msg):
        q = msg.orientation  # NWU coordinate system (North is x axis, West is y axis)
        _, _, hdg = euler_from_quaternion(
            (q.x, q.y, q.z, q.w)
        )  # absolute imu magnetic heading
        orient = hdg - self.angle_nwu_to_map
        self.dyaw = orient - self.orient
        self.orient = orient

    def compute_velocity(self):
        if not self.ready or self.scan_data is None:
            return
        round_dist = scan_data_to_round_dist(self.scan_data, len(self.scan_data))
        # vx, rz, ts = self.controller.step(round_dist, math.degrees(self.guide_angle))
        guide_angle = math.degrees(self.guide_angle)
        print(f"yaw: {self.yaw}, dyaw: {self.dyaw}, guide_angle: {guide_angle}") 
        vx, rz, ts = self.controller.step(round_dist, guide_angle)
        vel_msg = self.vel_msg
        vel_msg.vx = vx  # centermeters
        vel_msg.rz = rz  # degrees
        # vel_msg.rz = math.radians(rz)
        self.vel_pub.publish(self.vel_msg)

    def publish_velocity(self):
        self.vel_pub.publish(self.vel_msg)


if __name__ == "__main__":
    rospy.init_node("robo_ctrl_ros")
    robot = RoboCtrlRos()
    # rospy.spin()
    while not rospy.is_shutdown():
        robot.compute_velocity()
        rospy.sleep(1)
