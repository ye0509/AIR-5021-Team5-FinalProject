#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import math
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy


class MoveItCartesianDemo:
    def __init__(self):
        # 初始化 move_group 的 API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化 ROS 节点
        rospy.init_node('moveit_cartesian_demo', anonymous=True)

        # 初始化机械臂控制组
        arm = MoveGroupCommander('sagittarius_arm')

        # 设置运动规划参数
        arm.allow_replanning(True)
        arm.set_pose_reference_frame('world')
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)

        # 获取末端执行器link名称
        end_effector_link = arm.get_end_effector_link()

        # 回到初始位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)

        # 获取起始位姿
        start_pose = arm.get_current_pose(end_effector_link).pose
        print(start_pose)

        # 初始化路径点列表
        waypoints = []
        
        # ========== 核心修改部分 ==========
        # 圆形轨迹参数
        radius = 0.1   # 圆形半径（米）
        num_points = 30  # 轨迹点数
        center_x = start_pose.position.x
        center_y = start_pose.position.y
        center_z = start_pose.position.z  # 保持初始高度

        # 生成圆形轨迹点
        for i in range(num_points):
            theta = 2 * math.pi * i / num_points
            wpose = deepcopy(start_pose)
            wpose.position.x = center_x + radius * math.cos(theta)
            wpose.position.y = center_y + radius * math.sin(theta)
            waypoints.append(deepcopy(wpose))
            
        # 闭合圆形轨迹（添加起始点）
        waypoints.append(deepcopy(start_pose))
        # ========== 修改结束 ==========

        # 路径规划参数
        fraction = 0.0
        maxtries = 100
        attempts = 0

        # 开始路径规划
        arm.set_start_state_to_current_state()
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = arm.compute_cartesian_path(
                waypoints,
                0.01,   # 终端步进值（米）
                0.0,    # 跳跃阈值
                True    # 避障规划
            )
            attempts += 1
            if attempts % 10 == 0:
                rospy.loginfo(f"规划尝试次数：{attempts}，进度：{fraction*100:.1f}%")

        # 执行运动
        if fraction == 1.0:
            rospy.loginfo("路径规划成功，开始执行")
            arm.execute(plan)
        else:
            rospy.loginfo(f"规划失败，最终进度：{fraction*100:.1f}%")

        # 返回休眠位置
        rospy.sleep(1)
        arm.set_named_target('sleep')
        arm.go()

        # 关闭接口
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass