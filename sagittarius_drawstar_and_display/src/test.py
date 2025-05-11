#!/usr/bin/env python

# -*- coding: utf-8 -*-


import rospy

import sys

import moveit_commander

from moveit_commander import MoveGroupCommander, PlanningSceneInterface

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

from copy import deepcopy


class HanoiSolver:

    def __init__(self):

        # 初始化MoveIt!

        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('hanoi_solver', anonymous=True)

        

        # 初始化机械臂、夹爪和规划场景

        self.arm = MoveGroupCommander('sagittarius_arm')

        self.gripper = MoveGroupCommander('sagittarius_gripper')

        self.scene = PlanningSceneInterface()

        

        # 设置汉诺塔参数

        self._setup_parameters()

        self._setup_environment()

        

        # 初始化柱子状态，盘子从大到小堆叠在源柱上

        self.peg_states = {

            'source': [2, 1, 0],  # 2为最大盘，0为最小盘

            'aux': [],

            'target': []

        }

        

    def _setup_parameters(self):

        # 机械臂参数

        self.arm.set_pose_reference_frame('world')

        self.arm.set_max_velocity_scaling_factor(0.3)

        self.arm.set_planning_time(5)

        

        # 定义三根柱子的坐标（source 最左，aux 中间，target 最右，底部高度为0）

        self.pegs = {

            'source': Pose(Point(0.3, 0.2, 0.0), Quaternion(0,0,0,1)),  # 最左

            'aux':    Pose(Point(0.3, 0.0, 0.0), Quaternion(0,0,0,1)),  # 中间

            'target': Pose(Point(0.3, -0.2, 0.0), Quaternion(0,0,0,1))   # 最右

        }

        

        # 盘子尺寸（长宽高均为0.05m）

        self.disk_size = 0.05

        self.disk_thickness = self.disk_size  # 盘子高度

        

    def _setup_environment(self):

        # 添加柱子到规划场景

        for peg in ['source', 'aux', 'target']:

            pose_stamped = PoseStamped()

            pose_stamped.header.frame_id = 'world'

            pose_stamped.pose = self.pegs[peg]

            self.scene.add_box(

                name=peg,

                pose=pose_stamped,

                size=(0.02, 0.02, 0.15)  # 柱子尺寸

            )

    

    def _move_to_peg(self, peg_name, z_height):

        target_pose = deepcopy(self.pegs[peg_name])

        target_pose.position.z = z_height

        self.arm.set_pose_target(target_pose)

        return self.arm.go()

    

    def _transfer_disk(self, src, dest):

        if not self.peg_states[src]:

            rospy.logerr(f"No disk to move from {src}")

            return False

        

        # 获取源柱子最顶层的盘子

        disk_level = self.peg_states[src][-1]

        

        # 检查汉诺塔规则：不能将大盘子放在小盘子上

        if self.peg_states[dest] and self.peg_states[dest][-1] < disk_level:

            rospy.logerr(f"Cannot move disk {disk_level} onto smaller disk {self.peg_states[dest][-1]} on {dest}")

            return False

        

        # 计算源柱子上最顶层盘子的 z 高度（盘子中心）

        num_disks_src = len(self.peg_states[src])

        z_src = 0.0 + (num_disks_src - 0.5) * self.disk_thickness

        

        # 夹取动作：直接到达盘子中心，打开夹爪，再闭合夹爪

        self._move_to_peg(src, z_src)  # 直接降到盘子中心

        self._control_gripper('open')  # 打开夹爪

        rospy.sleep(0.5)  # 等待夹爪动作完成

        self._control_gripper('close')  # 闭合夹爪

        rospy.sleep(0.5)  # 等待夹爪动作完成

        

        # 离开源柱子，竖直移动到 z=0.35

        self._move_to_peg(src, 0.35)

        

        # 计算目标柱子上放置盘子的 z 高度（盘子中心）

        num_disks_dest = len(self.peg_states[dest])

        z_dest = 0.0 + (num_disks_dest + 0.5) * self.disk_thickness

        

        # 水平移动到目标柱子上方 (x_dest, y_dest, 0.35)

        target_pose = deepcopy(self.pegs[dest])

        target_pose.position.z = 0.35

        self.arm.set_pose_target(target_pose)

        self.arm.go()

        

        # 放置动作：降到放置位置，打开夹爪

        self._move_to_peg(dest, z_dest)  # 降到放置位置

        self._control_gripper('open')  # 打开夹爪释放盘子

        rospy.sleep(0.5)  # 等待夹爪动作完成

        

        # 离开目标柱子，竖直移动到 z=0.35

        self._move_to_peg(dest, 0.35)

        

        # 更新柱子状态

        self.peg_states[src].pop()

        self.peg_states[dest].append(disk_level)

        

        return True

    

    def _control_gripper(self, state):

        self.gripper.set_named_target(state)

        self.gripper.go()

        

    def solve_hanoi(self):

        # 三层汉诺塔的七步移动序列

        moves = [

            ('source', 'target'),  # 移动最小盘

            ('source', 'aux'),     # 移动中间盘

            ('target', 'aux'),     # 移动最小盘

            ('source', 'target'),  # 移动最大盘

            ('aux', 'source'),     # 移动最小盘

            ('aux', 'target'),     # 移动中间盘

            ('source', 'target'),  # 移动最小盘

        ]

        

        try:

            self.arm.set_named_target('sleep')

            self.arm.go()

            

            for src, dest in moves:

                print(f"正在移动盘子: {src} -> {dest}")

                self._transfer_disk(src, dest)

                

            print("汉诺塔任务完成！")

            

        except Exception as e:

            print(f"操作失败: {str(e)}")

        finally:

            moveit_commander.roscpp_shutdown()


if __name__ == "__main__":

    try:

        solver = HanoiSolver()

        solver.solve_hanoi()

    except rospy.ROSInterruptException:

        pass