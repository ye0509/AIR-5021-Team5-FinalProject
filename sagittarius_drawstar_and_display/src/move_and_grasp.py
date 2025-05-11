#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy

class MoveItPickPlace:
    def __init__(self):
        # 初始化API
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_pick_place', anonymous=True)

        # 初始化机械臂和夹爪控制组
        self.arm = MoveGroupCommander('sagittarius_arm')
        self.gripper = MoveGroupCommander('sagittarius_gripper')

        # 设置基本参数
        self._setup_parameters()

    def _setup_parameters(self):
        # 机械臂参数设置
        self.arm.set_pose_reference_frame('world')
        self.arm.allow_replanning(True)
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.001)
        self.arm.set_max_velocity_scaling_factor(0.5)
        self.arm.set_max_acceleration_scaling_factor(0.5)

        # 夹爪参数设置
        self.gripper.set_goal_joint_tolerance(0.001)
        self.gripper.set_max_velocity_scaling_factor(0.5)
        self.gripper.set_max_acceleration_scaling_factor(0.5)

    def _move_arm_to_pose(self, pose):
        self.arm.set_pose_target(pose)
        self.arm.go()
        rospy.sleep(1)

    def _control_gripper(self, state):
        if state == 'open':
            self.gripper.set_named_target('open')
        elif state == 'close':
            self.gripper.set_named_target('close')
        self.gripper.go()
        rospy.sleep(1)

    def _get_current_pose(self):
        return self.arm.get_current_pose().pose

    def execute_pick_place(self):
        try:
            # 初始化位置
            print("回到初始位置")
            self.arm.set_named_target('sleep')
            self.arm.go()
            rospy.sleep(1)

            # 移动到物体上方（假设物体在当前位置前0.2米，高0.1米）
            print("移动到物体上方")
            current_pose = self._get_current_pose()
            approach_pose = deepcopy(current_pose)
            approach_pose.position.x += 0.2
            approach_pose.position.z += 0.1
            self._move_arm_to_pose(approach_pose)

            # 张开夹爪
            print("张开夹爪")
            self._control_gripper('open')

            # 下降到抓取位置
            print("下降到抓取位置")
            grasp_pose = deepcopy(approach_pose)
            grasp_pose.position.z -= 0.15
            self._move_arm_to_pose(grasp_pose)

            # 闭合夹爪
            print("闭合夹爪")
            self._control_gripper('close')

            # 提升物体
            print("提升物体")
            self._move_arm_to_pose(approach_pose)

            # 移动到放置位置（假设右移0.15米）
            print("移动到放置位置上方")
            place_pose = deepcopy(approach_pose)
            place_pose.position.y += 0.15
            self._move_arm_to_pose(place_pose)

            # 下降到放置高度
            print("下降到放置高度")
            release_pose = deepcopy(place_pose)
            release_pose.position.z -= 0.10
            self._move_arm_to_pose(release_pose)

            # 张开夹爪
            print("释放物体")
            self._control_gripper('open')

            # 返回初始位置
            print("返回初始位置")
            self.arm.set_named_target('sleep')
            self.arm.go()
            rospy.sleep(1)

        except Exception as e:
            print(f"操作失败: {str(e)}")
        finally:
            moveit_commander.roscpp_shutdown()
            moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        pick_place = MoveItPickPlace()
        pick_place.execute_pick_place()
    except rospy.ROSInterruptException:
        pass