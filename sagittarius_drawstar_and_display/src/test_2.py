#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, Point, Quaternion

class HanoiSolver:
    def __init__(self):
        # 初始化 MoveIt!
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('hanoi_solver', anonymous=True)
        
        # 初始化机械臂和夹爪
        self.arm = MoveGroupCommander('sagittarius_arm')
        self.gripper = MoveGroupCommander('sagittarius_gripper')
        
        # 设置机械臂参数
        self.arm.set_pose_reference_frame('world')
        self.arm.set_max_velocity_scaling_factor(0.3)
        self.arm.set_planning_time(10)
        self.arm.allow_replanning(True)
        self.arm.set_planner_id("RRTConnectkConfigDefault")
        
        # 定义七组抓取点和放置点坐标
        self.moves = [
            ((0.3, 0.2, 0.135), (0.3, -0.2, 0.035)),  # 第一次
            ((0.3, 0.2, 0.082), (0.3, 0.0, 0.035)),   # 第二次
            ((0.3, -0.2, 0.035), (0.3, 0.0, 0.082)),  # 第三次
            ((0.3, 0.2, 0.035), (0.3, -0.2, 0.035)),  # 第四次
            ((0.3, 0.0, 0.082), (0.3, 0.2, 0.035)),   # 第五次
            ((0.3, 0.0, 0.035), (0.3, -0.2, 0.082)),  # 第六次
            ((0.3, 0.2, 0.035), (0.3, -0.2, 0.135))   # 第七次
        ]
    
    def _move_to_point(self, x, y, z, max_attempts=3):
        """移动机械臂到指定坐标，支持重试"""
        print(f"Moving to ({x}, {y}, {z})")
        target_pose = Pose()
        target_pose.position = Point(x, y, z)
        target_pose.orientation = Quaternion(0, 0, 0, 1)
        self.arm.set_pose_target(target_pose)
        for attempt in range(max_attempts):
            success = self.arm.go()
            if success:
                return True
            rospy.logwarn(f"Attempt {attempt + 1} failed to move to ({x}, {y}, {z})")
            self.arm.clear_pose_targets()
        rospy.logerr(f"Failed to move to ({x}, {y}, {z}) after {max_attempts} attempts")
        return False
    
    def _control_gripper(self, state):
        """控制夹爪打开或闭合"""
        self.gripper.set_named_target(state)
        self.gripper.go()
    
    def solve_hanoi(self):
        try:
            # 初始化：移动到第一个抓取点的上方 (0.3, 0.2, 0.3)
            print(f"初始化：移动到第一个抓取点上方 (0.3, 0.2, 0.3)")
            self._move_to_point(0.3, 0.2, 0.3)
            
            # 第一步：抓取 (0.3, 0.2, 0.135) -> 放置 (0.3, -0.2, 0.035)
            print("第 1 次移动：抓取 (0.3, 0.2, 0.135) -> 放置 (0.3, -0.2, 0.035)")
            self._move_to_point(0.3, 0.2, 0.135)  # 下降到抓取点
            self._control_gripper('open')
            rospy.sleep(0.5)
            self._control_gripper('close')
            rospy.sleep(0.5)
            self._move_to_point(0.3, 0.2, 0.3)  # 上升到 z=0.3
            self._move_to_point(0.3, -0.2, 0.3)  # 移动到放置点上方
            self._move_to_point(0.3, -0.2, 0.035)  # 下降到放置点
            self._control_gripper('open')
            rospy.sleep(0.5)
            self._move_to_point(0.3, -0.2, 0.3)  # 放置后上升
            self._move_to_point(0.3, 0.2, 0.3)  # 移动到下一个抓取点上方
            
            # 第二步：抓取 (0.3, 0.2, 0.082) -> 放置 (0.3, 0.0, 0.035)
            print("第 2 次移动：抓取 (0.3, 0.2, 0.082) -> 放置 (0.3, 0.0, 0.035)")
            self._move_to_point(0.3, 0.2, 0.082)
            self._control_gripper('open')
            rospy.sleep(0.5)
            self._control_gripper('close')
            rospy.sleep(0.5)
            self._move_to_point(0.3, 0.2, 0.3)
            self._move_to_point(0.3, 0.0, 0.3)
            self._move_to_point(0.3, 0.0, 0.035)
            self._control_gripper('open')
            rospy.sleep(0.5)
            self._move_to_point(0.3, 0.0, 0.3)
            self._move_to_point(0.3, -0.2, 0.3)  # 移动到下一个抓取点上方
            
            # 第三步：抓取 (0.3, -0.2, 0.035) -> 放置 (0.3, 0.0, 0.082)
            print("第 3 次移动：抓取 (0.3, -0.2, 0.035) -> 放置 (0.3, 0.0, 0.082)")
            self._move_to_point(0.3, -0.2, 0.035)
            self._control_gripper('open')
            rospy.sleep(0.5)
            self._control_gripper('close')
            rospy.sleep(0.5)
            self._move_to_point(0.3, -0.2, 0.3)
            self._move_to_point(0.3, 0.0, 0.3)
            self._move_to_point(0.3, 0.0, 0.082)
            self._control_gripper('open')
            rospy.sleep(0.5)
            self._move_to_point(0.3, 0.0, 0.3)
            self._move_to_point(0.3, 0.2, 0.3)  # 移动到下一个抓取点上方
            
            # 第四步：抓取 (0.3, 0.2, 0.035) -> 放置 (0.3, -0.2, 0.035)
            print("第 4 次移动：抓取 (0.3, 0.2, 0.035) -> 放置 (0.3, -0.2, 0.035)")
            self._move_to_point(0.3, 0.2, 0.035)
            self._control_gripper('open')
            rospy.sleep(0.5)
            self._control_gripper('close')
            rospy.sleep(0.5)
            self._move_to_point(0.3, 0.2, 0.3)
            self._move_to_point(0.3, -0.2, 0.3)
            self._move_to_point(0.3, -0.2, 0.035)
            self._control_gripper('open')
            rospy.sleep(0.5)
            self._move_to_point(0.3, -0.2, 0.3)
            self._move_to_point(0.3, 0.0, 0.3)  # 移动到下一个抓取点上方
            
            # 第五步：抓取 (0.3, 0.0, 0.082) -> 放置 (0.3, 0.2, 0.035)
            print("第 5 次移动：抓取 (0.3, 0.0, 0.082) -> 放置 (0.3, 0.2, 0.035)")
            self._move_to_point(0.3, 0.0, 0.082)
            self._control_gripper('open')
            rospy.sleep(0.5)
            self._control_gripper('close')
            rospy.sleep(0.5)
            self._move_to_point(0.3, 0.0, 0.3)
            self._move_to_point(0.3, 0.2, 0.3)
            self._move_to_point(0.3, 0.2, 0.035)
            self._control_gripper('open')
            rospy.sleep(0.5)
            self._move_to_point(0.3, 0.2, 0.3)
            self._move_to_point(0.3, 0.0, 0.3)  # 移动到下一个抓取点上方
            
            # 第六步：抓取 (0.3, 0.0, 0.035) -> 放置 (0.3, -0.2, 0.082)
            print("第 6 次移动：抓取 (0.3, 0.0, 0.035) -> 放置 (0.3, -0.2, 0.082)")
            self._move_to_point(0.3, 0.0, 0.035)
            self._control_gripper('open')
            rospy.sleep(0.5)
            self._control_gripper('close')
            rospy.sleep(0.5)
            self._move_to_point(0.3, 0.0, 0.3)
            self._move_to_point(0.3, -0.2, 0.3)
            self._move_to_point(0.3, -0.2, 0.082)
            self._control_gripper('open')
            rospy.sleep(0.5)
            self._move_to_point(0.3, -0.2, 0.3)
            self._move_to_point(0.3, 0.2, 0.3)  # 移动到下一个抓取点上方
            
            # 第七步：抓取 (0.3, 0.2, 0.035) -> 放置 (0.3, -0.2, 0.135)
            print("第 7 次移动：抓取 (0.3, 0.2, 0.035) -> 放置 (0.3, -0.2, 0.135)")
            self._move_to_point(0.3, 0.2, 0.035)
            self._control_gripper('open')
            rospy.sleep(0.5)
            self._control_gripper('close')
            rospy.sleep(0.5)
            self._move_to_point(0.3, 0.2, 0.3)
            self._move_to_point(0.3, -0.2, 0.3)
            self._move_to_point(0.3, -0.2, 0.135)
            self._control_gripper('open')
            rospy.sleep(0.5)
            self._move_to_point(0.3, -0.2, 0.3)  # 最后一步，上升到 z=0.3
            
            print("任务完成！")
        
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