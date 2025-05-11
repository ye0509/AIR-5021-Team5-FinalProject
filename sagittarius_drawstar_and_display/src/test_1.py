#!/usr/bin/env python

# -*- coding: utf-8 -*-


import rospy

import sys

import moveit_commander

from moveit_commander import MoveGroupCommander

from geometry_msgs.msg import Pose, Point, Quaternion


class HanoiSolver:

    def __init__(self):

        # Initialize MoveIt!

        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('hanoi_solver', anonymous=True)

        

        # Initialize arm and gripper

        self.arm = MoveGroupCommander('sagittarius_arm')

        self.gripper = MoveGroupCommander('sagittarius_gripper')

        

        # Set arm parameters

        self.arm.set_pose_reference_frame('world')

        self.arm.set_max_velocity_scaling_factor(0.2)

        self.arm.set_planning_time(10)

        self.arm.allow_replanning(True)

        self.arm.set_planner_id("RRTConnectkConfigDefault")

        

        # Define seven sets of grab and place points

        self.moves = [

            ((0.33, 0.2, 0.13), (0.33, -0.2, 0.030)),  # First

            ((0.33, 0.2, 0.080), (0.33, 0.0, 0.030)),   # Second

            ((0.33, -0.2, 0.030), (0.33, 0.0, 0.080)),  # Third

            ((0.33, 0.2, 0.030), (0.33, -0.2, 0.030)),  # Fourth

            ((0.33, 0.0, 0.080), (0.33, 0.2, 0.030)),   # Fifth

            ((0.33, 0.0, 0.030), (0.33, -0.2, 0.080)),  # Sixth

            ((0.33, 0.2, 0.030), (0.33, -0.2, 0.13))   # Seventh

        ]


    def _move_to_point(self, x, y, z, max_attempts=3):

        """Move arm to specified coordinates with retry"""

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

        """Control gripper open or close"""

        self.gripper.set_named_target(state)

        self.gripper.go()


    def solve_hanoi(self):

        try:

            # Move to initial center height (0, 0, 0.3)

            print("Initialize: Moving to center height (0, 0, 0.3)")

            self._move_to_point(0, 0, 0.3)

            

            # Initialize: Move above first grab point (0.2, 0.2, 0.3)

            print(f"Initialize: Moving above first grab point (0.2, 0.2, 0.3)")

            self._move_to_point(0.2, 0.2, 0.3)

            

            # Loop through seven grab and place operations

            for i, (grab_point, place_point) in enumerate(self.moves, 1):

                print(f"Move {i}: Grab {grab_point} -> Place {place_point}")

                

                # 1. Vertically descend to grab point

                self._move_to_point(*grab_point)

                # 2. Open gripper

                self._control_gripper('open')

                rospy.sleep(1.0)

                # 3. Close gripper to grab object

                self._control_gripper('close')

                rospy.sleep(1.0)

                # 4. Vertically ascend to z=0.3

                self._move_to_point(grab_point[0], grab_point[1], 0.3)

                # 5. Horizontally move above place point at z=0.3

                self._move_to_point(place_point[0], place_point[1], 0.3)

                # 6. Vertically descend to place point

                self._move_to_point(*place_point)

                # 7. Open gripper to release object

                self._control_gripper('open')

                rospy.sleep(1.0)

                # 8. Vertically ascend to z=0.3

                self._move_to_point(place_point[0], place_point[1], 0.3)

                

                # 9. If not the last move, move above next grab point at z=0.3

                if i < len(self.moves):

                    next_grab_point = self.moves[i][0]

                    print(f"Moving above next grab point {next_grab_point[0], next_grab_point[1], 0.3}")

                    self._move_to_point(next_grab_point[0], next_grab_point[1], 0.3)

            

            print("Task completed!")

            

            # Move to final center height (0, 0, 0.3)

            print("End: Moving to center height (0, 0, 0.3)")

            self._move_to_point(0, 0, 0.3)

        

        except Exception as e:

            print(f"Operation failed: {str(e)}")

        finally:

            moveit_commander.roscpp_shutdown()


if __name__ == "__main__":

    try:

        solver = HanoiSolver()

        solver.solve_hanoi()

    except rospy.ROSInterruptException:

        pass