#!/usr/bin/env python3

# -*- coding: utf-8 -*-


import rospy

import numpy as np

import cv2

from cv_bridge import CvBridge

from sensor_msgs.msg import Image

import actionlib

from sagittarius_object_color_detector.msg import SGRCtrlAction, SGRCtrlGoal


# HSV thresholds for red

RED_LOWER1 = np.array([0, 70, 50])

RED_UPPER1 = np.array([10, 240, 240])

RED_LOWER2 = np.array([170, 70, 50])

RED_UPPER2 = np.array([180, 240, 240])

AREA_THRESHOLD = 80 * 80  # pixels



def object_detector(img):

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask = cv2.add(

        cv2.inRange(hsv, RED_LOWER1, RED_UPPER1),

        cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)

    )

    mask = cv2.erode(mask, None, iterations=2)

    mask = cv2.dilate(mask, None, iterations=2)

    mask = cv2.GaussianBlur(mask, (5, 5), 0)

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    max_area = 0

    cx = cy = None

    for c in contours:

        rect = cv2.minAreaRect(c)

        box = cv2.boxPoints(rect)

        box = np.int0(box)

        w = np.linalg.norm(box[0] - box[1])

        h = np.linalg.norm(box[1] - box[2])

        area = w * h

        if area > max_area:

            max_area = area

            cx = np.mean(box[:, 0])

            cy = np.mean(box[:, 1])

    if max_area >= AREA_THRESHOLD:

        return cx, cy

    return None, None



def main():

    rospy.init_node('red_block_scanner_node')

    bridge = CvBridge()


    # Conversion params

    k1 = rospy.get_param('~k1', -0.0015)

    b1 = rospy.get_param('~b1', 0.45)

    k2 = rospy.get_param('~k2', 0.0015)

    b2 = rospy.get_param('~b2', -0.12)


    # Arm client

    arm_name = rospy.get_param('~arm_name', 'sgr532')

    client = actionlib.SimpleActionClient(f"{arm_name}/sgr_ctrl", SGRCtrlAction)

    client.wait_for_server()


    # Base goal setup

    goal = SGRCtrlGoal()

    goal.action_type = SGRCtrlGoal.ACTION_TYPE_XYZ_RPY

    goal.grasp_type = SGRCtrlGoal.GRASP_OPEN

    goal.pos_x = 0.3
    goal.pos_z = 0.15

    goal.pos_pitch = 0.77


    # Scan Y

    y_vals = np.linspace(-0.25, 0.25, 9)  # Updated scan range

    found = False

    pos_arm_x = pos_arm_y = None

    for y in y_vals:

        goal.pos_y = float(y)

        rospy.loginfo(f"Moving scan to pos_y={goal.pos_y:.3f}")

        client.send_goal_and_wait(goal, rospy.Duration(10.0))

        rospy.sleep(0.25)  # Reduced wait time

        try:

            img_msg = rospy.wait_for_message('/usb_cam/image_raw', Image, timeout=2.0)

            cv_image = bridge.imgmsg_to_cv2(img_msg, 'bgr8')

        except Exception as e:

            rospy.logwarn(f"Image capture failed: {e}")

            continue

        cx, cy = object_detector(cv_image)

        if cx is not None:

            pos_arm_x = k1 * cy + b1

            pos_arm_y = k2 * cx + b2

            rospy.loginfo(f"Detected red block at pixel x={cx:.1f}, y={cy:.1f}")

            rospy.loginfo(f"Mapped to arm x={pos_arm_x:.3f}, y={pos_arm_y:.3f}")

            found = True

            break


    if not found:

        rospy.loginfo("Scan complete, no red block found.")

        return


if __name__ == '__main__':

    main()