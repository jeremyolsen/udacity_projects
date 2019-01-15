#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
# import pydevd
# pydevd.settrace('10.0.0.8', port=5678)

import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np

# Moved all code to to initialization since these calcs are only used once per transform request
initialized = False
# Create symbols
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')  # link offset
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')  # link length
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')  # twist angle
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # joint angle

# Create Modified DH parameters
dh_table = {alpha0: 0, a0: 0, d1: 0.75, q1: q1,
            alpha1: -pi / 2., a1: 0.35, d2: 0, q2: q2 - pi / 2,
            alpha2: 0, a2: 1.25, d3: 0, q3: q3,
            alpha3: -pi / 2., a3: -0.054, d4: 1.5, q4: q4,
            alpha4: pi / 2., a4: 0, d5: 0, q5: q5,
            alpha5: -pi / 2., a5: 0, d6: 0, q6: q6,
            alpha6: 0, a6: 0, d7: 0.303, q7: 0}


# Define Modified DH Transformation matrix
def tf_matrix_template(alpha, a, d, q):
    tf_matrix = Matrix([[cos(q), -sin(q), 0, a],
                 [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                 [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d],
                 [0, 0, 0, 1]])
    return tf_matrix


# Create individual transformation matrices
tf_matrix_0_1 = tf_matrix_template(alpha0, a0, d1, q1).subs(dh_table)
tf_matrix_1_2 = tf_matrix_template(alpha1, a1, d2, q2).subs(dh_table)
tf_matrix_2_3 = tf_matrix_template(alpha2, a2, d3, q3).subs(dh_table)
tf_matrix_3_4 = tf_matrix_template(alpha3, a3, d4, q4).subs(dh_table)
tf_matrix_4_5 = tf_matrix_template(alpha4, a4, d5, q5).subs(dh_table)
tf_matrix_5_6 = tf_matrix_template(alpha5, a5, d6, q6).subs(dh_table)
tf_matrix_6_ee = tf_matrix_template(alpha6, a6, d7, q7).subs(dh_table)

t0_ee = tf_matrix_0_1 * tf_matrix_1_2 * tf_matrix_2_3 * tf_matrix_3_4 * tf_matrix_4_5 * tf_matrix_5_6 * tf_matrix_6_ee

# Extract extrinsic rotation matrices from the transformation matrices
r, p, y = symbols('r p y')
rotation_x = Matrix([[1, 0, 0],
                [0, cos(r), -sin(r)],
                [0, sin(r), cos(r)]])  # Roll
rotation_y = Matrix([[cos(p), 0, sin(p)],
                [0, 1, 0],
                [-sin(p), 0, cos(p)]])  # pitch
rotation_z = Matrix([[cos(y), -sin(y), 0],
                [sin(y), cos(y), 0],
                [0, 0, 1]])  # yaw
rotation_ee = rotation_z * rotation_y * rotation_x
rotation_error = rotation_z.subs(y, radians(180)) * rotation_y.subs(p, radians(-90))

rotation_error = rotation_ee * rotation_error

initialized = True

###
def queue_calc_request(req):
    while initialized is False:
        # do nothing
        print "Do nothing"
        continue
    handle_calculate_IK(req)

def handle_calculate_IK(req):
    if initialized is False:
        queue_calc_request(req)

    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            print('in loop x:', x)
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo

            rot_ee = rotation_error.subs({'r': roll, 'p': pitch, 'y': yaw})

            # Calculate joint angles using Geometric IK method
            ee = Matrix([[px],
                         [py],
                         [pz]])
            # wrist center matrix calc - 0.303 is the end effector offset from the DH Table/URDF file
            wc = ee - 0.303 * rot_ee[:, 2]

            # --Inverse Position Solution--

            # theta 1 angle is derived by looking down the z-axis
            # wc[1] is y coordinate and wc[0] is x coordinate
            theta_1 = atan2(wc[1], wc[0])

            # Distance obtained from DH Table / URDF - link offset of links 3->4
            side_a = 1.501
            # side b obtained by using Pythagorean Theorem
            side_b = sqrt(pow((sqrt(wc[0] * wc[0] + wc[1] * wc[1]) - 0.35), 2) +
                          pow((wc[2] - 0.75), 2))
            # Distance obtained from DH Table / URDF - link distance between links 2->3
            side_c = 1.25

            # Using the Laws of Sines (SOHCAHTOA) to determine the angles of the inverse orientation triangle
            angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
            angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))

            #  using Law of Cosines, we can derive theta 2 and 3 joint angles
            theta_2 = pi / 2 - angle_a - atan2(wc[2] - 0.75, sqrt(wc[0] * wc[0] + wc[1] * wc[1]) - 0.35)
            theta_3 = pi / 2 - (angle_b + 0.036)  # 0.036 is sag in link4 of -0.054m

            # --Inverse Orientation Solution--

            # create a composite rotation matrix from the transformation matrices that were created for the FK section
            r0_3 = tf_matrix_0_1[0:3, 0:3] * tf_matrix_1_2[0:3, 0:3] * tf_matrix_2_3[0:3, 0:3]
            # sub in the found theta's into the resulting matrix which gives us the rotation matrix from 0-3
            r0_3 = r0_3.evalf(subs={q1: theta_1, q2: theta_2, q3: theta_3})

            # multiplying the inverse of the 0-3 rotation matrix by the rotation of the end-effector results in
            # the rotation matrix for 3-6 which we can use to calculate the final thetas
            r3_6 = r0_3.inv("LU") * rot_ee

            # theta's 4-6 are calculated using an extrinsic xyz rotation per the lesson.  There are comments on slack
            # saying that this is the incorrect method and the wrist joints are xyx.  I've tried using different
            # calcs using the tf.transformation.euler_to_matrix() method without much success
            theta_4 = atan2(r3_6[2, 2], -r3_6[0, 2])
            theta_5 = atan2(sqrt(r3_6[0, 2] * r3_6[0, 2] + r3_6[2, 2] * r3_6[2, 2]), r3_6[1, 2])
            theta_6 = atan2(-r3_6[1, 1], r3_6[1, 0])

            # Populate response for the IK request
            # In the next line replace theta_1,theta_2...,theta_6 by your joint angle variables
            joint_trajectory_point.positions = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)

def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()


if __name__ == "__main__":
    IK_server()