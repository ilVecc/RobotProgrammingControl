#!/usr/bin/env python

from __future__ import print_function

import rospy
import tf
from sensor_msgs.msg import JointState
from kinematics import make_robot, h_to_pose

def abs_error(x0, x1):
    return sum([abs(e0 - e1) for e0, e1 in zip(x0, x1)])

if __name__ == "__main__":
    rospy.init_node("homework_scara_controller", anonymous=True)

    js = [0.5, -1.2, -0.3, 0.4]

    ####################
    # ROS COMPUTATION
    ####################
    pub = rospy.Publisher("/scara_joint_states", JointState, queue_size=20, latch=True)
    print("Publishing...", end="")
    pub.publish(
        JointState(
            name=['shoulder_joint', 'elbow_joint', 'wrist_joint', 'probe_joint'],
            position=js
        )
    )
    print("done")
    rospy.sleep(0.5)  # wait a little bit to properly send the message

    listener = tf.TransformListener()
    # WARNING: quering /tf too soon after node initialization will just say 
    #          there's no transform at all... Thus, the wait command above
    listener.waitForTransform("/base_link", "/ee_link", rospy.Time(), rospy.Duration(4.0))
    # get H_b_e
    t_auto, r_auto = listener.lookupTransform("/base_link", "/ee_link", rospy.Time())
    

    ####################
    # MANUAL COMPUTATION
    ####################
    robot = make_robot()
    # get H_b_e
    r, t = h_to_pose(robot.fk(js))
    t_manual = t.tolist()
    r_manual = [r.x, r.y, r.z, r.w]


    ####################
    # COMPARISON
    ####################
    print("  auto =>", t_auto, r_auto)
    print("manual =>", t_manual, r_manual)
    print("abs error t =", abs_error(t_auto, t_manual))
    print("abs error r =", abs_error(r_auto, r_manual))

    br = tf.TransformBroadcaster()
    send_tf = 5
    while send_tf > 0:
        br.sendTransform(t_auto, r_auto, rospy.Time.now(), "auto", "base_link")
        br.sendTransform(t_manual, r_manual, rospy.Time.now(), "manual", "base_link")
        rospy.sleep(0.5)
        send_tf -= 1

    pub.unregister()
