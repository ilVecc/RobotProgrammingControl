#!/usr/bin/env python

from __future__ import print_function

import copy
import sys

import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import numpy as np
import rospy
import tf.transformations
from six.moves import input

from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = np.sqrt((x1 - x0)**2 + (y1 - y0)**2 + (z1 - z0)**2)
        # phi = angle between orientations
        cos_phi_half = np.abs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= np.cos(tolerance / 2.0)

    return True


def make_pose_rpy(ref_frame, xyz, rpy):
    pose_goal = geometry_msgs.msg.PoseStamped()
    pose_goal.header.frame_id = ref_frame
    pose_goal.pose.position.x = xyz[0]
    pose_goal.pose.position.y = xyz[1]
    pose_goal.pose.position.z = xyz[2]
    q_rot = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
    pose_goal.pose.orientation.x = q_rot[0]
    pose_goal.pose.orientation.y = q_rot[1]
    pose_goal.pose.orientation.z = q_rot[2]
    pose_goal.pose.orientation.w = q_rot[3]
    return pose_goal


class MoveGroupUR5(object):

    def __init__(self):
        super(MoveGroupUR5, self).__init__()

        # init moveit
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('homework_moveit_ur5', anonymous=True)

        # get robot info and commander
        robot = moveit_commander.RobotCommander()
        # list all the groups in the robot ...
        group_names = robot.get_group_names()  # ['endeffector', 'manipulator']
        print("============ Available Planning Groups:", robot.get_group_names())
        # ... and select the "manipulator" group (from base_link to ee_link, ignoring hand)
        if 'manipulator' in group_names:
            group_name = 'manipulator'
        else:
            raise RuntimeError('No \'manipulator\' planning group available')
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_num_planning_attempts(10)

        # get the 3D scene handle
        scene = moveit_commander.PlanningSceneInterface()
        # init the publisher to display the trajectories in Rviz using "moveit_msgs.msg.DisplayTrajectory" messages
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        # save some stuff
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
  
    def check_state_near(self, goal_state, tolerance=0.01):
        """
        Convenience method for testing if the values in two lists are within a tolerance of each other.
        For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
        between the identical orientations q and -q is calculated correctly).
        @param: goal       A list of floats, a Pose or a PoseStamped
        @param: actual     A list of floats, a Pose or a PoseStamped
        @param: tolerance  A float
        @returns: bool
        """
        if type(goal_state) is list:
            actual_state = self.move_group.get_current_joint_values()
            for index in range(len(goal_state)):
                if abs(actual_state[index] - goal_state[index]) > tolerance:
                    return False
        
        elif type(goal_state) is geometry_msgs.msg.PoseStamped:
            actual_state = self.move_group.get_current_pose()
            return all_close(goal_state.pose, actual_state.pose, tolerance)
        
        elif type(goal_state) is geometry_msgs.msg.Pose:
            actual_state = self.move_group.get_current_pose().pose
            x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual_state)
            x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal_state)
            # Euclidean distance
            d = np.sqrt((x1-x0)**2 + (y1-y0)**2 + (z1-z0)**2)
            # phi = angle between orientations
            cos_phi_half = abs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
            return d <= tolerance and cos_phi_half >= np.cos(tolerance / 2.0)
        
        return True
    
    def go_to(self, goal):
        self.move_group.go(goal, wait=True)
        self.move_group.stop()
        return self.check_state_near(goal)

    def go_to_rest(self):
        return self.go_to([0, 0, 0, 0, 0, 0])
    
    def go_to_ready(self):
        return self.go_to([0, -np.pi/2, np.pi/2, 0, 0, 0])

    def line_to(self, pose):
        # interpolate at a resolution of 1 cm (eef_step=0.01)
        # ignoring the check for infeasible jumps in joint space (ok for now)
        (plan, fraction) = self.move_group.compute_cartesian_path(
            [pose],  # single waypoint to go to
            0.01,  # eef_step
            0.0)  # jump_threshold

        # beware! planning displays the trajectory automatically
        return plan, fraction


    def task_1(self):
        """
        Go to a specific pose
        """
        pose = make_pose_rpy(self.move_group.get_planning_frame(), [0.45, -0.25, 0.3], [0, 0, np.pi])
        return self.go_to(pose)

    def task_2(self):
        """
        Go to a specific joint state
        """
        return self.go_to_ready()

    def task_3(self):
        """
        Execute line trajectory
        """
        pose = self.move_group.get_current_pose().pose
        pose.position.z -= 0.2  # move up (z)
        pose.position.y += 0.4  # and sideways (y)
        plan, _ = self.line_to(pose)
        self.move_group.execute(plan, wait=True)

    def task_4(self, scale=-1):
        """
        Execute multiple points trajectory
        """
        # specify a list of waypoints for the end-effector Cartesian path
        waypoints = []
        wpose = self.move_group.get_current_pose().pose

        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # interpolate at a resolution of 1 cm (eef_step=0.01)
        # ignoring the check for infeasible jumps in joint space (ok for now)
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        # beware! planning displays the trajectory automatically
        self.move_group.execute(plan, wait=True)

    def task_5(self):
        """
        Execute specific line trajectory
        """
        pose = make_pose_rpy(self.move_group.get_planning_frame(), [0.3, 0.1, 0.3], [0, 0, 0]).pose
        self.go_to(pose)
        rospy.sleep(rospy.Duration(secs=1))
        pose.position.x += 0.2
        plan, _ = self.line_to(pose)
        self.move_group.execute(plan, wait=True)

    def task_6(self):
        """
        Show collision avoidance
        """
        pose = make_pose_rpy(self.move_group.get_planning_frame(), [0.3, 0.4, 0.45], [np.pi, 0, 0])
        self.go_to(pose)
        self.add_obstacle()
        pose = make_pose_rpy(self.move_group.get_planning_frame(), [0.5, -0.4, 0.45], [np.pi, 0, 0])
        self.go_to(pose)
        self.remove_object("obstacle")

    def task_7(self):
        """
        Show collision avoidance with extra cylinder
        """
        pose = make_pose_rpy(self.move_group.get_planning_frame(), [0.3, 0.4, 0.45], [np.pi, 0, 0])
        self.go_to(pose)
        self.add_obstacle()
        self.add_cylinder()
        self.attach_object("cylinder")
        pose = make_pose_rpy(self.move_group.get_planning_frame(), [0.45, -0.2, 0.3], [np.pi, 0, 0])
        self.go_to(pose)
        self.detach_object("cylinder")
        self.remove_object("cylinder")
        self.remove_object("obstacle")


    def wait_for_state_update(self, object_name, object_is_known=False, object_is_attached=False, timeout=4):
        # ensure that the updates are made by waiting until we see the changes reflected in the
        # get_attached_objects() and get_known_object_names() lists
        start = rospy.get_time()
        while (rospy.get_time() - start < timeout) and not rospy.is_shutdown():
            # test if box is attached
            attached_objects = self.scene.get_attached_objects([object_name])
            is_attached = len(attached_objects.keys()) > 0
            # test if box is in scene (beware! if box is attached then it's not known anymore)
            is_known = object_name in self.scene.get_known_object_names()

            # test if we are in the expected state
            if (object_is_attached == is_attached) and (object_is_known == is_known):
                return True

            # let's wait a little more... and hope...
            rospy.sleep(0.1)

        # if timeout or ros was killed
        return False

    def add_obstacle(self, obstacle_name="obstacle"):
        obstacle_pose = make_pose_rpy(self.move_group.get_planning_frame(), [0.40, 0, 0], [0, 0, 0])
        self.scene.add_box(obstacle_name, obstacle_pose, size=(0.1, 0.1, 1.6))
        assert self.wait_for_state_update(obstacle_name, object_is_known=True), "Could not add obstacle"

    def add_cylinder(self, cylinder_name="cylinder"):
        cylinder_pose = make_pose_rpy(self.move_group.get_end_effector_link(), [0.15, 0, 0], [0, np.pi/2, 0])
        self.scene.add_cylinder(cylinder_name, cylinder_pose, height=0.3, radius=0.05)
        assert self.wait_for_state_update(cylinder_name, object_is_known=True), "Could not add %s".format(cylinder_name)

    def attach_object(self, object_name):
        # attach a cylinder to the UR5 wrist
        # to do so, we just add each link name of the group to the ``touch_links`` array,
        # so we can touch without the planning scene reporting the contact as a collision
        group_names = self.robot.get_group_names()
        if 'endeffector' in group_names:
            grasping_group = 'endeffector'  # is just the gripper group
        else:
            raise RuntimeError('No \'endeffector\' planning group available')
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.move_group.get_end_effector_link(), object_name, touch_links=touch_links)
        assert self.wait_for_state_update(object_name, object_is_attached=True, object_is_known=False), "Could not attach %s".format(object_name)

    def detach_object(self, object_name):

        # detach the object from the planning scene:
        self.scene.remove_attached_object(self.move_group.get_end_effector_link(), name=object_name)
        assert self.wait_for_state_update(object_name, object_is_attached=False, object_is_known=True), "Could not detach %s".format(object_name)

    def remove_object(self, object_name):
        self.scene.remove_world_object(object_name)
        assert self.wait_for_state_update(object_name, object_is_known=False), "Could not remove %s".format(object_name)


def print_header():
    margin = 10
    title = "MoveIt UR5"
    print("")
    print("-" * (2*margin + len(title)))
    print(" " * margin + title + " " * margin)
    print("-" * (2*margin + len(title)))
    print("Press Ctrl-D to exit at any time")
    print("")


def main():
    wrapper = None
    try:
        print_header()
        print("============ Set up the moveit_commander ...")
        wrapper = MoveGroupUR5()

        ## WARNING: before adding objects to the planning scene we must move the robot,
        ##          even just a little bit; I don't have any idea why and I won't investigate

        input("============ TASK 1: press `Enter` for arbitrary pose ...")
        wrapper.task_1()
        
        input("============ TASK 2: press `Enter` for arbitrary joint state ...")
        wrapper.task_2()

        input("============ TASK 3: press `Enter` for line trajectory ...")
        wrapper.task_3()

        input("============ TASK 4: press `Enter` for multiple waypoints trajectory ...")
        wrapper.task_4()
        
        input("============ TASK 5: press `Enter` for a XY-plane line trajectory ...")
        wrapper.task_5()

        input("============ TASK 6: press `Enter` for a collision-avoidance demo ...")
        wrapper.task_6()

        input("============ TASK 7: press `Enter` for a collision-avoidance demo with extra cylinder ...")
        wrapper.task_7()

        wrapper = None

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    finally:
        if wrapper is not None:
            wrapper.remove_object("cylinder")
            wrapper.remove_object("obstacle")


if __name__ == '__main__':
    main()
