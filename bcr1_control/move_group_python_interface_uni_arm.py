# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R

import tf
import geometry_msgs

import time

moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "bcr1_planning"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")

print("============ Printing robot home position")
print(move_group.get_current_rpy())
print("")

def robot_pos(pose):
    move_group.set_planner_id(planner_id = "PTP") # STRIDE, RRT, AnytimePathShortening
    move_group.set_planning_time(seconds = 4)
    # set scaling factor
    move_group.set_max_acceleration_scaling_factor(value = 0.4)
    move_group.set_max_velocity_scaling_factor(value = 0.4)
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = pose[0]
    pose_goal.position.y = pose[1]
    pose_goal.position.z = pose[2]

    pose_goal.orientation.x = pose[3]
    pose_goal.orientation.y = pose[4]
    pose_goal.orientation.z = pose[5]
    pose_goal.orientation.w = pose[6]

    move_group.set_pose_target(pose_goal)

    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

joint_goal = [0, 0, 0, 0, 0, 0]

i = 1
while i < 8:
    # # Position and Rotation for z-axis
    # robot_pos([0.25, 0.15, 0.22, 1, 0, 0, 0])
    # time.sleep(2)
    # robot_pos([0.25, 0.15, 0.22-0.12, 1, 0, 0, 0])
    # time.sleep(2)
    # robot_pos([0.25, 0.15, 0.22, 1, 0, 0, 0])
    # time.sleep(2)
    # move_group.go(joint_goal, wait=True)
    # move_group.stop()
    
    # Position and Rotation for x-axis
    robot_pos([0.25, 0.15, 0.22, 0.7071068, 0, 0.7071068, 0 ])
    time.sleep(2)
    robot_pos([0.25+0.12, 0.15, 0.22, 0.7071068, 0, 0.7071068, 0 ])
    time.sleep(2)
    robot_pos([0.25, 0.15, 0.22, 0.7071068, 0, 0.7071068, 0 ])
    time.sleep(2)
    move_group.go(joint_goal, wait=True)
    move_group.stop()

    # # Position and Rotation for y-axis
    # robot_pos([0.25, 0.15, 0.22, 0.5, 0.5, 0.5, -0.5])
    # time.sleep(2)
    # robot_pos([0.25, 0.15+0.12, 0.22, 0.5, 0.5, 0.5, -0.5])
    # time.sleep(2)
    # robot_pos([0.25, 0.15, 0.22, 0.5, 0.5, 0.5, -0.5])
    # time.sleep(2)
    # move_group.go(joint_goal, wait=True)
    # move_group.stop()
    # time.sleep(2)

    # robot_pos([0.25, 0.15, 0.22, -0.38018842, 0, 0, 0.92490906])


    # r = R.from_matrix([
    # [1, 0, 0],
    # [0, -0.78, 0],
    # [0, 0, 0.78]])
    
    # print(r.as_quat())

    i=i+1
 
print("PROCESS SUCCEDED")
