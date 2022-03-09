#!/usr/bin/env python

#####################################################################################
# IMPORT                                                                            #
#####################################################################################
import sys
import math
import rospy
import moveit_commander
import geometry_msgs.msg


#####################################################################################
# CONSTANTS & GLOBAL VARIABLES                                                      #
#####################################################################################
BALL_RADIUS = 0.2


#####################################################################################
# FUNCTIONS                                                                         #
#####################################################################################
# -----------------------------------------------------------------------------------
# generateEnvironment
#   Generate the environment for the robot.
# -----------------------------------------------------------------------------------
def generateEnvironment(scene, moveGroup):
    wallPose1 = geometry_msgs.msg.PoseStamped()
    wallPose1.header.frame_id = moveGroup.get_planning_frame()
    wallPose1.pose.position.x = 5.0
    wallPose1.pose.position.y = 0.0
    wallPose1.pose.position.z = 1.0
    scene.add_box("wall1", wallPose1, (0.1, 10.0, 2))

    wallPose2 = geometry_msgs.msg.PoseStamped()
    wallPose2.header.frame_id = moveGroup.get_planning_frame()
    wallPose2.pose.position.x = -5.0
    wallPose2.pose.position.y = 0.0
    wallPose2.pose.position.z = 1.0
    scene.add_box("wall2", wallPose2, (0.1, 10.0, 2))

    wallPose3 = geometry_msgs.msg.PoseStamped()
    wallPose3.header.frame_id = moveGroup.get_planning_frame()
    wallPose3.pose.position.x = 0.0
    wallPose3.pose.position.y = 5.0
    wallPose3.pose.position.z = 1.0
    scene.add_box("wall3", wallPose3, (10.0, 0.1, 2))

    wallPose4 = geometry_msgs.msg.PoseStamped()
    wallPose4.header.frame_id = moveGroup.get_planning_frame()
    wallPose4.pose.position.x = 0.0
    wallPose4.pose.position.y = -5.0
    wallPose4.pose.position.z = 1.0
    scene.add_box("wall4", wallPose4, (10.0, 0.1, 2))

    wallPose5 = geometry_msgs.msg.PoseStamped()
    wallPose5.header.frame_id = moveGroup.get_planning_frame()
    wallPose5.pose.position.x = -3.5
    wallPose5.pose.position.y = 0.0
    wallPose5.pose.position.z = 1.0
    scene.add_box("wall5", wallPose5, (3.0, 0.1, 2))

    wallPose6 = geometry_msgs.msg.PoseStamped()
    wallPose6.header.frame_id = moveGroup.get_planning_frame()
    wallPose6.pose.position.x = 3.5
    wallPose6.pose.position.y = 0.0
    wallPose6.pose.position.z = 1.0
    scene.add_box("wall6", wallPose6, (3.0, 0.1, 2))

    wallPose7 = geometry_msgs.msg.PoseStamped()
    wallPose7.header.frame_id = moveGroup.get_planning_frame()
    wallPose7.pose.position.x = -2.0
    wallPose7.pose.position.y = -3.5
    wallPose7.pose.position.z = 1.0
    scene.add_box("wall7", wallPose7, (0.1, 3.0, 2))

    wallPose8 = geometry_msgs.msg.PoseStamped()
    wallPose8.header.frame_id = moveGroup.get_planning_frame()
    wallPose8.pose.position.x = -2.0
    wallPose8.pose.position.y = 3.5
    wallPose8.pose.position.z = 1.0
    scene.add_box("wall8", wallPose8, (0.1, 3.0, 2))

    wallPose9 = geometry_msgs.msg.PoseStamped()
    wallPose9.header.frame_id = moveGroup.get_planning_frame()
    wallPose9.pose.position.x = 2.0
    wallPose9.pose.position.y = -3.5
    wallPose9.pose.position.z = 1.0
    scene.add_box("wall9", wallPose9, (0.1, 3.0, 2))

    wallPose10 = geometry_msgs.msg.PoseStamped()
    wallPose10.header.frame_id = moveGroup.get_planning_frame()
    wallPose10.pose.position.x = 2.0
    wallPose10.pose.position.y = 3.5
    wallPose10.pose.position.z = 1.0
    scene.add_box("wall10", wallPose10, (0.1, 3.0, 2))

    wallPose11 = geometry_msgs.msg.PoseStamped()
    wallPose11.header.frame_id = moveGroup.get_planning_frame()
    wallPose11.pose.position.x = -1.5
    wallPose11.pose.position.y = -2.0
    wallPose11.pose.position.z = 1.0
    scene.add_box("wall11", wallPose11, (1.0, 0.1, 2))

    wallPose12 = geometry_msgs.msg.PoseStamped()
    wallPose12.header.frame_id = moveGroup.get_planning_frame()
    wallPose12.pose.position.x = 1.5
    wallPose12.pose.position.y = -2.0
    wallPose12.pose.position.z = 1.0
    scene.add_box("wall12", wallPose12, (1.0, 0.1, 2))

    wallPose13 = geometry_msgs.msg.PoseStamped()
    wallPose13.header.frame_id = moveGroup.get_planning_frame()
    wallPose13.pose.position.x = -1.5
    wallPose13.pose.position.y = 2.0
    wallPose13.pose.position.z = 1.0
    scene.add_box("wall13", wallPose13, (1.0, 0.1, 2))

    wallPose14 = geometry_msgs.msg.PoseStamped()
    wallPose14.header.frame_id = moveGroup.get_planning_frame()
    wallPose14.pose.position.x = 1.5
    wallPose14.pose.position.y = 2.0
    wallPose14.pose.position.z = 1.0
    scene.add_box("wall14", wallPose14, (1.0, 0.1, 2))

    ballPose1 = geometry_msgs.msg.PoseStamped()
    ballPose1.header.frame_id = moveGroup.get_planning_frame()
    ballPose1.pose.position.x = 4.0
    ballPose1.pose.position.y = 4.0
    ballPose1.pose.position.z = BALL_RADIUS
    scene.add_sphere("ball1", ballPose1, BALL_RADIUS)

    ballPose2 = geometry_msgs.msg.PoseStamped()
    ballPose2.header.frame_id = moveGroup.get_planning_frame()
    ballPose2.pose.position.x = -4.0
    ballPose2.pose.position.y = 4.0
    ballPose2.pose.position.z = BALL_RADIUS
    scene.add_sphere("ball2", ballPose2, BALL_RADIUS)

    ballPose3 = geometry_msgs.msg.PoseStamped()
    ballPose3.header.frame_id = moveGroup.get_planning_frame()
    ballPose3.pose.position.x = 4.0
    ballPose3.pose.position.y = -4.0
    ballPose3.pose.position.z = BALL_RADIUS
    scene.add_sphere("ball3", ballPose3, BALL_RADIUS)

    ballPose4 = geometry_msgs.msg.PoseStamped()
    ballPose4.header.frame_id = moveGroup.get_planning_frame()
    ballPose4.pose.position.x = -4.0
    ballPose4.pose.position.y = -4.0
    ballPose4.pose.position.z = BALL_RADIUS
    scene.add_sphere("ball4", ballPose4, BALL_RADIUS)

    ballPose5 = geometry_msgs.msg.PoseStamped()
    ballPose5.header.frame_id = moveGroup.get_planning_frame()
    ballPose5.pose.position.x = 0.0
    ballPose5.pose.position.y = 4.0
    ballPose5.pose.position.z = BALL_RADIUS
    scene.add_sphere("ball5", ballPose5, BALL_RADIUS)


#####################################################################################
# PROGRAM ENTRY POINT                                                               #
#####################################################################################
if __name__ == '__main__':
    try:
        # Initialize moveit_commander.
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node.
        rospy.init_node('algorithm_benchmark', anonymous=True)

        # Instantiate a PlanningSceneInterface object.
        # This object is an interface to the world surrounding the robot.
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a MoveGroupCommander object.
        # This enable us to generate plans for the selected planning group.
        moveGroup = moveit_commander.MoveGroupCommander("base")

        # Set the maximum amount of time to use when planning (in seconds).
        # If no plan is found before the time limit, the planning fails.
        moveGroup.set_planning_time(10.0)

        # Wait for RVIZ to initialize.
        rospy.sleep(5)

        # Clear the environment.
        scene.remove_world_object()

        # Generate the environment.
        generateEnvironment(scene, moveGroup)

        # Wait for the environment to display.
        rospy.sleep(1)

        # TODO:  Construct and execute a list of actions that causes the robot
        # TODO:  to bring all the balls to the center room (R0).

        # Shut down moveit_commander.
        moveit_commander.roscpp_shutdown()

    except rospy.ROSInterruptException:
        pass
