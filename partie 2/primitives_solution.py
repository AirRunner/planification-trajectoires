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
PR2_RADIUS = 0.38
CURRENT_PR2_ROOM = None
CURRENT_PR2_BALL = None


#####################################################################################
# CLASS: InvalidGoalState                                                           #
# Exception that is raised when we detect that the plan is invalid.                 #
#####################################################################################
class InvalidPlan(Exception):
    pass


#####################################################################################
# CLASS: InvalidAction                                                              #
# Exception that is raised when we detect that the action is invalid.               #
#####################################################################################
class InvalidAction(Exception):
    pass


#####################################################################################
# CLASS: Room                                                                       #
# Defines a room and its position in the environment.                               #
#####################################################################################
class Room:
    def __init__(self, name, x, y):
        self.name = name
        self.x_coord = x
        self.y_coord = y


#####################################################################################
# CLASS: Ball                                                                       #
# Defines a ball and its position                                                   #
#####################################################################################
class Ball:
    def __init__(self, name, currentRoom):
        self.name = name
        self.currentRoom = currentRoom


#####################################################################################
# CLASS: Action                                                                     #
# Superclass for all action.                                                        #
#####################################################################################
class Action:
    def __init__(self, moveGroup):
        self.moveGroup = moveGroup

    def execute(self):
        raise NotImplementedError("Action subclass must implement this method.")


#####################################################################################
# CLASS: Move                                                                       #
# Move action. The robot moves to the designated room.                              #
#####################################################################################
class Move(Action):
    def __init__(self, moveGroup, room):
        Action.__init__(self, moveGroup)
        self.destRoom = room

    def execute(self):
        global CURRENT_PR2_ROOM

        print("== move(" + self.destRoom.name + ")")

        targetJointValue = {"base_footprint_joint_rev": 0.0,
                            "base_footprint_joint_x": self.destRoom.x_coord,
                            "base_footprint_joint_y": self.destRoom.y_coord}

        self.moveGroup.set_joint_value_target(targetJointValue)

        # We use a large goal tolerance for the move action. We want the robot to move
        # approximately to the designated X and Y coordinates.
        self.moveGroup.set_goal_tolerance(0.95)

        plan = self.moveGroup.plan()

        if isPlanValid(plan):
            self.moveGroup.execute(plan)
            CURRENT_PR2_ROOM = self.destRoom
            print("== move completed")
        else:
            print("== move FAILED")
            raise InvalidPlan


#####################################################################################
# CLASS: PickBall                                                                   #
# Pick a ball action. The robot moves toward the ball and pick it.                  #
#####################################################################################
class PickBall(Action):
    def __init__(self, moveGroup, ball):
        Action.__init__(self, moveGroup)
        self.ball = ball

    def execute(self):
        global CURRENT_PR2_BALL

        print("== pickBall(" + self.ball.name + ")")

        if not self.__isPreconditionValid():
            print("== pickBall FAILED")
            raise InvalidAction




        #moveGroup.get_current_pose('l_shoulder_pan_joint') / moveGroup.get_active_joints()        



        #print(self.moveGroup2.get_current_joint_values())

        # With pr2_moveit_base2, planning group "base":
        # base_joint_values = ['base_footprint_joint_y', 'base_footprint_joint_x', 'base_footprint_joint_rev']
        base_joint_values = self.moveGroup.get_current_joint_values()
        base_y = base_joint_values[0]
        base_x = base_joint_values[1]
        base_rev = base_joint_values[2]

        obj = scene.get_objects([self.ball.name])
        obj_x = obj[self.ball.name].primitive_poses[0].position.x
        obj_y = obj[self.ball.name].primitive_poses[0].position.y

        print(obj[self.ball.name].primitive_poses[0].position.z)


        obj[self.ball.name].primitive_poses[0].position.z = 10


        # Compute the angle between the robot and the ball. We want the robot to face the ball.
        angle = math.atan2(obj_y - base_y, obj_x - base_x)

        # Compute the distance between the robot and the edge of the ball.
        distance = math.sqrt((obj_x - base_x) ** 2 + (obj_y - base_y) ** 2) - (BALL_RADIUS + PR2_RADIUS)

        targetJointValue = {"base_footprint_joint_rev": angle,
                            "base_footprint_joint_x": base_x + (distance * math.cos(angle)),
                            "base_footprint_joint_y": base_y + (distance * math.sin(angle))}

        self.moveGroup.set_joint_value_target(targetJointValue)

        # Since we want the robot to move exactly at the calculated position,
        # we set the goal tolerance to 0.
        self.moveGroup.set_goal_tolerance(0.0)

        plan = self.moveGroup.plan()

        if isPlanValid(plan):
            self.moveGroup.execute(plan)
            moveGroup.attach_object(self.ball.name)
            CURRENT_PR2_BALL = self.ball
            print("== pickBall completed")
        else:
            print("== pickBall FAILED")
            raise InvalidPlan

    def __isPreconditionValid(self):
        global CURRENT_PR2_ROOM
        global CURRENT_PR2_BALL

        if (self.ball.currentRoom is CURRENT_PR2_ROOM) and (CURRENT_PR2_BALL is None):
            return True
        else:
            return False


#####################################################################################
# CLASS: DropBall                                                                   #
# Drop a ball action. The robot drop the ball at the current position.              #
#####################################################################################
class DropBall(Action):
    def __init__(self, moveGroup, ball):
        Action.__init__(self, moveGroup)
        self.ball = ball

    def execute(self):
        global CURRENT_PR2_BALL
        global CURRENT_PR2_ROOM

        print("== dropBall(" + self.ball.name + ")")

        if not self.__isPreconditionValid():
            raise InvalidAction

        moveGroup.detach_object(self.ball.name)
        CURRENT_PR2_BALL = None
        self.ball.currentRoom = CURRENT_PR2_ROOM

        print("== dropBall completed")

    def __isPreconditionValid(self):
        global CURRENT_PR2_BALL

        if CURRENT_PR2_BALL is self.ball:
            return True
        else:
            return False


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

# -------------------------------------------------------------------------------
# isPlanValid
#   Check if a plan is valid.
# return: True if plan is valid, False otherwise.
# -------------------------------------------------------------------------------
def isPlanValid(plan):
    # A valid plan will contains a trajectory.
    #remove_world_objectss
    if not plan.joint_trajectory.points:
        return False
    else:
        return True


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
        moveGroup = moveit_commander.MoveGroupCommander("whole_body")

        moveGroup.get_active_joints()        

        #moveGroup2 =  moveit_commander.MoveGroupCommander("whole_body")

        # Set the maximum amount of time to use when planning (in seconds).
        # If no plan is found before the time limit, the planning fails.
        moveGroup.set_planning_time(10.0)
        
        #moveGroup2.set_planning_time(10.0)


        # Wait for RVIZ to initialize.
        rospy.sleep(5)

        # Clear the environment.
        scene.remove_world_object()

        # Generate the environment.
        generateEnvironment(scene, moveGroup)

        # Wait for the environment to display.
        rospy.sleep(1)

        room0 = Room("room0", 0.0, 0.0)
        room1 = Room("room1", 4.0, 3.0)
        room2 = Room("room2", -4.0, 3.0)
        room3 = Room("room3", 4.0, -3.0)
        room4 = Room("room4", -4.0, -3.0)
        room5 = Room("room5", 0.0, 3.0)
        room6 = Room("room6", 0.0, -3.0)

        ball1 = Ball("ball1", room1)
        ball2 = Ball("ball2", room2)
        ball3 = Ball("ball3", room3)
        ball4 = Ball("ball4", room4)
        ball5 = Ball("ball5", room5)

        CURRENT_PR2_ROOM = room0

        # Create a list of actions.
        # These action will cause the robot to bring all the balls to the center room.
        actions = []

        actions.append(Move(moveGroup, room1))
        #actions.append(PickBall(moveGroup, ball1,moveGroup2))
        actions.append(PickBall(moveGroup, ball1))

        actions.append(Move(moveGroup, room0))
        actions.append(DropBall(moveGroup, ball1))

        actions.append(Move(moveGroup, room5))
        actions.append(PickBall(moveGroup, ball5))
        actions.append(Move(moveGroup, room0))
        actions.append(DropBall(moveGroup, ball5))

        actions.append(Move(moveGroup, room2))
        actions.append(PickBall(moveGroup, ball2))
        actions.append(Move(moveGroup, room0))
        actions.append(DropBall(moveGroup, ball2))

        actions.append(Move(moveGroup, room3))
        actions.append(PickBall(moveGroup, ball3))
        actions.append(Move(moveGroup, room0))
        actions.append(DropBall(moveGroup, ball3))

        actions.append(Move(moveGroup, room4))
        actions.append(PickBall(moveGroup, ball4))
        actions.append(Move(moveGroup, room0))
        actions.append(DropBall(moveGroup, ball4))

        actions.append(Move(moveGroup, room6))

        # Execute the list of actions.
        for action in actions:
            try:
                action.execute()
            except InvalidPlan:
                print("Action failed: Failed to find a plan or the plan is invalid.")
                break
            except InvalidAction:
                print("Action failed: Precondition is not respected")
                break

        # Shut down moveit_commander.
        moveit_commander.roscpp_shutdown()

    except rospy.ROSInterruptException:
        pass
