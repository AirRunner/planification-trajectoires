#!/usr/bin/env python

#####################################################################################
# IMPORT                                                                            #
#####################################################################################
import sys
import math

from numpy import False_
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
    completed = " completed"
    failed = " failed"
    def __init__(self, moveGroup):
        self.moveGroup = moveGroup
        self.plan = None
        self.name = ""
        
        
    def execute(self):
        raise NotImplementedError("Action subclass must implement this method.")
    
    def print_action(self):
        print("== (" + self.name + ")")
        
    
    def print_end_action(self,success = True):
        print("== " + self.name + (Action.completed if success else Action.failed))

    def set_target_join(self,targetJointValue,tolerance = 0, dico = {}):
        targetJointValue.update(dico)

        print("target")
        print(targetJointValue)

        self.moveGroup.set_joint_value_target(targetJointValue)

        print("actual target")
        print(self.moveGroup.get_joint_value_target())

        self.moveGroup.set_goal_tolerance(tolerance)
        self.plan = self.moveGroup.plan()

        
        
    
    def move_plan(self,angle,x,y,tolerance = 0, dico = {}):
        targetJointValue = {"base_footprint_joint_rev": angle,
                            "base_footprint_joint_x": x,
                            "base_footprint_joint_y": y}
        self.set_target_join(targetJointValue,tolerance, dico)
        
    
    def execute_plan(self):
        valid_plan = isPlanValid(self.plan)
        self.print_end_action(valid_plan)
        if valid_plan:
            self.moveGroup.execute(self.plan)
        else:
            raise InvalidPlan
        
    def valid_precondition(self):
        if not self.isPreconditionValid():
            self.print_end_action(False)
            raise InvalidAction
        
    def get_robot_position(self):
        return self.moveGroup.get_current_joint_values()[:3]
    
    def get_object(self):
        obj_list = scene.get_objects([self.ball.name])
        pos = obj_list[self.ball.name].primitive_poses[0].position
        return pos.x, pos.y
    
    def add_ball_scene(self,x,y,name = "ball"):
        ballPose = geometry_msgs.msg.PoseStamped()
        ballPose.header.frame_id = moveGroup.get_planning_frame()
        ballPose.pose.position.x = x
        ballPose.pose.position.y = y
        ballPose.pose.position.z = BALL_RADIUS
        scene.add_sphere(name, ballPose, BALL_RADIUS)
    
    


#####################################################################################
# CLASS: Move                                                                       #
# Move action. The robot moves to the designated room.                              #
#####################################################################################
class Move(Action):
    def __init__(self, moveGroup, room):
        Action.__init__(self, moveGroup)
        self.destRoom = room
        self.name = "move"
        

    def execute(self):
        global CURRENT_PR2_ROOM

        self.print_action()
        self.move_plan(0,self.destRoom.x_coord,self.destRoom.y_coord,tolerance=0.25)
        
        self.execute_plan()
        print(CURRENT_PR2_ROOM.name + " --> "+ self.destRoom.name)
        CURRENT_PR2_ROOM = self.destRoom


#####################################################################################
# CLASS: PickBall                                                                   #
# Pick a ball action. The robot moves toward the ball and pick it.                  #
#####################################################################################
class PickBall(Action):
    pick_ball_pos = {"l_shoulder_pan_joint" : 0,
                            "r_shoulder_pan_joint" : 0,
                            "l_shoulder_lift_joint" : 1.2963,
                            "r_shoulder_lift_joint" : 1.2963,
                            "l_upper_arm_roll_joint" : 0,
                            "r_upper_arm_roll_joint" : 0,
                            "l_elbow_flex_joint" : -0.6,
                            "r_elbow_flex_joint" : -0.6,
                            "l_forearm_roll_joint" : 0,
                            "r_forearm_roll_joint" : 0,
                            "l_wrist_flex_joint" : -0.1,
                            "r_wrist_flex_joint" : -0.1,
                            "l_wrist_roll_joint" : 0.8,
                            "r_wrist_roll_joint" : -0.8
                            }
    raise_ball_pos = {
                                "l_shoulder_lift_joint" : 0,
                                "r_shoulder_lift_joint" : 0,
                                "l_elbow_flex_joint" : -0.6,
                                "r_elbow_flex_joint" : -0.6
                                }
    def __init__(self, moveGroup, ball):
        Action.__init__(self, moveGroup)
        self.ball = ball
        self.name = "pickball"

    def execute(self):
        global CURRENT_PR2_BALL

        self.print_action()
        
        self.valid_precondition()
        
        r_y, r_x, r_angle = self.get_robot_position()

        b_x, b_y = self.get_object()

        # Compute the angle between the robot and the ball. We want the robot to face the ball.
        angle = math.atan2(b_y - r_y, b_x - r_x)

        # Compute the distance between the robot and the edge of the ball.
        distance = math.sqrt((b_x - r_x) ** 2 + (b_y - r_y) ** 2) - (BALL_RADIUS + PR2_RADIUS)
        
        nr_x = r_x + (distance * math.cos(angle))
        nr_y = r_y + (distance * math.sin(angle))

        
        

        self.move_plan(angle,nr_x,nr_y,tolerance=0.1, dico = PickBall.pick_ball_pos)
        


        self.execute_plan()
        
        moveGroup.attach_object(self.ball.name, link_name= 'r_gripper_r_finger_link')
        CURRENT_PR2_BALL = self.ball
        
        self.name = "raiseball"
        self.set_target_join(PickBall.raise_ball_pos,tolerance = 0.1)
        self.execute_plan()
        
        
        print(self.ball.name+ " attached to robot")
        

    def isPreconditionValid(self):
        global CURRENT_PR2_ROOM
        global CURRENT_PR2_BALL

        return (self.ball.currentRoom is CURRENT_PR2_ROOM) and (CURRENT_PR2_BALL is None)


#####################################################################################
# CLASS: DropBall                                                                   #
# Drop a ball action. The robot drop the ball at the current position.              #
#####################################################################################
class DropBall(Action):
    targetdetach = {
                            "l_shoulder_pan_joint" : -0.08,
                            "r_shoulder_pan_joint" : 0.08,
                            "l_shoulder_lift_joint" : 1.2963,
                            "r_shoulder_lift_joint" : 1.2963,
                            "l_upper_arm_roll_joint" : 0,
                            "r_upper_arm_roll_joint" : 0,
                            "l_elbow_flex_joint" : -0.39,
                            "r_elbow_flex_joint" : -0.39,
                            "l_forearm_roll_joint" : 0,
                            "r_forearm_roll_joint" : 0,
                            "l_wrist_flex_joint" : -0.1,
                            "r_wrist_flex_joint" : -0.1,
                            "l_wrist_roll_joint" : 0.8,
                            "r_wrist_roll_joint" : -0.8

                            }
    def __init__(self, moveGroup, ball):
        Action.__init__(self, moveGroup)
        self.ball = ball
        self.name = "dropball"

    def execute(self):
        global CURRENT_PR2_BALL
        global CURRENT_PR2_ROOM

        self.print_action()

        self.valid_precondition()
        
        self.set_target_join(DropBall.targetdetach)
        self.execute_plan()

        moveGroup.detach_object(self.ball.name)
        CURRENT_PR2_BALL = None
        self.ball.currentRoom = CURRENT_PR2_ROOM

        print(self.ball.name+ " dropped to the ground")

    def isPreconditionValid(self):
        global CURRENT_PR2_BALL

        return CURRENT_PR2_BALL is self.ball
        

#####################################################################################
# CLASS: KickBall                                                                   #
# Kick a ball action. The robot kicks the ball at the current position.              #
#####################################################################################
class KickBall(Action):
    def __init__(self, moveGroup, ball, room, hand = True):
        Action.__init__(self, moveGroup)
        self.ball = ball
        self.room = room
        self.name = "kickBall"
        self.hand = hand

    def execute(self):
        global CURRENT_PR2_BALL
        global CURRENT_PR2_ROOM

        self.print_action()

        #self.valid_precondition()
        
        ro_y, ro_x = self.room.y_coord, self.room.x_coord
        
        r_y, r_x, r_angle = self.get_robot_position()
        

        if not self.hand:
            b_x, b_y = self.get_object()
        
            distance = (BALL_RADIUS/1.25 + PR2_RADIUS)
            dy = ro_y - b_y
            dx = ro_x - b_x
            
            print(b_x, b_y)
            print(r_y, r_x)
            angle = math.atan2(dy, dx) 
            angle_o = ((angle + math.pi) % (2*math.pi))

            nb_x = b_x  + distance * math.cos(angle_o)
            nb_y = b_y  + distance * math.sin(angle_o)
                
    
        
                       
            self.move_plan(angle,nb_x,nb_y,tolerance=0.0)
        
        else:
            CURRENT_PR2_BALL = None
            moveGroup.detach_object(self.ball.name)
            angle = math.atan2(ro_y - r_y, ro_x - r_x)
            b_x, b_y = self.get_object()


            self.move_plan(angle,r_x,r_y,tolerance=0.0)
            
        self.execute_plan()
        
        
        print("shoot with the angle : ", angle * 180 / math.pi)

        ballPose = geometry_msgs.msg.PoseStamped()
        ballPose.header.frame_id = moveGroup.get_planning_frame()
        nbt_x = b_x + 2*math.cos(angle)
        nbt_y = b_y + 2*math.sin(angle)
        self.add_ball_scene(nbt_x,nbt_y, name=self.ball.name)
        
        self.ball.currentRoom = self.room
        print(self.ball.name + " entering " + self.room.name + " from " + ("hand" if self.hand else "foot"))


    def isPreconditionValid(self):
        global CURRENT_PR2_BALL
        global CURRENT_PR2_ROOM
        if self.hand:
            if self.ball != CURRENT_PR2_BALL:
                return False
        else:
            if CURRENT_PR2_ROOM != self.ball.currentRoom:
                return False
        return True
            

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
    scene.add_sphere("b1", ballPose1, BALL_RADIUS)

    ballPose2 = geometry_msgs.msg.PoseStamped()
    ballPose2.header.frame_id = moveGroup.get_planning_frame()
    ballPose2.pose.position.x = -4.0
    ballPose2.pose.position.y = 4.0
    ballPose2.pose.position.z = BALL_RADIUS 
    scene.add_sphere("b2", ballPose2, BALL_RADIUS)

    ballPose3 = geometry_msgs.msg.PoseStamped()
    ballPose3.header.frame_id = moveGroup.get_planning_frame()
    ballPose3.pose.position.x = 4.0
    ballPose3.pose.position.y = 0.0
    ballPose3.pose.position.z = BALL_RADIUS 
    scene.add_sphere("b3", ballPose3, BALL_RADIUS)

    ballPose4 = geometry_msgs.msg.PoseStamped()
    ballPose4.header.frame_id = moveGroup.get_planning_frame()
    ballPose4.pose.position.x = -4.0
    ballPose4.pose.position.y = 0.0
    ballPose4.pose.position.z = BALL_RADIUS 
    scene.add_sphere("b4", ballPose4, BALL_RADIUS)

    ballPose5 = geometry_msgs.msg.PoseStamped()
    ballPose5.header.frame_id = moveGroup.get_planning_frame()
    ballPose5.pose.position.x = 4.0
    ballPose5.pose.position.y = -4.0
    ballPose5.pose.position.z = BALL_RADIUS 
    scene.add_sphere("b5", ballPose5, BALL_RADIUS)

    ballPose6 = geometry_msgs.msg.PoseStamped()
    ballPose6.header.frame_id = moveGroup.get_planning_frame()
    ballPose6.pose.position.x = -4.0
    ballPose6.pose.position.y = -4.0
    ballPose6.pose.position.z = BALL_RADIUS 
    scene.add_sphere("b6", ballPose6, BALL_RADIUS)

# -------------------------------------------------------------------------------
# isPlanValid
#   Check if a plan is valid.
# return: True if plan is valid, False otherwise.
# -------------------------------------------------------------------------------
def isPlanValid(plan):
    # A valid plan will contains a trajectory.
    return plan.joint_trajectory.points

##### PART 3 ######

import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
from rosplan_dispatch_msgs.srv import DispatchService
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback
from rosplan_knowledge_msgs.srv import GetAttributeService


# PDDL conversion
def pddl_to_rosplan(pddl_action, moveGroup, rooms, balls):
    if pddl_action[0] == "move": return Move(moveGroup, rooms[pddl_action[2]])
    elif pddl_action[0] == "pick_ball": return PickBall(moveGroup, balls[pddl_action[1]])
    elif pddl_action[0] == "drop_ball": return DropBall(moveGroup, balls[pddl_action[1]])
    elif pddl_action[0] == "kick_hand": return KickBall(moveGroup, balls[pddl_action[3]], rooms[pddl_action[2]])
    elif pddl_action[0] == "kick_ground": return KickBall(moveGroup, balls[pddl_action[3]], rooms[pddl_action[2]], hand=False)

    return None

Move
# Subscribers
def callback(msg):
    print("-- Begin plan --")
    print(msg)
    print("-- End plan --")

def dispatch(msg, moveGroup, rooms, balls):
    res = list()
    res.append(msg.name)

    for param in msg.parameters:
        res.append(param.value)
    
    print("Executing action (" + ' '.join(res) + ')')
    action = pddl_to_rosplan(res, moveGroup, rooms, balls)
    if action is not None:
        action.execute()

def callback_feedback(msg):
    if msg.status == 2:
        print("action achieved")
    elif msg.status == 1:
        print("action enabled")
    elif msg.status == 10:
        print("action failed")


# Publishers
def disp_world(state):
    res = ""
    for attribute in state.attributes:
        res += "( " + attribute.attribute_name + " "
        for elt in attribute.values:
            res += elt.key + " " + elt.value + " "
        res += ")\n"
    print(res)


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

        HANDS = False
        # Instantiate a MoveGroupCommander object.
        # This enable us to generate plans for the selected planning group.

        if HANDS:
            moveGroup = moveit_commander.MoveGroupCommander("whole_body")
        else:
            moveGroup = moveit_commander.MoveGroupCommander("base")

        # Set the maximum amount of time to use when planning (in seconds).
        # If no plan is found before the time limit, the planning fails.
        moveGroup.set_planning_time(10.0)
        
        
        rospy.sleep(5)

        # Clear the environment.
        scene.remove_world_object()

        # Generate the environment.
        generateEnvironment(scene, moveGroup)
        
        
        rospy.sleep(1)

        rooms = {
            "r1_l": Room("r1_l", 4.0, 3.0),
            "r1_r": Room("r1_r", 3.5, 1.25),
            "r2_r": Room("r2_r", 4.0, -3.0),
            "r2_l": Room("r2_l", 3.5, -1.25),
            "r3": Room("r3", 0.0, 3.0),
            "r4": Room("r4", 0.0, 0.0),
            "r5": Room("r5", 0.0, -3.0),
            "r6_l": Room("r6_l", -4.0, 3.0),
            "r6_r": Room("r6_r", -3.5, 1.25),
            "r7_r": Room("r7_r", -4.0, -3.0),
            "r7_l": Room("r7_l", -3.5, -1.25)
        }

        balls = {
            "b1": Ball("b1", rooms["r1_l"]),
            "b2": Ball("b2", rooms["r2_r"]),
            "b3": Ball("b3", rooms["r3"]),
            "b4": Ball("b4", rooms["r5"]),
            "b5": Ball("b5", rooms["r6_l"]),
            "b6": Ball("b6", rooms["r7_r"])
        }

        CURRENT_PR2_ROOM = rooms["r4"]


        ## ROSPlan server calls
        callback_dispatch = lambda msg: dispatch(msg, moveGroup, rooms, balls)

        sub_plan = rospy.Subscriber("/rosplan_planner_interface/planner_output", String, callback)
        sub_dispatch = rospy.Subscriber("/rosplan_plan_dispatcher/action_dispatch", ActionDispatch, callback_dispatch)
        sub_feedback = rospy.Subscriber("/rosplan_plan_dispatcher/action_feedback", ActionFeedback, callback_feedback)

        rospy.wait_for_service('/rosplan_knowledge_base/state/propositions')
        rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
        rospy.wait_for_service('/rosplan_planner_interface/planning_server')
        rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
        rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')

        state = rospy.ServiceProxy('/rosplan_knowledge_base/state/propositions', GetAttributeService)
        initial_world_state = state()
        disp_world(initial_world_state)

        problem_generation = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', Empty)
        if not problem_generation():
            rospy.logerr("KCL: (%s) No problem was generated!" % rospy.get_name())

        planning = rospy.ServiceProxy('/rosplan_planner_interface/planning_server', Empty)
        pi_res = planning()

        # Plan dispatcher
        parser = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan', Empty)
        parse_res = parser()


        dispatcher = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan', DispatchService)
        if not dispatcher():
            rospy.logerr("KCL: (%s) Cannot dispatch plan!" % rospy.get_name())


        # Listening loop
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            rate.sleep()

        # Shut down moveit_commander.
        moveit_commander.roscpp_shutdown()

    except rospy.ROSInterruptException:
        pass
