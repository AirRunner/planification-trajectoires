import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
from rosplan_dispatch_msgs.srv import DispatchService
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback
from rosplan_knowledge_msgs.srv import GetAttributeService, GetDomainOperatorService
from primitives_solution_en_cours_remplissage import Move, PickBall, DropBall, KickBall


rospy.init_node("node1")
rospy.loginfo('start')


# Feedback publisher
pub = rospy.Publisher("/rosplan_plan_dispatcher/action_feedback", ActionFeedback, queue_size=10)


# PDDL conversion
def pddl_to_rosplan(pddl_action, moveGroup):
    matches = {
        "move": Move(moveGroup, pddl_action[2]),
        "pick_ball": PickBall(moveGroup, pddl_action[2]),
        "drop_ball": DropBall(moveGroup, pddl_action[2]),
        "kick_hand": KickBall(moveGroup, pddl_action[4], pddl_action[3], hand=True),
        "kick_ground": KickBall(moveGroup, pddl_action[4], pddl_action[3], hand=False)
    }
    matches[pddl_action[0]].execute()


# Subscribers
def callback(msg):
    print("-- Begin plan --")
    print(msg)
    print("-- End plan --")

def dispatch(msg, moveGroup):
    res = list()
    res.append(msg.name)

    for param in msg.parameters:
        res.append(param.value)
    
    # TODO: call ROSPlan primitives
    pddl_to_rosplan(res, moveGroup)
    
    print("Executing action (" + ' '.join(res) + ')')

def feedback(msg, pub):
    #pub.publish(msg)
    if msg.status == 2:
        # Publish "action achieved"
        pass
    elif msg.status == 1:
        # Publish "action enabled"
        pass
    elif msg.status == 10:
        # Publish "action failed"
        pass

callback_dispatch = lambda msg: feedback(msg, moveGroup)
callback_feedback = lambda msg: feedback(msg, pub)

sub_plan = rospy.Subscriber("/rosplan_planner_interface/planner_output", String, callback)
sub_dispatch = rospy.Subscriber("/rosplan_plan_dispatcher/action_dispatch", ActionDispatch, callback_dispatch)
sub_feedback = rospy.Subscriber("/rosplan_plan_dispatcher/action_feedback", ActionFeedback, callback_feedback)


# Publishers
def disp_world(state):
    res = ""
    for attribute in state.attributes:
        res += "( " + attribute.attribute_name + " "
        for elt in attribute.values:
            res += elt.key + " " + elt.value + " "
        res += ")\n"
    print(res)

rospy.wait_for_service('/rosplan_knowledge_base/state/propositions')
rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
rospy.wait_for_service('/rosplan_planner_interface/planning_server')
rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')

operators = rospy.ServiceProxy('/rosplan_knowledge_base/domain/operators', GetDomainOperatorService)
op_res = operators()

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
