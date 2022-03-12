import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
from rosplan_dispatch_msgs.srv import DispatchService
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback # EsterelPlan


rospy.init_node("node1")
rospy.loginfo('start')


# Subscribers
def callback(msg):
    print(msg)

sub_plan = rospy.Subscriber("/rosplan_planner_interface/planner_output", String, callback)
# sub_parse = rospy.Subscriber("/rosplan_planner_interface/plan_topic", EsterelPlan, callback)
sub_dispatch = rospy.Subscriber("/rosplan_plan_dispatcher/action_dispatch", ActionDispatch, callback)
sub_feedback = rospy.Subscriber("/rosplan_plan_dispatcher/action_feedback", ActionFeedback, callback)


# Publishers
rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
print("problem generation: started")
rospy.wait_for_service('/rosplan_planner_interface/planning_server')
print("planning server: started")

problem_generation = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', Empty)
if not problem_generation():
    rospy.logerr("KCL: (%s) No problem was generated!" % rospy.get_name())

planning = rospy.ServiceProxy('/rosplan_planner_interface/planning_server', Empty)
pi_res = planning()

# Plan dispatcher
parser = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan', Empty)
parse_res = parser()

dispatcher = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan', DispatchService)
disp_res = dispatcher()


# Listening loop
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    rate.sleep()



# IF action name matches THEN:
#   check action for malformed parameters
#   update knowledge base (at start effects)
#   publish (action enabled)
#   success = concreteCallback()
#   IF success THEN:
#       update knowledge base (at end effects)
#       publish (action achieved)
#   ELSE
#       publish (action failed)
#   RETURN success
