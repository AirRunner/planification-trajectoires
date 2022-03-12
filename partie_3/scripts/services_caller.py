import rospy
import std_msgs.msg
from std_srvs.srv import Empty


def callback(msg):
    print(msg)

rospy.init_node("node1")
rospy.loginfo('start')

sub = rospy.Subscriber("planner_output", std_msgs.msg.String, callback)

rospy.wait_for_service('/rosplan_planner_interface/planning_server')
rospy.wait_for_service('/rosplan_planner_interface/problem_generation_server')


problem_generation = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', Empty)
problem_generation()

dispatch_service = rospy.ServiceProxy('/rosplan_planner_interface/planning_server', Empty)
res = dispatch_service()

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    rate.sleep()
