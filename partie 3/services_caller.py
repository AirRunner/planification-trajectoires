import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty


rospy.init_node("node1")
rospy.loginfo('start')


# Subscriber
def callback(msg):
    print(msg)

sub = rospy.Subscriber("/rosplan_planner_interface/planner_output", String, callback)


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


# Listening loop
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    rate.sleep()
