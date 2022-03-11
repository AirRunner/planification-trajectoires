import rospy
import std_msgs.msg


def callback(msg):
    print(msg)


rospy.init_node("hoge")
rospy.loginfo('start')

sub = rospy.Subscriber("action_feedback", std_msgs.msg.String, callback)

#pub = rospy.Publisher("action_feedback", std_msgs.msg.Int32, queue_size=10)

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    #pub.publish(3)
    #print("a")
    rate.sleep()