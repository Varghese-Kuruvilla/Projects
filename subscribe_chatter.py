#Script to subscribe to the /chatter topic
import rospy
from std_msgs.msg import String
import ast

def callback_msg(box_pose):
    data = box_pose.data
    #print("data:",data)
    data_ls = ast.literal_eval(data)
    for (i,element) in enumerate(data_ls):
        print("Box{}".format(i))
        print(element)
        print(type(element))

def subs_chatter():
    rospy.init_node('list_1',anonymous=True)
    msg = rospy.Subscriber('/chatter',String,callback_msg)
    rospy.spin()


if __name__ == '__main__':
    subs_chatter();
