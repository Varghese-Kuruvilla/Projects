#Script to subscribe to the /chatter topic
import rospy
from std_msgs.msg import String

def callback_msg(msg):
    data = msg.data
    data = list(data.split(':'))
    print("data:",data)
    for element in data:
        print("Element:",element)
        print("Type(element):",type(element))

def subs_chatter():
    rospy.init_node('list_1',anonymous=True)
    msg = rospy.Subscriber('/chatter',String,callback_msg)
    rospy.spin()


if __name__ == '__main__':
    subs_chatter();
