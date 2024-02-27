#!/usr/bin/env python

# import rospy
# from std_msgs.msg import String
import persistqueue

def publish(topic, message):
    q = persistqueue.SQLiteQueue(topic, auto_commit=True)
    q.put(message)

    # pub = rospy.Publisher('chatter', String, queue_size=10)
    # rospy.init_node('talker', anonymous=True)
    # erat = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    #     hello_str = "hello world %s" % rospy.get_time()
    #     rospy.loginfo(hello_str)
    #     pub.publish(hello_str)
    #     rate.sleep()


if __name__ == '__main__':
    publish('vision', 'test')
