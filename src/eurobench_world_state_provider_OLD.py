#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from eurobench_worldstate_provider.msg import EurobenchWorldState


def talker():
     pub = rospy.Publisher('eurobench_worldstate', EurobenchWorldState, queue_size=1)
     rospy.init_node('eurobench_worldstate_provider', anonymous=True)

     r = rospy.Rate(10) #10hz
     msg = EurobenchWorldState()
#     msg.acquisition_from_camera = "ROS User"
     msg.com_pose.linear.x = 1.0
     msg.com_pose.linear.y = 1.0
     msg.com_pose.linear.x = 1.0

     print ("initialized")

     while not rospy.is_shutdown():
         rospy.loginfo(msg)
         pub.publish(msg)
         r.sleep()

def image_callback(msg):
    image_camera =

def callback(data):
    rospy.loginfo("%s is age: %d" % (data.name, data.age))
    print ("initialized")

def listener():
#    rospy.init_node('eurobench_worldstate_provider', anonymous=True)
    image_camera = rospy.Subscriber("sensor_msgs/Image", Image, callback)

      # spin() simply keeps python from exiting until this node is stopped
#    rospy.spin()
#    print ("listener 4")


if __name__ == '__main__':
    print ("before listener ")
    listener()
    print ("initialized")

    try:
         talker()
    except rospy.ROSInterruptException: pass
