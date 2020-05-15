#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
#from ground_truth_odom.msg import *
from geometry_msgs.msg import Twist
#from eurobench_worldstate_provider.msg import EurobenchWorldState
from urdf_parser_py.urdf import URDF
from std_msgs.msg import Float64
#from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
#from pykdl_utils.kdl_kinematics import KDLKinematics
from eurobench_state_collector.srv import MadrobDoorDummy

import sys

VERBOSE=True
#VERBOSE=False

class eurobench_state_collector:
     def __init__(self):

          # ################### where i am goung to publish ##################
          #self.eb_ws_pub = rospy.Publisher('eurobench_state_collector',
          #                                 EurobenchWorldState, queue_size=1)
          self.door_pub = rospy.Publisher('/madrob/preprocessed_data/passage/door',
                                           Float64, queue_size=1)




          # ################## where i am going to be subscribed #############
          self.image_camera = rospy.Subscriber("sensor_msgs/Image", Image,
                                               self.image_callback, queue_size=1)
          if VERBOSE:
               print ("subcribed on sensor_msgs.Image")


          self.ee_pose_wrt_com = rospy.Subscriber("ground_truth_odom", Twist,
                                               self.com_pose_callback, queue_size=1)
          if VERBOSE:
               print ("subcribed on groud truth odom")

         # self.ee_pose = rospy.Subscriber("geometry_msgs/Twist", Twist,
         #                                      self.ee_pose_callback, queue_size=1)

          if VERBOSE:
               print ("subcribed on joint states")
          self.ee_pose = rospy.Subscriber("sensor_msgs/JointState", Twist,
                                               self.ee_pose_callback, queue_size=1)


          if VERBOSE:
               print ("setup of the kdl kinematics")
               # pykdl_utils setup
          #compute_forward_kinematics()



     def compute_forward_kinematics():
 #          robot_urdf = URDF.from_xml_string("<robot name='reemc'></robot>") NON FUNZIONA!
           print("forward kinematics")
           robot_urdf = URDF.from_parameter_server()

           print(robot_urdf)
           robot_urdf = URDF.from_xml_file("reemc_simulation/reemc_gazebo/models/reemc_full/model.urdf")
           kdl_kin = KDLKinematics(robot_urdf, 'arm_right_1_joint', 'arm_right_7_joint')
           tree = kdl_tree_from_urdf_model(robot_urdf)
           print (tree.getNrOfSegments())
           chain = tree.getChain( 'arm_right_1_joint', 'arm_right_7_joint')
           print (chain.getNrOfJoints())

     #     robot = URDF.from_parameter_server()
      #    tree = kdl_tree_from_urdf_model(robot)
      #    print (tree.getNrOfSegments())
      #    chain = tree.getChain( 'arm_right_7_joint', 'arm_right_1_joint')
      #    print (chain.getNrOfJoints())


     def com_pose_callback(self, ros_data):
          msg = EurobenchWorldState()
     #     msg.com_pose.linear =   #qui ci metto graound truth odom!! (pose and angle)
     def ee_pose_callback(self, ros_data):
          msg = EurobenchWorldState()

          msg.ee_pose_wrt_com.linear.x = 1.0
          msg.ee_pose_wrt_com.linear.y = 1.0
          msg.ee_pose_wrt_com.linear.x = 1.0
          self.eb_ws_pub.publish(msg)

     def image_callback(self, ros_data):
          if VERBOSE :
               print ('received image of type: "%s"' % ros_data.format)

          msg = EurobenchWorldState()

          msg.com_pose.linear.x = 1.0
          msg.com_pose.linear.y = 1.0
          msg.com_pose.linear.x = 1.0
          self.eb_ws_pub.publish(msg)



def talker(ebws):
#     pub = rospy.Publisher('eurobench_state_collector', EurobenchWorldState, queue_size=1)
     #ebws.
#     self.door_pub = rospy.Publisher('/madrob/preprocessed_data/passage/door',
#                                           Float64, queue_size=1)

     r = rospy.Rate(10) #10hz
#     msg = EurobenchWorldState()
#     msg.acquisition_from_camera = "ROS User"
#     msg.com_pose.linear.x = 1.0
#     msg.com_pose.linear.y = 1.0
#     msg.com_pose.linear.x = 1.0

     print ("initialized")

#     while not rospy.is_shutdown():
#         rospy.loginfo(msg)
#         pub.publish(msg)
#         r.sleep()

     msg = Float64()
     msg = 1.0
     while not rospy.is_shutdown():
         #rospy.loginfo(msg)
         ebws.door_pub.publish(msg)
         r.sleep()


def callback(data):
    rospy.loginfo("%s is age: %d" % (data.name, data.age))
    print ("initialized")

def handle_madrob_door_dummy_srv(req):
    print("Handled dummy service")

def listener():
#    rospy.init_node('eurobench_worldstate_provider', anonymous=True)
    image_camera = rospy.Subscriber("sensor_msgs/Image", Image, callback)

      # spin() simply keeps python from exiting until this node is stopped
#    rospy.spin()
#    print ("listener 4")


def main(args):
     ebws =  eurobench_state_collector()
     rospy.init_node('eurobench_state_collector', anonymous=True) #CHECK IF REMOVE 'PROVIDER'

     s = rospy.Service('/madrob/door/set_mode', MadrobDoorDummy, handle_madrob_door_dummy_srv) 
     print ("service madrob/door initialized in eurobench_state_collector_node")    

     try:
         talker(ebws)
         rospy.spin()
     except KeyboardInterrupt:
           print ("Shutting down ROS eurobench_state_collector module")


if __name__ == '__main__':
     main(sys.argv)
