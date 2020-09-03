#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from sensor_msgs.msg import JointState
#from ground_truth_odom.msg import *
from geometry_msgs.msg import Twist
#from eurobench_worldstate_provider.msg import EurobenchWorldState
from urdf_parser_py.urdf import URDF
from std_msgs.msg import Float64
#from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
#from pykdl_utils.kdl_kinematics import KDLKinematics
from eurobench_state_collector.srv import MadrobDoorDummy
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetModelProperties
from gazebo_msgs.srv import GetJointProperties
from madrob_srvs.srv import *
from madrob_msgs.msg import Passage
from eurobench_bms_msgs_and_srvs.srv import *

import roslaunch
import subprocess
import rospkg
import os
import numpy as np



import sys

VERBOSE=True
#VERBOSE=False

class eurobench_state_collector:
    def __init__(self):

        self.cw_left = np.array([None, None, None, None])
        self.cw_right = np.array([None, None, None, None]) 
        self.ccw_left = np.array([None, None, None, None])
        self.ccw_right = np.array([None, None, None, None])
        
        self.current_benchmark_name = None
        self.current_door_opening_side = None
        self.current_robot_approach_side = None
        
        self.old_benchmark_name = None
        self.old_door_opening_side = None
        self.old_robot_approach_side = None
        
        self.startSim()
        
        # ################### where i am goung to publish ##################
          
        #self.eb_ws_pub = rospy.Publisher('eurobench_state_collector',
        #                                 EurobenchWorldState, queue_size=1)
        
        self.door_pub = rospy.Publisher('/madrob/preprocessed_data/passage/door',
                                           Float64, queue_size=1)
                                           
        self.door_handle_pub = rospy.Publisher('/madrob/preprocessed_data/passage/handle',
                                           Float64, queue_size=1)
                                           
        self.cw_left_pub = rospy.Publisher('/madrob/passage/cw_left',
                                        Passage, queue_size=1)                             
        self.cw_right_pub = rospy.Publisher('/madrob/passage/cw_right',
                                        Passage, queue_size=1)        
        self.ccw_left_pub = rospy.Publisher('/madrob/passage/ccw_left', 
                                        Passage, queue_size=1)                             
        self.ccw_right_pub =rospy.Publisher('/madrob/passage/ccw_right',
                                        Passage, queue_size=1)                             
                   

        # ################## where i am going to be subscribed #############
        
        self.distance_sens_front_0 = rospy.Subscriber("/sensor/base_ir_front_0", Range,
          									   self.cw_left_callback, queue_size=1)
          
        self.distance_sens_front_1 = rospy.Subscriber("/sensor/base_ir_front_1", Range,
          									   self.cw_left_callback, queue_size=1)

        self.distance_sens_front_2 = rospy.Subscriber("/sensor/base_ir_front_2", Range,
          									   self.cw_left_callback, queue_size=1)

        self.distance_sens_front_3 = rospy.Subscriber("/sensor/base_ir_front_3", Range,
          									   self.cw_left_callback, queue_size=1)
          									   

        self.distance_sens_front_4 = rospy.Subscriber("/sensor/base_ir_front_4", Range,
          									   self.cw_right_callback, queue_size=1)

        self.distance_sens_front_5 = rospy.Subscriber("/sensor/base_ir_front_5", Range,
          									   self.cw_right_callback, queue_size=1)

        self.distance_sens_front_6 = rospy.Subscriber("/sensor/base_ir_front_6", Range,
          									   self.cw_right_callback, queue_size=1)
          							
        self.distance_sens_front_7 = rospy.Subscriber("/sensor/base_ir_front_7", Range,
          									   self.cw_right_callback, queue_size=1)
          									   

          									   

          									   
        self.distance_sens_back_0 = rospy.Subscriber("/sensor/base_ir_back_0", Range,
          									   self.ccw_left_callback, queue_size=1)
          
        self.distance_sens_back_1 = rospy.Subscriber("/sensor/base_ir_back_1", Range,
          									   self.ccw_left_callback, queue_size=1)
          									   
        self.distance_sens_back_2 = rospy.Subscriber("/sensor/base_ir_back_2", Range,
          									   self.ccw_left_callback, queue_size=1)
          									   
        self.distance_sens_back_3 = rospy.Subscriber("/sensor/base_ir_back_3", Range,
          									   self.ccw_left_callback, queue_size=1)          									   
        self.distance_sens_back_4 = rospy.Subscriber("/sensor/base_ir_back_4", Range,
          									   self.ccw_right_callback, queue_size=1)    
          									   
        self.distance_sens_back_5 = rospy.Subscriber("/sensor/base_ir_back_5", Range,
        									   self.ccw_right_callback, queue_size=1)      
        									   
        self.distance_sens_back_6 = rospy.Subscriber("/sensor/base_ir_back_6", Range,
        									   self.ccw_right_callback, queue_size=1)          									   
        self.distance_sens_back_7 = rospy.Subscriber("/sensor/base_ir_back_7", Range,
        									   self.ccw_right_callback, queue_size=1)          							
          									   
        if VERBOSE:
             print ("subcribed on sensor_distances")
          
          
        self.image_camera = rospy.Subscriber("sensor_msgs/Image", Image,
                                             self.image_callback, queue_size=1)
        if VERBOSE:
             print ("subcribed on sensor_msgs.Image")


#        self.ee_pose_wrt_com = rospy.Subscriber("ground_truth_odom", Twist,
#                                             self.com_pose_callback, queue_size=1)
#        if VERBOSE:
#             print ("subcribed on groud truth odom")

         # self.ee_pose = rospy.Subscriber("geometry_msgs/Twist", Twist,
         #                                      self.ee_pose_callback, queue_size=1)

#        if VERBOSE:
#             print ("subcribed on joint states")
#        self.ee_pose = rospy.Subscriber("sensor_msgs/JointState", Twist,
#                                             self.ee_pose_callback, queue_size=1)


        if VERBOSE:
             print ("setup of the kdl kinematics")
             # pykdl_utils setup
          #compute_forward_kinematics()

    def startSim(self):
        
        #door:=simple direction:=pull gzpose:="-x -1.0 -y 0.4 -z 0.86 -R 0.0 -P 0.0 -Y 3.1416"
        package = 'eurobench_reemc_door'
        launch_file = 'reemc_door.launch'
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        
        #roslaunch.core.Node(package, launch_file, args='door:=simple direction:=pull gzpose:="-x 1.0 -y 0.4 -z 0.86 -R 0.0 -P 0.0 -Y 0"')
                 
        launch_file = os.path.join(rospkg.RosPack().get_path(package), 'launch', launch_file)
        sys.argv = [ 'door:=wind', 'direction:=push']
        

        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
        self.launch.start()
        # https://answers.ros.org/question/42849/how-to-launch-a-launch-file-from-python-code/
        # http://wiki.ros.org/roslaunch/API%20Usage
        # https://answers.ros.org/question/237109/passing-arguments-to-launch-file-from-python-api/
        # https://answers.ros.org/question/10493/programmatic-way-to-stop-roslaunch/
        
        '''
        package = 'eurobench_reemc_door'
        launch_file = 'reemc_door.launch'
        
        launch_file = os.path.join(rospkg.RosPack().get_path(package), 'launch', launch_file)
        
        print launch_file
        cli_args = [launch_file,'door:=simple']
        roslaunch_args = cli_args[1:]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        '''
        
        

    def are_ranges_complete(self, ranges):
        for single_range in ranges:
            if single_range == None:
                return False
        return True
        
    def null_the_ranges(self, ranges):
        for single_range in ranges:
            single_range = None
        
    def sensor_identifier(self, ros_data):
        return int(ros_data.header.frame_id[-1])%4

    def publish_strips(self, tmp_ranges, ros_data, ranges_pub):
        sensorID = self.sensor_identifier(ros_data)
        tmp_ranges[sensorID] = ros_data
        if (self.are_ranges_complete(tmp_ranges)):
            msg = Passage()
            #msg.STATUS_OK = 1
            msg.ranges[0] = tmp_ranges[0]
            msg.ranges[1] = tmp_ranges[1]
            msg.ranges[2] = tmp_ranges[2]
            msg.ranges[3] = tmp_ranges[3]
            ranges_pub.publish(msg)
            self.null_the_ranges(tmp_ranges)

    def cw_left_callback(self, ros_data):
        self.publish_strips(self.cw_left, ros_data, self.cw_left_pub)

    def cw_right_callback(self, ros_data):
        self.publish_strips(self.cw_right, ros_data, self.cw_right_pub)
                
    def ccw_left_callback(self, ros_data):
        self.publish_strips(self.ccw_left, ros_data, self.ccw_left_pub)        
                  
    def ccw_right_callback(self, ros_data):
        self.publish_strips(self.ccw_right, ros_data, self.ccw_right_pub)        


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
#    def ee_pose_callback(self, ros_data):
#        msg = EurobenchWorldState()

#        msg.ee_pose_wrt_com.linear.x = 1.0
#        msg.ee_pose_wrt_com.linear.y = 1.0
#        msg.ee_pose_wrt_com.linear.x = 1.0
#        self.eb_ws_pub.publish(msg)

    def image_callback(self, ros_data):
        if VERBOSE :
            print ('received image of type: "%s"' % ros_data.format)
        msg = EurobenchWorldState()

        msg.com_pose.linear.x = 1.0
        msg.com_pose.linear.y = 1.0
        msg.com_pose.linear.x = 1.0
        self.eb_ws_pub.publish(msg)
        
        
      


def talker(ebws):
    if VERBOSE:
        print ("subcribed on sensor_distances")

    r = rospy.Rate(10) #10hz

    msg = Float64()
    
    while not rospy.is_shutdown():
        msg = getDoorAperture()
        ebws.door_pub.publish(msg)

        msg_handle = getHandlePosition()
        ebws.door_handle_pub.publish(msg_handle)

        retrieveBenchmarkConfiguration(ebws)
        if benchmarkConfigurationHasChanged(ebws):
            if ebws.current_door_opening_side is not None: 
                restartSim(ebws)

        r.sleep()


def callback(data):
    rospy.loginfo("%s is age: %d" % (data.name, data.age))
    print ("initialized")

def handle_madrob_door_dummy_srv(req):
    print("Handled dummy service")
    return SetDoorControllerModeResponse(True, "")


def listener(self):
#    rospy.init_node('eurobench_worldstate_provider', anonymous=True)
    image_camera = rospy.Subscriber("sensor_msgs/Image", Image, callback)
    
  #  self.distance_sens0 = rospy.Subscriber("/sensor/base_ir_front_0", Float64,
  #        									   self.ds_callback, queue_size=1)
          
  #  self.distance_sens1 = rospy.Subscriber("/sensor/base_ir_front_1", Float64,
  #        									   self.ds_callback, queue_size=1)
    
    print('---------- sensors reading ---------')

      # spin() simply keeps python from exiting until this node is stopped
#    rospy.spin()
#    print ("listener 4")

def getDoorAperture(): 
    try:
        get_model_properties = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
    except rospy.ServiceException, e:
        print "ServiceProxy failed: %s"%e
        exit(0)
    model_prop = get_model_properties("door_simple")
    try:
        get_door_joint_props = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
    except rospy.ServiceException, e:
        print "ServiceProxy failed: %s"%e
        exit(0)
    if VERBOSE: print('---------- door aperture ---------')
    joint_prop = get_door_joint_props('joint_frame_door')
    if VERBOSE: print(joint_prop.position[0])
   
    return joint_prop.position[0]


def getHandlePosition():
    try:
        get_model_properties = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
    except rospy.ServiceException, e:
        print "ServiceProxy failed: %s"%e
        exit(0)
    model_prop = get_model_properties("door_simple")
    try:
        get_door_joint_props = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
    except rospy.ServiceException, e:
        print "ServiceProxy failed: %s"%e
        exit(0)
    joint_prop_handle = get_door_joint_props('joint_door_lever')
    if VERBOSE: print('---------- handle position ---------')
    if VERBOSE: print(joint_prop_handle.position[0])
    return joint_prop_handle.position[0]


def retrieveBenchmarkConfiguration(ebws):    # Based on the currently selected benchmark type
    try:
        get_benchmark_params = rospy.ServiceProxy('madrob/gui/benchmark_params', MadrobBenchmarkParams)
    except rospy.ServiceException, e:
        print "ServiceProxy failed: %s"%e
        exit(0)
    response = get_benchmark_params()
    ebws.current_benchmark_name = response.benchmark_type
    ebws.current_door_opening_side = response.door_opening_side
    ebws.current_robot_approach_side = response.robot_approach_side
    if VERBOSE:
        print [ebws.current_benchmark_name , ebws.current_door_opening_side , ebws.current_robot_approach_side]


def benchmarkConfigurationHasChanged(ebws):
    if (ebws.current_benchmark_name != ebws.old_benchmark_name 
    or ebws.current_door_opening_side != ebws.old_door_opening_side
    or ebws.current_robot_approach_side != ebws.old_robot_approach_side):
        ebws.old_benchmark_name = ebws.current_benchmark_name
        ebws.old_door_opening_side = ebws.current_door_opening_side
        ebws.old_robot_approach_side = ebws.current_robot_approach_side
        if VERBOSE: 
            print("PARAMETERS ARE CHANGED")
            
        return True;
    return False;
         
    
def restartSim(ebws):
    print("***** RESTARTING SIMULATION FOR PARAMETERS CHANGE *****")
    ebws.launch.shutdown()
    arg0 = ebws.current_benchmark_name
    arg1 = "direction:=pull" if ebws.current_door_opening_side == "CW" else "direction:=push"
    arg2 = "side1" if ebws.current_robot_approach_side == "CW" else "side2"
    
    package = 'eurobench_reemc_door'
    launch_file = 'reemc_door.launch'
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
#roslaunch.core.Node(package, launch_file, args='door:=simple direction:=pull gzpose:="-x 1.0 -y 0.4 -z 0.86 -R 0.0 -P 0.0 -Y 0"')                 
    launch_file = os.path.join(rospkg.RosPack().get_path(package), 'launch', launch_file)
    sys.argv = [ 'door:=simple', arg1]
    ebws.launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
    ebws.launch.start()

    


def main(args):
     ebws =  eurobench_state_collector()
     rospy.init_node('eurobench_state_collector', anonymous=True) #CHECK IF REMOVE 'PROVIDER'
     
     listener(ebws)

     s = rospy.Service('/madrob/door/set_mode', SetDoorControllerMode, handle_madrob_door_dummy_srv) 
     print ("service madrob/door initialized in eurobench_state_collector_node")    

     try:
         talker(ebws)
         rospy.spin()
     except KeyboardInterrupt:
           print ("Shutting down ROS eurobench_state_collector module")



if __name__ == '__main__':
     main(sys.argv)
