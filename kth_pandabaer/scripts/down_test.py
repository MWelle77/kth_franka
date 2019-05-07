#!/usr/bin/python

import rospy
import tf.transformations
import numpy as np
import sys, select, termios, tty

from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState



EE_pose_baer = PoseStamped()
EE_pose_stein = PoseStamped()
initial_pose_found_baer = False
initial_pose_found_stein = False



def baer_state_callback(msg):
    initial_quaternion = \
        tf.transformations.quaternion_from_matrix(
            np.transpose(np.reshape(msg.O_T_EE,
                                    (4, 4))))
    initial_quaternion = initial_quaternion / np.linalg.norm(initial_quaternion)
    EE_pose_baer.pose.orientation.x = initial_quaternion[0]
    EE_pose_baer.pose.orientation.y = initial_quaternion[1]
    EE_pose_baer.pose.orientation.z = initial_quaternion[2]
    EE_pose_baer.pose.orientation.w = initial_quaternion[3]
    EE_pose_baer.pose.position.x = msg.O_T_EE[12]
    EE_pose_baer.pose.position.y = msg.O_T_EE[13]
    EE_pose_baer.pose.position.z = msg.O_T_EE[14]
    global initial_pose_found_baer
    initial_pose_found_baer = True

def stein_state_callback(msg):
    initial_quaternion = \
        tf.transformations.quaternion_from_matrix(
            np.transpose(np.reshape(msg.O_T_EE,
                                    (4, 4))))
    initial_quaternion = initial_quaternion / np.linalg.norm(initial_quaternion)
    EE_pose_stein.pose.orientation.x = initial_quaternion[0]
    EE_pose_stein.pose.orientation.y = initial_quaternion[1]
    EE_pose_stein.pose.orientation.z = initial_quaternion[2]
    EE_pose_stein.pose.orientation.w = initial_quaternion[3]
    EE_pose_stein.pose.position.x = msg.O_T_EE[12]
    EE_pose_stein.pose.position.y = msg.O_T_EE[13]
    EE_pose_stein.pose.position.z = msg.O_T_EE[14]
    global initial_pose_found_stein
    initial_pose_found_stein = True



def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key





if __name__ == "__main__":
    rospy.init_node("down_test_node")
    settings = termios.tcgetattr(sys.stdin)
    print("GOGOGOGO")

    state_sub_baer = rospy.Subscriber("pandabaer/franka_state_controller/franka_states",
                                 FrankaState, baer_state_callback)

    state_sub_stein = rospy.Subscriber("frankastein/franka_state_controller/franka_states",
                                 FrankaState, stein_state_callback)


    listener = tf.TransformListener()

    # Get initial pose for the interactive marker
    while not (initial_pose_found_stein and initial_pose_found_baer):
    	print(initial_pose_found_stein)
    	print(initial_pose_found_baer)
        rospy.sleep(1)
    state_sub_baer.unregister()
    state_sub_stein.unregister()
    print(initial_pose_found_stein)
    print(initial_pose_found_baer)

    print("INTI POSE FOUND!!")

    pose_pub_baer = rospy.Publisher(
        "/pandabaer/equilibrium_pose", PoseStamped, queue_size=10)

    pose_pub_stein = rospy.Publisher(
        "/frankastein/equilibrium_pose", PoseStamped, queue_size=10)

    while(1):
    	# pose publisher
        EE_pose_stein.header.frame_id = "/frankastein/panda_link0"
        EE_pose_stein.header.stamp = rospy.Time(0)
        pose_pub_stein.publish(EE_pose_stein)
        EE_pose_baer.header.frame_id = "/pandabaer/panda_link0"
        EE_pose_baer.header.stamp = rospy.Time(0)
        pose_pub_baer.publish(EE_pose_baer)

    	#rospy.sleep(0.5)
        key = getKey()
        #print(key)
        if (key == '\x03'):
            break

        if key == 's':
            print(key)
            EE_pose_baer.pose.position.z = EE_pose_baer.pose.position.z - 0.01
            EE_pose_stein.pose.position.z = EE_pose_stein.pose.position.z - 0.01

        if key == 'w':
            print(key)
            EE_pose_baer.pose.position.z = EE_pose_baer.pose.position.z + 0.01
            EE_pose_stein.pose.position.z = EE_pose_stein.pose.position.z + 0.01

        if key == 'd':
            print(key)
            EE_pose_baer.pose.position.y = EE_pose_baer.pose.position.y + 0.01
            EE_pose_stein.pose.position.y = EE_pose_stein.pose.position.y - 0.01

        if key == 'a':
            print(key)
            EE_pose_baer.pose.position.y = EE_pose_baer.pose.position.y - 0.01
            EE_pose_stein.pose.position.y = EE_pose_stein.pose.position.y + 0.01

	    



    print("DONZO")
	    #rospy.spin()