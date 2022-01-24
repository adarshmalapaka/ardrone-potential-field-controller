#!/usr/bin/env python
from numpy.core.fromnumeric import size
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion
import numpy as np
import matplotlib.pyplot as plt

p_d = np.array([0.0, 0.0])
v_d = np.array([0.0, 0.0])

#***** Setting the target and obstacle X-Y coordinates *******
# Uncomment for the corresponding X-Y plot of the Figures in the paper
# For Figures 8 & 9
p_t = np.matrix((np.array((2.5,-1.0)), np.array((2.5,1.0)), np.array((-2.5,1.0)), np.array((-2.5,-1.0)), np.array((2.5,-1.0)))).T
p_o = np.matrix((np.array([1.0, 1.0]))).T

# For Figure 10
# p_t = np.matrix((np.array([6.0,0.0]), np.array([0.0,0.0]))).T
# p_o = np.matrix((np.array([1.2,0.0]), np.array([2.5,0.6]), np.array([3.0,0.0]), np.array([4.2,0.5]), np.array([4.7,-0.5]))).T

# For Figure 14
# p_t = np.matrix((np.array((4.2,0.0)))).T
# p_o = np.matrix(np.empty).T

# For Figure 15
# p_t = np.matrix((np.array([5.2,0.0]))).T
# p_o = np.matrix((np.array([2.5,0.0]))).T

# For Figure 17 & 19
# p_t = np.matrix((np.array([-1.5,-0.5]), np.array([1.5,-0.5]), np.array([1.5,0.5]), np.array([-1.5,0.5]), np.array([-1.5,-0.5]))).T
# p_o = np.matrix(np.array([0.0,0.4])).T


num_tgt = np.shape(p_t)[1] # number of targets
num_obs = np.shape(p_o)[1] # number of obstacles
if (np.shape(p_o)[0] is 1) and (np.shape(p_o)[1] is 1):
    num_obs = 0


# Callback Function to receive the drone's pose data
def pose_callback(data):
    global p_d
    global roll, pitch, yaw

    # Array to save the received drone X and Y positions
    p_d = np.array([data.position.x, data.position.y]) 

    # Array to save the received drone quaternion orientations
    drone_angles = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]

    # converting quaternion to Euler angles 
    (roll, pitch, yaw) = euler_from_quaternion (drone_angles) 


# Callback Function to receive the drone's velocity data 
def vel_callback(data):
    global v_d
    # Array to save the received drone X and Y linear velocities
    v_d = np.array([data.linear.x, data.linear.y]) 


# Function to compute the extended potential field velocities 
def ex_PFC():

    global v_d, p_d

    # Positive scalars of exPFC controller 
    lambda_1 = 0.6
    lambda_2 = 0.1
    eta_1 = 0.02
    
    control_vel = Twist()

    i_tgt = 0 # target index
    p_avoid = 0.8 # obstacle avoiding range
    
    rate = rospy.Rate(50) # 50 Hz

    while ((not rospy.is_shutdown())):
        rospy.loginfo("Heading to Waypiont No. {"+str(i_tgt+1)+"}")
        if lambda_2 is not 0.0:
            plt.title('Extended PFC - lambda1: ' + str(lambda_1)+', lambda2: ' + str(lambda_2) + ', eta1:  ' + str(eta_1))
        else:
            plt.title('Traditional PFC - lambda1: ' + str(lambda_1)+', lambda2: ' + str(lambda_2) + ', eta1:  ' + str(eta_1))

        # Comment for Fig 14, 15 or 19 in paper 
        plt.xlabel(' X Position [m]')
        plt.ylabel(' Y Position [m]')

        # Uncomment for Fig 19 in paper
        # plt.xlabel(' Time [s]')
        # plt.ylabel(' Error [m]')

        # Uncomment for Fig 14, 15
        # plt.ylabel(' Relative Distance to Target [m]')

        # Uncomment for Fig 15
        # plt.ylabel(' Relative Distance to Obstacle [m]')

        # drone is considered to reach target if it is <= 0.1m of the target,
        while(np.linalg.norm(p_d - (p_t[:,i_tgt]).T) > 0.1): 

            v_pfc_att = -lambda_1*(p_d - p_t[:,i_tgt].T) # attractive potential velocity
            v_pfc_rep = np.array([0,0]) # repulsive potential velocity

            for i in range(0,num_obs):
                if (np.linalg.norm(p_d - (p_o[:,i]).T) <= p_avoid):
                    v_pfc_rep = (eta_1/(np.linalg.norm((p_d - (p_o[:,i]).T)))**4) * (p_d - (p_o[:,i]).T)
                else:
                    v_pfc_rep = v_pfc_rep + np.array([0,0])

            v_pfc = v_pfc_att + v_pfc_rep
            v_epfc_iner = v_pfc - lambda_2*(v_d)

            # rotation matrix to transform from inertial frame to body frame
            R = np.matrix([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]]) 

            v_epfc_body = (np.matmul(R, v_epfc_iner.T)).T # array
            v_epfc_body = np.clip(v_epfc_body, -1, 1) # saturating the calculated velocity to [-1,1] 

            # publishing the calculated velocity to the drone simulation
            control_vel.linear.x = v_epfc_body[0,0]
            control_vel.linear.y = v_epfc_body[0,1]
            control_vel.linear.z = 0
            control_vel.angular.x = 0
            control_vel.angular.y = 0
            control_vel.angular.z = 0
            pfc_pub.publish(control_vel)

            # Comment for Fig 14, 15, or 19 in paper
            plt.scatter(p_d[0], p_d[1])

            # Uncomment for Fig 19 in paper
            # plt.scatter(rospy.get_time(), p_d[0] - (p_t[0,i_tgt]).T)

            # Uncomment for Fig 14, 15 (ploting target distance) in paper
            # plt.scatter(rospy.get_time(), (p_t[0,i_tgt]).T - p_d[0])

            # Uncomment for Fig 15 (ploting obstacle distance) in paper
            # plt.scatter(rospy.get_time(), np.linalg.norm((p_o[:,0]).T - p_d))

            rate.sleep()
            # rospy.spin()

        control_vel.linear.x = 0.0
        control_vel.linear.y = 0.0
        control_vel.linear.z = 0.0
        control_vel.angular.x = 0.0
        control_vel.angular.y = 0.0
        control_vel.angular.z = 0.0
        pfc_pub.publish(control_vel)
        
        # Uncomment for Fig 19 in paper 
        # plt.scatter(rospy.get_time(), p_d[0] - (p_t[0,i_tgt]).T)

        # Uncomment for Fig 14, 15 (ploting target distance) in paper 
        # plt.scatter(rospy.get_time(), (p_t[0,i_tgt]).T - p_d[0])

        # Uncomment for Fig 15 (ploting obstacle distance) in paper
        # plt.scatter(rospy.get_time(), (p_o[0,0]).T - p_d[0])

        i_tgt = i_tgt + 1

        # Comment for Fig 14, 15 or 19 in paper
        plt.scatter(p_d[0], p_d[1])
        

        if(i_tgt >= num_tgt):
            # Comment the below 2 lines for Fig 14, 15 or 19 in paper
            for i in range(0,num_obs):
                plt.scatter(p_o[0,i], p_o[1,i], marker='x', linewidths=5)
            plt.show()
            break
        
if __name__ == '__main__':
    try:
        rospy.init_node('ePFC', anonymous=False)
        
        # Publisher to initially make the drone to takeoff 
        takeoff_pub = rospy.Publisher('/drone/takeoff', Empty, queue_size=10)
        
        # Loop until the takeoff topic has a connection
        while(takeoff_pub.get_num_connections() <= 0):
            pass

        takeoff_pub.publish(Empty())

        pose_sub = rospy.Subscriber("/drone/gt_pose", Pose, pose_callback)
        vel_sub = rospy.Subscriber("/drone/gt_vel", Twist, vel_callback)
        pfc_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Loop until there are publishers and subscribers to the above 3 topics 
        while(pose_sub.get_num_connections() <= 0 or vel_sub.get_num_connections() <= 0 or pfc_pub.get_num_connections() <= 0):
            pass

        ex_PFC()

    except rospy.ROSInterruptException:
        pass