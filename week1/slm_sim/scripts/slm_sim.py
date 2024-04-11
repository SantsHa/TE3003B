#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

# Declare Variables to be used
k = 0.01
m = 0.75
l = 0.36 
g = 9.8
Tau = 0.0
x1 = 0.0
x2 = 0.0

# Setup Variables to be used

# Declare the input Message

# Declare the  process output message


# Define the callback functions

# wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi

# Callback function for the Tau message
def tau_callback(msg):
    global Tau
    Tau = msg.data

if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("SLM_Sim")

    # Get Parameters   

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate", 100))

    # Setup the Subscribers
    rospy.Subscriber("/tau", Float32, tau_callback)

    # Setup the publishers
    joint_state_publisher = rospy.Publisher("/joint_states", JointState, queue_size=10)

    print("The SLM sim is Running")
    try:
        while not rospy.is_shutdown():
            # SLM governing equation
            x1 += x2 * 0.01  # Update x1 based on x2
            J = m * l**2  # Moment of inertia
            x2_dot = (1 / (J + m * l**2)) * (-m * g * l * np.cos(x1) - k * x2 + Tau)
            x2 += x2_dot * 0.01  # Update x2 based on x2_dot

            # Publish joint state
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = ['joint2']
            joint_state_msg.position = [x1]
            joint_state_msg.velocity = [x2]
            joint_state_publisher.publish(joint_state_msg)

            # Print some values for debugging
            rospy.loginfo('x1: {}, x2: {}'.format(x1, x2))

            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass  # Initialise and Setup node


''' import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import math  # Import the math module for cosine function

# SLM Parameters
k = 0.01
m = 0.75
l = 0.36 
g = 9.8
Tau = 0.0
x1 = 0.0
x2 = 0.0
J = 1/3 * m * l**2  # Inertia of a rod about one end

def tau_callback(data):
    global Tau
    Tau = 1#data.data

def slm_simulation():
    global x1, x2
    rospy.init_node('slm_sim', anonymous=True)
    rate = rospy.Rate(100)  # 100Hz

    joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    tau_sub = rospy.Subscriber('/tau', Float64, tau_callback)
    print("8")


    while not rospy.is_shutdown():
        x1 += x2 * 0.01  # dt = 0.01s
        x2_dot = (1 / (J + m * l**2)) * (-m * g * l * math.cos(x1) - k * x2 + Tau)  # Use math.cos
        x2 += x2_dot * 0.01

        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['joint2']
        joint_state.position = [x1]
        joint_state.velocity = [x2]
        joint_pub.publish(joint_state)

        rospy.loginfo_throttle(1, "SLM Joint State - Position: %f, Velocity: %f" % (x1, x2))


        rate.sleep()
    rospy.spin()

if __name__ == '_main_':
    try:
        slm_simulation()
    except rospy.ROSInterruptException:
        pass

'''