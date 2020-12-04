#!/usr/bin/env python2
import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from math import atan2, hypot
import math
import geometry_msgs.msg

# Use to transform between frames
tf_buffer = None
listener = None

# The exploration simple action client
goal_client = None
# The collision avoidance service client
control_client = None
# The velocity command publisher
pub = None

# The robots frame
robot_frame_id = "base_link"

# Max linear velocity (m/s)
max_linear_velocity = 0.5
# Max angular velocity (rad/s)
max_angular_velocity = 1.0


def move(path):
    global control_client, robot_frame_id, pub
    
    # Call service client with path
    """print("Mas dentro imposible")
    answer=control_client(path)
    new_path = answer.new_path"""

    # Transform Setpoint from service client
    """if answer.setpoint.header.frame_id!=robot_frame_id:
        transf=tf_buffer.lookup_transform(robot_frame_id,answer.setpoint.header.frame_id, rospy.Time())
        transformed_setpoint = tf2_geometry_msgs.do_transform_point(answer.setpoint, transf)
    else:
        transformed_setpoint=answer.setpoint"""

    # Create Twist message from the transformed Setpoint
    twmsg = geometry_msgs.msg.Twist() # Creation of the object type Twist

    """twmsg.angular.z = 4 * math.atan2(transformed_setpoint.point.y, transformed_setpoint.point.x)
    twmsg.linear.x = 0.5 * math.sqrt(transformed_setpoint.point.x ** 2 + transformed_setpoint.point.y ** 2)
    if twmsg.linear.x>max_linear_velocity:
        twmsg.linear.x=max_linear_velocity
    if twmsg.angular.z>max_angular_velocity:
        twmsg.angular.z=max_angular_velocity


    # Publish Twist 
    pub.publish(twmsg)"""
    
    # Call service client again if the returned path is not empty and do stuff again
    """if new_path.poses:
        move(new_path)"""
    new_path=path
    while new_path.poses:
        answer=control_client(new_path)
        new_path=answer.new_path
        print("Dentro 2")

        if answer.setpoint.header.frame_id!=robot_frame_id:
            transf=tf_buffer.lookup_transform(robot_frame_id,answer.setpoint.header.frame_id, rospy.Time())
            transformed_setpoint = tf2_geometry_msgs.do_transform_point(answer.setpoint, transf)

        else:
            transformed_setpoint=answer.setpoint

        twmsg.angular.z = 4 * math.atan2(transformed_setpoint.point.y, transformed_setpoint.point.x)
        twmsg.linear.x = 0.5 * math.sqrt(transformed_setpoint.point.x ** 2 + transformed_setpoint.point.y ** 2)
        if twmsg.linear.x>max_linear_velocity:
            twmsg.linear.x=max_linear_velocity
        if twmsg.angular.z>max_angular_velocity:
            twmsg.angular.z=max_angular_velocity
        
        pub.publish(twmsg)



    # Send 0 control Twist to stop robot
    twmsg.angular.z=0.0
    twmsg.linear.x=0.0
    pub.publish(twmsg)
    # Get new path from action server
    get_path()


def get_path():
    global goal_client

    # Get path from action server
    print("in")
    goal = irob_assignment_1.msg.GetNextGoalAction()
    print("mas in")
    goal_client.send_goal(goal)
    print("Cuasi in")
    goal_client.wait_for_result()
    print("super in")

    


    # Call move with path from action server
    print("1")
    move(goal_client.get_result().path)
    


if __name__ == "__main__":

    # Init node 
    rospy.init_node('controller')
    tf_buffer=tf2_ros.Buffer()
    listener=tf2_ros.TransformListener(tf_buffer)
    
    # Init publisher
    pub=rospy.Publisher('/cmd_vel', Twist, queue_size=10) #Aqui puede fallar algo
    
    # Init simple action server es el explore
    print("Aqui")
    goal_client = actionlib.SimpleActionClient('get_next_goal', irob_assignment_1.msg.GetNextGoalAction)
    goal_client.wait_for_server()
    print("Un poquito mas")
    # Init service client
   
    rospy.wait_for_service('get_setpoint')
    control_client=rospy.ServiceProxy('get_setpoint', GetSetpoint)
    

    # Call get path
    get_path()

    # Spin
    rospy.spin()