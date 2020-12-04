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

# Min allowed gain to move along path (in feedback)
min_allowed_gain = 3

# Max linear velocity (m/s)
max_linear_velocity = 0.5
# Max angular velocity (rad/s)
max_angular_velocity = 1.0


def goal_active():
    rospy.loginfo("I got activated")


def goal_feedback(feedback):
    rospy.loginfo("I got feedback")

    # Check if this path has higher gain than min_allowed_gain
    if feedback.gain>min_allowed_gain:
        goal_client.cancel_goal()
        move(feedback.path)

    # If it has cancel goal and move along the path


def goal_result(state, result):
    rospy.loginfo("I got a result")
    if actionlib.TerminalState.SUCCEEDED == state:
        rospy.loginfo("Action returned succeeded")

        # TODO: Do your stuff
        if result.path.poses:
            move(result.path)


    elif actionlib.TerminalState.RECALLED == state:
        rospy.loginfo("Action returned recalled")
    elif actionlib.TerminalState.REJECTED == state:
        rospy.loginfo("Action returned rejected")
    elif actionlib.TerminalState.PREEMPTED == state:
        rospy.loginfo("Action returned preempted")
    elif actionlib.TerminalState.ABORTED == state:
        rospy.loginfo("Action returned aborted")
    elif actionlib.TerminalState.LOST == state:
        rospy.loginfo("Action returned lost")




def move(path):
    global control_client, robot_frame_id, pub

    twmsg = geometry_msgs.msg.Twist() # Creation of the object type Twist

    """twmsg.angular.z = 4 * math.atan2(transformed_setpoint.point.y, transformed_setpoint.point.x)
    twmsg.linear.x = 0.5 * math.sqrt(transformgoal_result_setpoint.point.x ** 2 + transformed_setpoint.point.y ** 2)
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

        if answer.setpoint.header.frame_id!=robot_frame_id:
            transf=tf_buffer.lookup_transform(robot_frame_id,answer.setpoint.header.frame_id, rospy.Time())
            transformed_setpoint = tf2_geometry_msgs.do_transform_point(answer.setpoint, transf)

        else:
            transformed_setpoint=answer.setpoint

        twmsg.angular.z = math.atan2(transformed_setpoint.point.y, transformed_setpoint.point.x)
        twmsg.linear.x = math.sqrt(transformed_setpoint.point.x ** 2 + transformed_setpoint.point.y ** 2)
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
    goal = irob_assignment_1.msg.GetNextGoalGoal()
    goal_client.send_goal(goal, active_cb=goal_active,feedback_cb=goal_feedback, done_cb=goal_result) #Todos los movimientos se hacen desde el feedback callback o desde el done callback



if __name__ == "__main__":
    # Init node
    rospy.init_node('controller feedback')
    tf_buffer=tf2_ros.Buffer()
    listener=tf2_ros.TransformListener(tf_buffer)

    # Init publisher
    pub=rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Init simple action server
    goal_client = actionlib.SimpleActionClient('get_next_goal', irob_assignment_1.msg.GetNextGoalAction)
    goal_client.wait_for_server()

    # Init service client
    rospy.wait_for_service('get_setpoint')
    control_client=rospy.ServiceProxy('get_setpoint', GetSetpoint)

    # Call get path
    get_path()

    # Spin
    rospy.spin()