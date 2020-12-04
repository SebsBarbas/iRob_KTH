#!/usr/bin/env python

import numpy as np
from numpy import linalg as LA

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState
from robotics_project.msg import PickUpPoseAction, PickUpPoseGoal, PickUpPoseResult, PickUpPoseFeedback

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray

from moveit_msgs.msg import MoveItErrorCodes, PickupGoal
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


class StateMachine(object):
    def __init__(self):
        
        self.node_name = "Student SM"

        # Access rosparams
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.pick_srv = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.place_srv = rospy.get_param(rospy.get_name() + '/place_srv')
        #self.pose_topic = rospy.get_param(rospy.get_name() + '/marker_pose_topic')

        self.cube_pose = rospy.get_param(rospy.get_name() + '/cube_pose')

        # Subscribe to topics
        
        #self.marker_sub = rospy.Subscriber("markers", MarkerArray, self.marker_cb)



        # Wait for service providers
        rospy.wait_for_service(self.mv_head_srv_nm, timeout=30)
        rospy.wait_for_service(self.pick_srv, timeout=30)
        rospy.wait_for_service(self.place_srv, timeout=30)


        rospy.wait_for_message("/joint_states", JointState)

        # Instantiate publishers
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        self.pick_pub = rospy.Publisher("/pose_topic", PoseStamped, queue_size=10)

       
        #Action client for pick up
        #self.pick_as = SimpleActionClient('/place_holder10', PickUpPoseAction)
        #self.pick_as.wait_for_server()


        # Set up action clients
        rospy.loginfo("%s: Waiting for play_motion action server...", self.node_name)
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /play_motion action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to play_motion action server", self.node_name)

        # Init state machine
        self.state = 0
        rospy.sleep(3)
        self.check_states()


    def moveForward(self, limit):

        move_msg = Twist()
        move_msg.linear.x = 0.5

        rate = rospy.Rate(10)
        converged = False
        cnt = 0
        rospy.loginfo("%s: Moving towards ", self.node_name)
        while not rospy.is_shutdown() and cnt < limit:
            self.cmd_vel_pub.publish(move_msg)
            rate.sleep()
            cnt = cnt + 1

        return

    def rotate(self, limit):

        move_msg = Twist()
        move_msg.angular.z = -0.5

        rate = rospy.Rate(10)
        converged = False
        cnt = 0
        rospy.loginfo("%s: Rotating ", self.node_name)
        while not rospy.is_shutdown() and cnt < limit:
            self.cmd_vel_pub.publish(move_msg)
            rate.sleep()
            cnt = cnt + 1

        return
    
    def marker_cb(self, marker_msg):

        marker_pose = PoseStamped()
        marker_pose.pose = marker_msg.markers[0].pose
        marker_pose.header.frame_id = marker_msg.markers[0].header.frame_id
        self.pick_pub.publish(marker_pose)
        self.scene_srv = rospy.ServiceProxy(self.pick_srv, SetBool)
        self.scene_srv(False)


    def check_states(self):

        while not rospy.is_shutdown() and self.state != 4:
            
            # State 0: Pick
            if self.state == 0:
                
                rospy.loginfo("%s: Tucking the arm...", self.node_name)
                goal = PlayMotionGoal()
                goal.motion_name = 'home'
                goal.skip_planning = True
                self.play_motion_ac.send_goal(goal)
                success_tucking = self.play_motion_ac.wait_for_result(rospy.Duration(100.0))

                if success_tucking:
                    rospy.loginfo("%s: Arm tucked.", self.node_name)
                else:
                    self.play_motion_ac.cancel_goal()
                    rospy.logerr("%s: play_motion failed to tuck arm, reset simulation", self.node_name)
                    self.state = 5

                rospy.sleep(1)                

                cube_pick=PoseStamped()
                cube_pick.header.frame_id = "/base_footprint"
                positions = self.cube_pose.split(", ")
                cube_pick.pose.position.x = float(positions[0])
                cube_pick.pose.position.y = float(positions[1]) 
                cube_pick.pose.position.z = float(positions[2])
                cube_pick.pose.orientation.x = float(positions[3])
                cube_pick.pose.orientation.y = float(positions[4])
                cube_pick.pose.orientation.z = float(positions[5])
                cube_pick.pose.orientation.w = float(positions[6])

                
                self.pick_pub.publish(cube_pick)
                #goal=PickUpPoseGoal()
                #goal.object_pose=cube_pick

                rospy.loginfo("Connecting to pick service")
                self.scene_srv = rospy.ServiceProxy(self.pick_srv, SetBool)
                rospy.loginfo("Connected")

                if self.scene_srv(False):
                    print("Succeded!")
                    self.state = 1
                else:
                    self.state = 5
                           

                """
                result = self.pick_pub.publish(cube_pick)
            
                s=self.pick_as.send_goal_and_wait(goal)
                rospy.loginfo("Done")
                result=self.pick_as.get_result()
                if str(moveit_error_dict[result.error_code])!="SUCCESS":
                    rospy.logerr("Failed to pick")
                else:
                    rospy.loginfo("Success")
                    self.state=1
                """
                
                

            # State 1:  Move to table_3_clone
            
            if self.state == 1:


                self.rotate(60)
                rospy.sleep(2)
                self.moveForward(18)
                rospy.sleep(2)
                
                self.state = 2

            
            # State 2:  Place
            if self.state == 2:
                
                rospy.loginfo("Connecting to place service")
                self.scene_srv = rospy.ServiceProxy(self.place_srv, SetBool)
                rospy.loginfo("Connected")

                if self.scene_srv(False):
                    print("Succeded!")
                    self.state = 3
                else:
                    self.state = 5


            # Error handling
            if self.state == 5:
                rospy.logerr("%s: State machine failed. Check your code and try again!", self.node_name)
                return

        rospy.loginfo("%s: State machine finished!", self.node_name)
        return


# import py_trees as pt, py_trees_ros as ptr

# class BehaviourTree(ptr.trees.BehaviourTree):

# 	def __init__(self):

# 		rospy.loginfo("Initialising behaviour tree")

# 		# go to door until at door
# 		b0 = pt.composites.Selector(
# 			name="Go to door fallback", 
# 			children=[Counter(30, "At door?"), Go("Go to door!", 1, 0)]
# 		)

# 		# tuck the arm
# 		b1 = TuckArm()

# 		# go to table
# 		b2 = pt.composites.Selector(
# 			name="Go to table fallback",
# 			children=[Counter(5, "At table?"), Go("Go to table!", 0, -1)]
# 		)

# 		# move to chair
# 		b3 = pt.composites.Selector(
# 			name="Go to chair fallback",
# 			children=[Counter(13, "At chair?"), Go("Go to chair!", 1, 0)]
# 		)

# 		# lower head
# 		b4 = LowerHead()

# 		# become the tree
# 		tree = pt.composites.Sequence(name="Main sequence", children=[b0, b1, b2, b3, b4])
# 		super(BehaviourTree, self).__init__(tree)

# 		# execute the behaviour tree
# 		self.setup(timeout=10000)
# 		while not rospy.is_shutdown(): self.tick_tock(1)


# class Counter(pt.behaviour.Behaviour):

# 	def __init__(self, n, name):

# 		# counter
# 		self.i = 0
# 		self.n = n

# 		# become a behaviour
# 		super(Counter, self).__init__(name)

# 	def update(self):

# 		# count until n
# 		while self.i <= self.n:

# 			# increment count
# 			self.i += 1

# 			# return failure :(
# 			return pt.common.Status.FAILURE

# 		# succeed after counter done :)
# 		return pt.common.Status.SUCCESS


# class Go(pt.behaviour.Behaviour):

# 	def __init__(self, name, linear, angular):

# 		# action space
# 		self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
# 		self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

# 		# command
# 		self.move_msg = Twist()
# 		self.move_msg.linear.x = linear
# 		self.move_msg.angular.z = angular

# 		# become a behaviour
# 		super(Go, self).__init__(name)

# 	def update(self):

# 		# send the message
# 		rate = rospy.Rate(10)
# 		self.cmd_vel_pub.publish(self.move_msg)
# 		rate.sleep()

# 		# tell the tree that you're running
# 		return pt.common.Status.RUNNING


# class TuckArm(pt.behaviour.Behaviour):

# 	def __init__(self):

# 		# Set up action client
# 		self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

# 		# personal goal setting
# 		self.goal = PlayMotionGoal()
# 		self.goal.motion_name = 'home'
# 		self.goal.skip_planning = True

# 		# execution checker
# 		self.sent_goal = False
# 		self.finished = False

# 		# become a behaviour
# 		super(TuckArm, self).__init__("Tuck arm!")

# 	def update(self):

# 		# already tucked the arm
# 		if self.finished: 
# 			return pt.common.Status.SUCCESS
		
# 		# command to tuck arm if haven't already
# 		elif not self.sent_goal:

# 			# send the goal
# 			self.play_motion_ac.send_goal(self.goal)
# 			self.sent_goal = True

# 			# tell the tree you're running
# 			return pt.common.Status.RUNNING

# 		# if I was succesful! :)))))))))
# 		elif self.play_motion_ac.get_result():

# 			# than I'm finished!
# 			self.finished = True
# 			return pt.common.Status.SUCCESS

# 		# if I'm still trying :|
# 		else:
# 			return pt.common.Status.RUNNING
		


# class LowerHead(pt.behaviour.Behaviour):

# 	def __init__(self):

# 		# server
# 		mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
# 		self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
# 		rospy.wait_for_service(mv_head_srv_nm, timeout=30)

# 		# execution checker
# 		self.tried = False
# 		self.tucked = False

# 		# become a behaviour
# 		super(LowerHead, self).__init__("Lower head!")

# 	def update(self):

# 		# try to tuck head if haven't already
# 		if not self.tried:

# 			# command
# 			self.move_head_req = self.move_head_srv("down")
# 			self.tried = True

# 			# tell the tree you're running
# 			return pt.common.Status.RUNNING

# 		# react to outcome
# 		else: return pt.common.Status.SUCCESS if self.move_head_req.success else pt.common.Status.FAILURE


	

if __name__ == "__main__":

	rospy.init_node('main_state_machine')
	try:
		#StateMachine()
		StateMachine()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
