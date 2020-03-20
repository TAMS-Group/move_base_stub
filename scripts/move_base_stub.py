#! /usr/bin/env python
import rospy
import actionlib
import move_base_msgs.msg
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped, Quaternion
from std_msgs.msg import Header

import tf2_ros
from tf.transformations import euler_from_quaternion

class MoveAction(object):
	def __init__(self):
		self.x = 0.0
		self.y = 0.0
		self.q = Quaternion(0,0,0,1)

		self._as = actionlib.SimpleActionServer("move_base", move_base_msgs.msg.MoveBaseAction, execute_cb=self.execute_cb, auto_start = False)

		self._move_base_wrapper_ac = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

		self.pose_pub = rospy.Publisher("amcl_pose", PoseWithCovarianceStamped, queue_size= 1)
		self.tf_pub = tf2_ros.TransformBroadcaster()

		self._result = move_base_msgs.msg.MoveBaseResult()

		self._as.start()
		# simple message interface for RViz
		rospy.Subscriber("move_base_simple/goal", PoseStamped, self.move_base_simple_cb)

		rospy.loginfo("move_base_stub startup successful")

	def spin_publisher(self):
		r= rospy.Rate(10)
		while not rospy.is_shutdown():
			h = Header(frame_id= 'map', stamp= rospy.Time.now())

			p = PoseWithCovarianceStamped()
			p.header = h
			p.pose.pose.position.x = self.x
			p.pose.pose.position.y = self.y
			p.pose.pose.orientation = self.q
			self.pose_pub.publish(p)

			t= TransformStamped()
			t.header= h
			t.child_frame_id= 'base_footprint'
			t.transform.translation.x= self.x
			t.transform.translation.y= self.y
			t.transform.translation.z= 0.0
			t.transform.rotation= self.q
			self.tf_pub.sendTransform(t)
			r.sleep()

	def move_base_simple_cb(self, data):
		print("resending move_base_simple/goal msg as an action")
		move_base_action_goal = move_base_msgs.msg.MoveBaseGoal(data)
		self._move_base_wrapper_ac.send_goal(move_base_action_goal)

	def execute_cb(self, goal):
		# Check if trixi is inside the bounds
		if goal.target_pose.header.frame_id != "map":
			rospy.logerr('Only goals in map frame are supported by stub')
			self._as.set_aborted(self._result)
			return

		self.x= goal.target_pose.pose.position.x
		self.y= goal.target_pose.pose.position.y
		self.q= goal.target_pose.pose.orientation
		_,_,theta=  euler_from_quaternion(
			[goal.target_pose.pose.orientation.x,
			 goal.target_pose.pose.orientation.y,
			 goal.target_pose.pose.orientation.z,
			 goal.target_pose.pose.orientation.w])

		rospy.loginfo("robot moved to position x: " + str(self.x) + " / y: " + str(self.y) + " / theta: " + str(theta))

		self._as.set_succeeded(self._result)

if __name__ == '__main__':
	rospy.init_node("move_base")
	server = MoveAction()
	server.spin_publisher()
