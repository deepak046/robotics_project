#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		self.pick_pose_top = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
		self.place_pose_top = rospy.get_param(rospy.get_name() + '/place_pose_topic')

		self.pick_pose_msg = rospy.wait_for_message(self.pick_pose_top, PoseStamped, timeout=5)
		self.place_pose_msg = rospy.wait_for_message(self.place_pose_top, PoseStamped, timeout=5)

		rospy.loginfo("Initialising behaviour tree")

		# tuck arm
		b_1 = tuckarm()

		# Localize
		b_2 = localize()

		# Navigate to pick pose
		b_3 = navigate("Move to pick pose", self.pick_pose_msg, "pick")

		# move head down to detect cube
		b_4 = movehead("down")

		# # check for cube
		# b2 = detect_cube()
		
		# Picking the cube
		b_5 = pick_and_place("pick")

		b_6 = movehead("up")

		b_6_5 = pt.composites.Selector(name="turn 90deg after pick", children=[counter(15, "count turn"), go("turn 90 deg", 0, -0.5)])

		b_7 = navigate("Move to place pose", self.place_pose_msg, "place")

		# place the cube on the table, if no success, move back
		b_8 = RSequence(
			name="Place and detect cube",
			children=[pick_and_place("place"), movehead("down"), detect_cube()] 
		)

		b_9 = RSequence(
			name="Place and detect cube",
			children=[respawn_cube(), reset()]
		)
		# check if cube was placed, if try again
		b_10 = pt.composites.Selector(
			name="Check if placing is successful fallback",
			children=[b_8, b_9]
		)

		# become the tree
		tree = RSequence(name="Main sequence", children=[movehead("up"), b_1, b_2, b_3, b_4, b_5, b_6, b_6_5, b_7, b_10])#, b_4, b_5, b_6, b_6_5, b_7, b_10
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
