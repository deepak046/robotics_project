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

		# Localize
		b_1 = localize()

		# Navigate to pick pose
		b_2 = navigate("Move to pick pose", self.pick_pose_msg)
		# tuck arm
		b0 = tuckarm()

		# move head down to detect cube
		b1 = movehead("down")

		# check for cube
		b2 = detect_cube()
		
		# pick the cube
		b3 = pick_and_place("pick")

		b4 = movehead("up")

		# turn 
		turn_1 = pt.composites.Selector(
			name="Turn around fallback",
			children=[counter(60, "Turned around?"), go("Turn around!", 0, -0.5)]
		)

		# move straight
		move_straight_1 = pt.composites.Selector(
			name="Move to table fallback",
			children=[counter(18, "At table?"), go("Go to table!", 0.5, 0)]
		)

		# turn 
		turn_2 = pt.composites.Selector(
			name="Turn around fallback",
			children=[counter(60, "Turned around?"), go("Turn around!", 0, -0.5)]
		)

		# move straight
		move_straight_2 = pt.composites.Selector(
			name="Move to table fallback",
			children=[counter(18, "At table?"), go("Go to table!", 0.5, 0)]
		)

		# move sequence to turn 180 deg and move straight to table
		move_table_1 = RSequence(
					   name="turn and move sequence",
					   children=[turn_1 ,move_straight_1]
		)

		move_table_2 = RSequence(
					   name="turn and move sequence",
					   children=[turn_2 ,move_straight_2]
		)

		# place the cube on the table, if no success, move back
		b5 = RSequence(
			name="Place and detect cube",
			children=[pick_and_place("place"), movehead("down"), detect_cube()] 
		)

		# check if cube was placed, if not move back
		b6 = pt.composites.Selector(
			name="Check if placing is successful fallback",
			children=[b5, move_table_1]
		)

		# become the tree
		tree = RSequence(name="Main sequence", children=[tuckarm(), b_1, b_2, tuckarm(), tuckarm(), tuckarm()])#children=[b0, b1, b3, b4, move_table_2, b6])
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
