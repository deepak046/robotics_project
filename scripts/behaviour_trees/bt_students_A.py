#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")

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
		turn = pt.composites.Selector(
			name="Turn around fallback",
			children=[counter(62, "Turned around?"), go("Turn around!", 0, -0.5)]
		)

		# move straight
		move_straight = pt.composites.Selector(
			name="Move to table fallback",
			children=[counter(18, "At table?"), go("Go to table!", 0.5, 0)]
		)

		# move sequence to turn 180 deg and move straight to table
		move_seq = RSequence(
					   name="turn and move sequence",
					   children=[turn,move_straight]
		)

		# place the cube on the table, if no success, move back
		b5 = RSequence(
			name="Place and detect cube",
			children=[pick_and_place("place"), movehead("down"), detect_cube()]
		)

		# check if cube was placed, if not move back
		b6 = pt.composites.Selector(
			name="Check if placing is successful fallback",
			children=[b5, move_seq]
		)

		# become the tree
		tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, b4, move_seq, b6])
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
