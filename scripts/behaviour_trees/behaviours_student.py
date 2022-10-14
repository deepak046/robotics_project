# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviours needed for the example student solution.

from mailbox import NotEmptyError
import numpy as np
from numpy import linalg as LA

import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

# variable to call reset behaviour which iterates through the elements in reset_beh_dict
# and resets each of the behaviours individually
call_reset = False
reset_success = False
reset_var = False
reset_beh_dict = {"movehead_up_1":"movehead_up_1", 
                  "tuckarm":"tuckarm", 
                  "spawn":"spawn", 
                  "navigate_pick":"navigate_pick", 
                  "movehead_down_1":"movehead_down_1", 
                  "pick":"pick", 
                  "movehead_up_2":"movehead_up_2", 
                  "navigate_place":"navigate_place", 
                  "place":"place", 
                  "movehead_down_2":"movehead_down_2", 
                  "detect_cube":"detect_cube"}
reset_beh = None

class counter(pt.behaviour.Behaviour):

    """
    Returns running for n ticks and success thereafter.
    """

    def __init__(self, n, name):

        rospy.loginfo("Initialising counter behaviour.")

        # counter
        self.i = 0
        self.n = n

        # become a behaviour
        super(counter, self).__init__(name)

    def update(self):

        # increment i
        self.i += 1

        # succeed after count is done
        return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS


class go(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, name, linear, angular):

        rospy.loginfo("Initialising go behaviour.")

        # action space
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        #self.cmd_vel_top = "/key_vel"
        #rospy.loginfo(self.cmd_vel_top)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = linear
        self.move_msg.angular.z = angular

        # become a behaviour
        super(go, self).__init__(name)

    def update(self):

        # send the message
        rate = rospy.Rate(10)
        self.cmd_vel_pub.publish(self.move_msg)
        rate.sleep()

        # tell the tree that you're running
        return pt.common.Status.RUNNING

class tuckarm(pt.behaviour.Behaviour):

    """
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising tuck arm behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")

    def update(self):

        global reset_beh, reset_var, reset_beh_dict
        if reset_var:
            if reset_beh == "tuckarm":
                # if cube placement failed, reset behaviour and execute it again
                self.sent_goal = False
                self.finished = False
                reset_beh = None

        # already tucked the arm
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to tuck arm if haven't already
        elif not self.sent_goal:
            # send the goal
            rospy.loginfo("Tucking arm.")
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # then I'm finished!
            self.finished = True
            if reset_var:
                # if behaviour was executed successfully in the reset chain,
                # call reset for next behaviour
                reset_beh = reset_beh_dict["spawn"]
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.play_motion_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING

class movehead(pt.behaviour.Behaviour):

    """
    Lowers or raises the head of the robot.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, direction):

        rospy.loginfo("Initialising move head behaviour.")

        # Variables for the operation for resetting in the correct order
        self.reset_oper = None
        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # head movement direction; "down" or "up"
        self.direction = direction

        self.tried = False
        self.done = False
        
        # become a behaviour
        super(movehead, self).__init__("Lower head!")
   
    def update(self):

        global reset_beh, reset_var, reset_beh_dict
        if reset_var:
            # if cube placement failed, reset behaviour and execute it again
            # since we need movehead up/down for both picking and placing,
            # reset them individually in the right chronological order
            if reset_beh == "movehead_down_1":
                self.reset_oper = 0
                self.direction = "down"
                self.tried = False
                self.done = False
                reset_beh = None
            elif reset_beh == "movehead_up_1":
                self.reset_oper = 1
                self.direction = "up"
                self.tried = False
                self.done = False
                reset_beh = None
            elif reset_beh == "movehead_down_2":
                self.reset_oper = 2
                self.direction = "down"
                self.tried = False
                self.done = False
                reset_beh = None
            elif reset_beh == "movehead_up_2":
                self.reset_oper = 3
                self.direction = "up"
                self.tried = False
                self.done = False
                reset_beh = None

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.move_head_req = self.move_head_srv(self.direction)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.move_head_req.success:
            self.done = True
            if reset_var:
                # if behaviour was executed successfully in the reset chain,
                # call reset for next behaviour
                if self.reset_oper == 0:
                    reset_beh = reset_beh_dict["pick"] 
                elif self.reset_oper == 1:
                    reset_beh = reset_beh_dict["tuckarm"]
                elif self.reset_oper == 2:
                    reset_beh = reset_beh_dict["detect_cube"]
                elif self.reset_oper == 3:
                    reset_beh = reset_beh_dict["navigate_place"]

            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_head_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

class detect_cube(pt.behaviour.Behaviour):
    """
    Detects the cube.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """
    def __init__(self):
        rospy.loginfo("Checking for cube...")

        # Aruco pose topic
        self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')

        self.no_cube = None
        # become a behaviour
        super(detect_cube, self).__init__()

    # def initialise(self):
    def update(self):

        global reset_beh, reset_var, reset_beh_dict, reset_success, call_reset
        if reset_var and reset_beh == "detect_cube":
            # if cube placement failed, reset behaviour and execute it again
            # since this is the last behaviour in the reset chain, end the chain by 
            # setting reset_var to false again
            self.no_cube = None
            reset_var = False

        # if cube is detected, return success
        if self.no_cube == True:
            return pt.common.Status.FAILURE
        try:
            rospy.wait_for_message(self.aruco_pose_top, PoseStamped, timeout=10)
            rospy.loginfo("Detected the cube")
            self.no_cube = False
            reset_success = True
            return pt.common.Status.SUCCESS
        except:
            # if this fails, we know, that no cube was placed, 
            # so we want to reset all relevant behaviours again
            call_reset = True
            self.no_cube = True
            rospy.loginfo("Couldn't detect the cube")
            return pt.common.Status.FAILURE

class pick_and_place(pt.behaviour.Behaviour):
    """
    Picks and places the cube.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """
    def __init__(self, operation_str):
        # Picking service
        self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        # Placing service
        self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')

        # Operation being done                
        self.operation = operation_str


        # Wait for service providers
        rospy.wait_for_service(self.pick_srv_nm, timeout=30)
        rospy.wait_for_service(self.place_srv_nm, timeout=30)

        if self.operation == "pick":
            rospy.loginfo("Picking the cube...")
            self.pick_srv = rospy.ServiceProxy(self.pick_srv_nm, SetBool)
        
        elif self.operation == "place":
            rospy.loginfo("Placing the cube...")
            self.place_srv = rospy.ServiceProxy(self.place_srv_nm, SetBool)     

        else:
            rospy.logerr("Invalid operation")       

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(pick_and_place, self).__init__("Pick_or_place")


    def update(self):

        global reset_beh, reset_var, reset_beh_dict, call_reset
        if reset_var:
            # if cube placement failed, reset behaviour and execute it again
            if reset_beh == "pick":
                self.tried = False
                self.done = False
                self.operation == "pick"
                reset_beh = None      
            elif reset_beh == "place":
                self.tried = False
                self.done = False
                self.operation == "place"
                reset_beh = None                       

        if self.operation == "pick":
            # success if done
            if self.done:
                return pt.common.Status.SUCCESS

            # try if not tried
            elif not self.tried:
                
                # command
                self.pick_srv_req = self.pick_srv()
                self.tried = True

                # tell the tree you're running
                return pt.common.Status.RUNNING

            # if succesful
            elif self.pick_srv_req.success:
                self.done = True
                if reset_var:
                    # if behaviour was executed successfully in the reset chain,
                    # call reset for next behaviour
                    reset_beh = reset_beh_dict["movehead_up_2"]
                rospy.loginfo("Pick service successful")
                return pt.common.Status.SUCCESS

            # if failed
            elif not self.pick_srv_req.success:
                return pt.common.Status.FAILURE

            # if still trying
            else:
                return pt.common.Status.RUNNING

        elif self.operation == "place":
                        # success if done
            if self.done:
                return pt.common.Status.SUCCESS

            # try if not tried
            elif not self.tried:
                
                # command
                self.place_srv_req = self.place_srv()
                self.tried = True

                # tell the tree you're running
                return pt.common.Status.RUNNING

            # if succesful
            elif self.place_srv_req.success:
                self.done = True
                if reset_var:
                    # if behaviour was executed successfully in the reset chain,
                    # call reset for next behaviour
                    reset_beh = reset_beh_dict["movehead_down_2"]
                rospy.loginfo("Place service successful")
                return pt.common.Status.SUCCESS

            # if failed
            elif not self.place_srv_req.success:
                # if placing failed, we want to reset all relevant behaviours to try again
                call_reset = True
                return pt.common.Status.FAILURE

            # if still trying
            else:
                return pt.common.Status.RUNNING


class localize(pt.behaviour.Behaviour):

    """
    Globally localizes the robot in the environment.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising move head behaviour.")

        # Global localization service
        self.global_loc_srv_nm = rospy.get_param(rospy.get_name() + '/global_loc_srv')
        self.global_loc_srv = rospy.ServiceProxy(self.global_loc_srv_nm, Empty)

        # Clear costmap service
        self.clear_cm_srv_nm = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')
        self.clear_cm_srv = rospy.ServiceProxy(self.clear_cm_srv_nm, Empty)

        # Move topic 
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        self.move_msg = Twist()
        self.move_msg.linear.x = 0
        self.move_msg.angular.z = 0.45

        # Subscribe to amcl topic to get the latest amcl estimate
        self.amcl_pose_top = rospy.get_param(rospy.get_name() + '/amcl_estimate')
        self.amcl_pose_sub = rospy.Subscriber(self.amcl_pose_top, PoseWithCovarianceStamped, self.amcl_pose_cb)

        # class variables to check if robot is localized and whethter to spread out particles again
        self.localized = False
        self.call_service = True
        # Counter to reset localization service if the robot doesn't localize before a given amount of time
        self.counter = 0

        # Threshold for convergence and covariance of amcl pose
        self.threshold = 0.05
        self.covar = None

        rospy.wait_for_service(self.global_loc_srv_nm, timeout=10)
        
        # become a behaviour
        super(localize, self).__init__("Localize!")
    
    def amcl_pose_cb(self, pose_msg):
        # covariance matrix of amcl estimate
        self.covar = np.reshape(pose_msg.pose.covariance, (6,6))

    def initialise(self):
        # spread out particles if not localized and call_service is true
        if self.call_service and not(self.localized):
            # Spread out particles to start localization
            self.global_loc_srv()
            # Clear costmap to make sure that navigatino finds a path
            self.clear_cm_srv()
            self.call_service = False
        
    def update(self):

        try:
            rate = rospy.Rate(10)
            # spin around
            if self.localized == False:
                self.cmd_vel_pub.publish(self.move_msg)
                self.counter += 1
            rate.sleep()

            # check for convergence
            if np.trace(self.covar) < self.threshold:
                self.localized = True
                self.call_service = True
                self.counter = 0
                return pt.common.Status.SUCCESS
                
            else:
                self.localized = False
                # if robot has been spinning too long, set call_service True
                # so particles are spread out again
                if self.counter > 170:
                    self.counter = 0
                    self.call_service = True
                if self.call_service:
                    return pt.common.Status.FAILURE
                # if not converged, running
                self.clear_cm_srv()
                return pt.common.Status.RUNNING
        except:
            return pt.common.Status.FAILURE


class navigate(pt.behaviour.Behaviour):

    """
    Navigates the robot to a given pose in the environment.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """
    def __init__(self, name, pose_msg, oper):
        
        # Action client to request move action
        self.move_client = SimpleActionClient('/move_base', MoveBaseAction)

        self.move_client.wait_for_server()

        # Operation
        self.oper = oper
        # stati of navigation goal
        self.PENDING=0
        self.ACTIVE=1
        self.PREEMPTED=2
        self.SUCCEEDED=3
        self.ABORTED=4
        self.REJECTED=5
        self.PREEMPTING=6
        self.RECALLING=7
        self.RECALLED=8
        self.LOST=9

        # some class variables
        self.goal_pose_msg = pose_msg
        self.navigation_result_status = None
        self.finished = False
        self.sent_goal = False

        super(navigate, self).__init__(name)

    def nav_result_cb(self, state, result):
        self.navigation_result_status = state

    def update(self):

        global reset_beh, reset_var, reset_beh_dict
        if reset_var:
            # if cube placement failed, reset behaviour and execute it again
            if self.oper == "pick" and reset_beh == "navigate_pick":
                self.navigation_result_status = None
                self.finished = False
                self.sent_goal = False
                reset_beh = None
            elif self.oper == "place" and reset_beh == "navigate_place":
                self.navigation_result_status = None
                self.finished = False
                self.sent_goal = False
                reset_beh = None

        # if already at goal
        if self.finished:
            return pt.common.Status.SUCCESS

        # if goal not sent yet, send it, dude
        elif not self.sent_goal:
            try: 
                goal = MoveBaseGoal(self.goal_pose_msg)

                self.move_client.send_goal(goal,
                    done_cb=self.nav_result_cb)

                self.sent_goal = True

                # let tree know we're running like Forrest Gump
                return pt.common.Status.RUNNING
                
            except rospy.ROSInterruptException:
                rospy.loginfo("Failed to send goal to move_base action.")
                return pt.common.Status.FAILURE
        
        # if action server returns SUCCEEDED, finish
        elif self.navigation_result_status == self.SUCCEEDED:
            self.finished = True
            # if behaviour was executed successfully in the reset chain,
            # call reset for next behaviour
            if reset_var and self.oper == "pick":
                reset_beh = reset_beh_dict["movehead_down_1"]
            elif reset_var and self.oper == "place":
                reset_beh = reset_beh_dict["place"]

            return pt.common.Status.SUCCESS

        # let's hope, this doesn't happen
        elif self.navigation_result_status == (self.PREEMPTED or self.ABORTED or self.LOST):
            return pt.common.Status.FAILURE

        # keep running otherwise
        else:
            return pt.common.Status.RUNNING

           
class respawn_cube(pt.behaviour.Behaviour):

    """
    Respawns the cube on table 1 if placement failed
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """
    
    def __init__(self):
        self.spawn_srv_nm = '/gazebo/set_model_state'
        rospy.wait_for_service(self.spawn_srv_nm, timeout=30)

        # initiate spawn service
        self.spawn_srv = rospy.ServiceProxy(self.spawn_srv_nm, SetModelState)

        # create msg to send to spawn service
        self.cube_model = ModelState()
        self.cube_model.model_name = "aruco_cube"
        self.cube_model.pose.position.x = -1.130530 
        self.cube_model.pose.position.y = -6.653650
        self.cube_model.pose.position.z =  0.862500    
        self.cube_model.reference_frame = "map"

        # execution checkers
        self.tried = False
        self.done = False
        
        # become a bereset_varhaviour
        super(respawn_cube, self).__init__("Respawn cube")
   
    def update(self):

        global reset_beh, reset_var, reset_beh_dict
        # if cube placement failed, reset behaviour and execute it again
        if reset_var and reset_beh == "spawn":
            self.tried = False
            self.done = False
            reset_beh = None 

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.spawn_srv_req = self.spawn_srv(self.cube_model)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.spawn_srv_req.success:
            self.done = True
            if reset_var:
                # if behaviour was executed successfully in the reset chain,
                # call reset for next behaviour
                reset_beh = reset_beh_dict["navigate_pick"]
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.spawn_srv_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

class reset(pt.behaviour.Behaviour):
       
    """
    Resets all relevant behaviours after failed placement
    """

    def __init__(self):
        
        # Trying to rotate the robot after it fails the tree and calls reset
        # Command velocity topic 
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        # Command velocity publisher
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.angular.z = -0.5
        super(reset, self).__init__()

    def update(self):

        global reset_beh, reset_var, reset_beh_dict, reset_success, call_reset
        # call_reset makes sure, we only reset once
        if call_reset:
            for _ in range(60):
                rate = rospy.Rate(10)
                self.cmd_vel_pub.publish(self.move_msg)
                rate.sleep()
            reset_var = True
            reset_beh = reset_beh_dict["movehead_up_1"]
            call_reset = False
            return pt.common.Status.FAILURE
        # if everything succeeded, return success
        if reset_success and not(call_reset):
            return pt.common.Status.SUCCESS
        # otherwise failure
        else:
            return pt.common.Status.FAILURE