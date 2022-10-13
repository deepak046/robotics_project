# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviours needed for the example student solution.


import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest  

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

        # already tucked the arm
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # than I'm finished!
            self.finished = True
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
        self.aruco_pose_rcv = False
        # Aruco pose topic
        self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')

        self.no_cube = None
        # become a behaviour
        super(detect_cube, self).__init__()

    # def initialise(self):
    def update(self):
        # if cube is detected, return success
        if self.no_cube == True:
            return pt.common.Status.FAILURE
        try:
            rospy.wait_for_message(self.aruco_pose_top, PoseStamped, timeout=10)
            rospy.loginfo("Detected the cube")
            self.no_cube == False
            return pt.common.Status.SUCCESS
        except:
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
        self.pick_srv = rospy.get_param(rospy.get_name() + '/pick_srv')
            # Placing service
        self.place_srv = rospy.get_param(rospy.get_name() + '/place_srv')

            # Operation being done                
        self.operation = operation_str


        # Wait for service providers
        rospy.wait_for_service(self.pick_srv, timeout=30)
        rospy.wait_for_service(self.place_srv, timeout=30)

        if self.operation == "pick":
            rospy.loginfo("Picking the cube...")
            self.pick_srv = rospy.ServiceProxy(self.pick_srv, SetBool)
        
        elif self.operation == "place":
            rospy.loginfo("Placing the cube...")
            self.place_srv = rospy.ServiceProxy(self.place_srv, SetBool)     

        else:
            rospy.logerr("Invalid operation")       

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(pick_and_place, self).__init__("Pick_or_place")


    def update(self):

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
                rospy.loginfo("Place service successful")
                return pt.common.Status.SUCCESS

            # if failed
            elif not self.place_srv_req.success:
                return pt.common.Status.FAILURE

            # if still trying
            else:
                return pt.common.Status.RUNNING