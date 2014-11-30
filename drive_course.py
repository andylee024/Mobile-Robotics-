#!/usr/bin/env python
import roslib; roslib.load_manifest('lab3')
import rospy
import rospkg
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState
from blobfinder.msg import MultiBlobInfo3D
from transform2d import *
import transform2d
import tf
import math
import numpy as np

"""
Initialization Statements
"""

# control at 100Hz
CONTROL_PERIOD = rospy.Duration(0.01)

# minimum duration of safety stop (s)
STOP_DURATION = rospy.Duration(1.0)

# change actions every 0.5s
ACTION_DURATION = rospy.Duration(0.5)

# minimum distance of cones to make gate (m)
MIN_GATE_DIST = 0.45

# maximum distance of cones to make gate (m)
MAX_GATE_DIST = 0.75

# minimum number of pixels to identify cone (px)
MIN_CONE_AREA = 200



"""
Controller class
"""


# define a class to handle our simple controller
class Controller:

    # initialize our controller
    def __init__(self):
        #test statement

        # initialize our ROS node
        rospy.init_node('starter')
    
        # record whether we should stop for safety
        self.should_stop = 0
        self.time_of_stop = rospy.get_rostime() - STOP_DURATION

        # set up a TransformListener to get odometry information
        self.odom_listener = tf.TransformListener()

        # initialize a dictionary of two empty lists of cones. each
        # list should hold world-frame locations of cone in XY
        self.cone_locations = dict(orange=[], green=[])

        # initialize dictionary for list of tape locations 
        self.tape_locations = dict(blue=[])

        # this bool will be set to True when we get a new cone message
        self.cones_updated = False
        self.tape_updated = False 

        # queue of instructions
        self.cmds = self.parsecmds()
        rospy.loginfo("self.cmds")
        rospy.loginfo(self.cmds[0][0])

        self.state = self.cmds[0][0]


        #intialize robot parameters 
        self.alpha = 1.16 #1.17
        self.k_x = 0.28
        self.k_theta = 2.85

        self.curr_gate = None
        self.curr_tape = None


        # set up publisher for commanded velocity
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity',
                                           Twist)

        # set up subscriber for sensor state for bumpers/cliffs
        rospy.Subscriber('/mobile_base/sensors/core',
                         SensorState, self.sensor_callback)

        # set up subscriber for orange cones 
        rospy.Subscriber('/blobfinder/orange_tape/blobs3d',
                         MultiBlobInfo3D,
                         self.blobs3d_callback,
                         callback_args='orange')

        # set up subscriber for green cones 
        rospy.Subscriber('/blobfinder/green_tape/blobs3d',
                         MultiBlobInfo3D,
                         self.blobs3d_callback,
                         callback_args='green')

        # set up subscriber for tape detection - blue
        rospy.Subscriber('/blobfinder/blue_tape/blobs3d',
                         MultiBlobInfo3D,
                         self.blobs3d_callback,
                         callback_args='blue')

        # set up our trivial 'state machine' controller
        rospy.Timer(CONTROL_PERIOD,
                    self.control_callback)

        
    



    """
    Callbacks
    """

    # called when sensor msgs received - just copy sensor readings to
    # class member variables
    def sensor_callback(self, msg):

        if msg.bumper & SensorState.BUMPER_LEFT:
            rospy.loginfo('***LEFT BUMPER***')
        if msg.bumper & SensorState.BUMPER_CENTRE:
            rospy.loginfo('***MIDDLE BUMPER***')
        if msg.bumper & SensorState.BUMPER_RIGHT:
            rospy.loginfo('***RIGHT BUMPER***')
        if msg.cliff:
            rospy.loginfo('***CLIFF***')

        if msg.bumper or msg.cliff:
            self.should_stop = True
            self.time_of_stop = rospy.get_rostime()
        else:
            self.should_stop = False


    # called when a blob message comes in
    def blobs3d_callback(self, msg, color):
        #rospy.loginfo("blobs3d Callback")
        
        T_world_from_robot = self.get_current_pose()

        if T_world_from_robot is None:
            rospy.logwarn('no xform yet in blobs3d_callback')
            return

        #blue-tape execution
        if color == 'blue':
            min_dist = 100
            r_pos = (0,0) #robot position in robot frame
            #self.curr_tape = None

            for blob3d in msg.blobs:
                if blob3d.have_pos and blob3d.blob.area > 0:
                    tape_pos = (blob3d.position.z, -blob3d.position.x) #tape position in robot frame
                    
                    #calculate distance from tape to robot
                    dist = self.calc_dist(r_pos,tape_pos)

                    #update min_dist and set tape to follow 
                    if dist < min_dist:
                        min_dist = dist
                        self.curr_tape = blob3d

            self.tape_updated = True

        #green/orange cone execution
        blob_locations = []

        for blob3d in msg.blobs:
            if blob3d.have_pos and blob3d.blob.area > MIN_CONE_AREA:
                blob_in_robot_frame = (blob3d.position.z, -blob3d.position.x)
                blob_locations.append(T_world_from_robot * blob_in_robot_frame)

        self.cone_locations[color] = blob_locations
        self.cones_updated = True











    """
    Control Callback

    Robot has 4 states 
    1) Cone Seeking 
        - finds cones to pursue 
    
    2) Gate Seeking
        - uses pure pursuit algorithm to go through gate
    
    3) Tape Seeking
        - follow tape 

    4) Left/Right Movement
        - follows direction of cone based on command lists document 
    """

    # called periodically to do top-level coordination of behaviors
    def control_callback(self, timer_event=None):
        
        # initialize vel to 0, 0
        cmd_vel = Twist()
        time_since_stop = rospy.get_rostime() - self.time_of_stop
        cur_pose = self.get_current_pose()

        if self.should_stop or time_since_stop < STOP_DURATION:
            rospy.loginfo('stopped')
        
        else:
            rospy.loginfo(self.curr_gate)

            """ Cone Seeking """
            if self.state != 'dummy state':
                #procedure to update gates, set one gate to pursue
                if self.cones_updated and cur_pose is not None:
                    all_gates = self.find_gates(cur_pose)
                    if len(all_gates) !=0:
                        if self.find_closest_gate(all_gates) != None:
                            self.curr_gate = self.find_closest_gate(all_gates)

                        self.state = 'gate'

            """ Gate Seeking """
            ### Pure Pursuit Algorithm ###
            if self.curr_gate != None:

                T_robot_from_world = cur_pose.inverse()
                T_world_from_gate = self.curr_gate[0]

                ### Step 1 - Transform to line frame (simplify calculations) ###
                w_r = [cur_pose.x,cur_pose.y,1] #robot's world frame coordinates           
                p_r = T_world_from_gate.inverse() * w_r #transform w_r to line frame

                ### Step 2 - calculate point to pursue, update positions
                p_c = [p_r[0],0,1] #point closest to robot on line in line frame
                p_d = [p_c[0]+self.alpha, 0, 1] #determine point to pursue in line frame


                ### Step 3 - transition back into world frame -> robot frame 
                r_d = T_robot_from_world * (T_world_from_gate * p_d) #get point to pursue in robot frame
                u,v = r_d[0],r_d[1]
           
                #pursue point in world frame
                cmd_vel.linear.x = self.k_x + self.speed_boost(v,u)
                cmd_vel.angular.z = self.k_theta*(v/u)

                #safety condition so robot does not go too quickly
                if cmd_vel.linear.x > .6:
                    cmd_vel.linear.x = .6
              
                #pass gate
                if p_c[0] > 0:
                    #update new behavior
                    self.cmds.pop(0)
                    self.curr_gate = None
                    self.state = self.cmds[0][0]
                    rospy.loginfo(self.cmds[0])
                
            else:
                self.state = self.cmds[0][0]


            """ Tape Seeking """
            if self.state == 'tape':
                #rospy.loginfo("tape found")

                if self.tape_updated and self.curr_tape is not None:
                    rospy.loginfo("following tape")
                    
                    theta = self.getTheta(self.curr_tape)
                    
                    # determine how area, x, and y, will inform our desired linear and angular vel
                    cmd_vel.linear.x = 0.7
                    cmd_vel.angular.z = theta
                
            """ Left Movement """
            if self.state == 'left':

                rospy.loginfo('following left')

                cmd_vel.linear.x = 0.3
                cmd_vel.angular.z = 0.4


            """ Right Movement """
            if self.state == 'right':

                rospy.loginfo('following right')

                cmd_vel.linear.x = 0.3
                cmd_vel.angular.z = -0.4


        #rospy.loginfo(cmd_vel)
        rospy.loginfo('state: %s', self.state)
        #rospy.loginfo(cmd_vel)
        self.cmd_vel_pub.publish(cmd_vel)









    """
    Helper Functions
    """

     # called during initialization to set up queue of instructions
    def parsecmds(self):
        datadir = rospkg.RosPack().get_path('lab4')
        print 'data dir is ', datadir
        f = open(datadir+'/data/final_course.txt', 'r')

        actions = []
        for line in f:
           actions.append(line.split())

        return actions

    
    #Determines theta turn radius while robot pursues tape 
    def getTheta(self, blob3d):
        if blob3d == None:
            return 0

        area = blob3d.blob.area
        x = blob3d.blob.cx

        #theta calculation
        k_p = (20*self.k_x)/640
        theta = k_p*(320-x)

        return theta

    # get current pose from TransformListener
    def get_current_pose(self):
        
        try:
            ros_xform = self.odom_listener.lookupTransform(
                '/odom', '/base_footprint',
                rospy.Time(0))

        except tf.LookupException:

            return None

        return transform2d_from_ros_transform(ros_xform)

    
    # called when we need to find a gate
    def find_gates(self, cur_pose):
        # build a list of all cones by combining colors
        all_cones = []
        for color in ['orange', 'green']:
            for world_xy in self.cone_locations[color]:
                all_cones.append( (color, world_xy) )

        # get the inverse transformation of cur pose to be able to map
        # points back into robot frame (so we can assess which gates
        # are left and right)
        T_robot_from_world = cur_pose.inverse()

        # build a list of all gates by investigating pairs of cones
        all_gates = []

        # for each cone
        for i in range(len(all_cones)):

            # for cone i: get color, world pos, robot pos
            (ci, wxy_i) = all_cones[i]
            rxy_i = T_robot_from_world * wxy_i #transform i (cone) i_w -> i_r

            # for every other cone
            for j in range(i):

                # get color, world pos, robot pos
                (cj, wxy_j) = all_cones[j]
                rxy_j = T_robot_from_world * wxy_j

                # get distance between cone pair
                dist_ij = transform2d.distance_2d(wxy_i, wxy_j)
                
                # if in the range that seems reasonable for a gate:
                if dist_ij > MIN_GATE_DIST and dist_ij < MAX_GATE_DIST:

                    # get midpoint of cones
                    midpoint = 0.5 * (wxy_i + wxy_j)

                    # sort the cones left to right (we want left to
                    # have a larger Y coordinate in the robot frame)
                    if rxy_i[1] > rxy_j[1]:
                        cl, wxy_l, cr, wxy_r = ci, wxy_i, cj, wxy_j
                    else:
                        cl, wxy_l, cr, wxy_r = cj, wxy_j, ci, wxy_i

                    # the direction of the gate frame's local Y axis
                    # is obtained by subtracting the right cone from
                    # the left cone.
                    gate_ydir = (wxy_l - wxy_r) 

                    # the angle of rotation for the gate is obtained
                    # by the arctangent given here
                    gate_theta = math.atan2(-gate_ydir[0], gate_ydir[1])
                    
                    T_world_from_gate = Transform2D(midpoint[0], midpoint[1], 
                                                   gate_theta)

                    #rospy.loginfo('found {0},{1} gate with T_world_from_gate = {2}'.format(
                    #        cl, cr, T_world_from_gate))

                    gate = (T_world_from_gate, cl, cr, wxy_l, wxy_r, dist_ij)

                    all_gates.append(gate)

        self.cones_updated = False
        return all_gates

    #optimally adjust robots speed based on distance from cone
    def speed_boost(self,v,u):
        ratio = math.fabs((v/u))
        
        if ratio > 0.25:
            boost = 0

        elif ratio<= 0.25 and ratio > 0.10:
            boost = 0.1

        elif ratio <= 0.10 and ratio > 0.05:
            boost = 0.2

        else:
            boost = 0.3 
        
        return boost

    #calculates distance between two points (x,y)
    def calc_dist(self,p1,p2):
        x_1 = p1[0]
        y_1 = p1[1]
        x_2 = p2[0]
        y_2 = p2[1]

        dist = math.sqrt( (x_1-x_2)**2 + (y_1 - y_2)**2 )

        return dist 


    # called to choose the next gate to drive through 
    def find_closest_gate(self, all_gates):

        # x,y location of robot
        cur_pose = self.get_current_pose()

        closest_dist = 1000 

        for gate in all_gates:
            if gate[1]==self.cmds[0][1] and gate[2]==self.cmds[0][2]:

                midpoint = 0.5 * (gate[3] + gate[4])
                #distance robot to midpoint of cone_i
                r_point = (cur_pose.x, cur_pose.y)
                #calculate distance
                dist = self.calc_dist(r_point,midpoint)
                #compare/sort
                if dist < closest_dist:
                    closest_dist = dist
                    next_gate = gate

                return next_gate
        return None

    
    # called by main function below (after init)
    def run(self):
        # timers and callbacks are already set up, so just spin
        rospy.spin()
        # if spin returns we were interrupted by Ctrl+C or shutdown
        rospy.loginfo('goodbye')

        

# main function
if __name__ == '__main__':
    try:
        ctrl = Controller()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass

