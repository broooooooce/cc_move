#!/usr/bin/env python

# motion.py - Move to cartesian coordinates function module

# Bruce Stracener - University of Arkansas for Medical Sciences
# started 01/29/18

# This module contains functions that may be called from other scripts
# to move Baxter around in cartesian space. The advantage of this is no longer
# having to copy/paste all of this code into any script calling for such
# capabilities. Instead, at the beginning, simply add:
#       import motion
# and then use this syntax throughout to program:
#       motion.ccmove(<x> <y> <z> <ox> <oy> <oz> <ow> <speed> '<limb>')

# IK service code usage based upon IK_Pick_and_Place_Demo from Rethink Robotics

import rospy                # ROS python API
import baxter_interface     # Baxter Python API
import argparse             # Parse command-line arguments
import sys                  # System-specific parameters and functions
import copy                 # Allows deep & shallow copying of mutable objects
import struct               # Convert between strings and binary data

from geometry_msgs.msg import (   
    PoseStamped,                  
    Pose,                   # Allows use of the message types which
    Point,                  # Baxter uses to publish his pose information 
    Quaternion,
)
from std_msgs.msg import (
    Header,                 # Used to communicate timestamped data
    Empty,                  # Allows for null msgs (void in C/C++ terms)
)
from baxter_core_msgs.srv import (
    SolvePositionIK,        # Msg type returned from IK service w/ joint angles
    SolvePositionIKRequest, # Msg type sent to IK service w/ cartesian coordinates
)

class MoveToCC(object):
    def __init__(self, limb, speed, verbose=True):
        self._limb_name = limb                          # string
        self._verbose = verbose                         # bool
        self._limb = baxter_interface.Limb(limb)        # selects limb
        self._limb.set_joint_position_speed(speed)      # sets overall speed
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:       # if TRUE
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def _servo_to_pose(self, pose):
        print '_servo_to_pose'
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

def ccmove(nx, ny, nz, nox, noy, noz, now, nspeed, nlimb):
    rospy.init_node("move_to_cartesian_coordinate", anonymous=True)

    mtocc = MoveToCC(str(nlimb), float(nspeed))
     
    desired_orientation = Quaternion(
            x=float(nox),    
            y=float(noy),   
            z=float(noz),    
            w=float(now))  

    desired_pose = Pose(
        position=Point(
            x=float(nx), 
            y=float(ny), 
            z=float(nz)),
        orientation=desired_orientation)
 
    mtocc._servo_to_pose(desired_pose)

    return 0

