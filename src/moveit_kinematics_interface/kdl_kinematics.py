#!/usr/bin/python

import numpy as np
import PyKDL

import rospy

from moveit_kinematics_interface.kdl_parser import kdl_tree_from_urdf_model
from moveit_kinematics_interface.utils import *
from urdf_parser_py.urdf import URDF

IK_SOLVERS = {'NR':'Newton-Raphson', 'LMA':'Levenberg-Marquardt', 'CCD':'Cyclic Coordinate Descent'}

class Kinematics(object):
  """
  Kinematics with PyKDL
  """
  def __init__(self, tip_link, base_link=None, ik_solver='NR'):
    # TODO: Read values from parameter server
    self._iterations = 100
    self._epsilon = 1e-3
    # TODO: Validate that the URDF exists
    self._urdf = URDF.from_parameter_server(key='robot_description')
    self._kdl_tree = kdl_tree_from_urdf_model(self._urdf)
    # TODO: Validate that the base and tip links exist
    if base_link:
      self._base_link = base_link
    else:
      self._base_link = self._urdf.get_root()
    self._tip_link = tip_link
    self._chain = self._kdl_tree.getChain(self._base_link, self._tip_link)
    self._ik_solver = ik_solver
    self._ik_solutions = []

    # Get the joint names
    self._num_joints = self._chain.getNrOfJoints()
    num_segments = self._chain.getNrOfSegments()
    self._joint_names = []
    for idx in range(num_segments):
      kdl_joint = self._chain.getSegment(idx).getJoint()
      if kdl_joint.getType() != PyKDL.Joint.None:
        self._joint_names.append(kdl_joint.getName())

    # Initialize required JntArray's
    self._jnt_positions = PyKDL.JntArray(self._num_joints)
    self._q_min = PyKDL.JntArray(self._num_joints)
    self._q_max = PyKDL.JntArray(self._num_joints)

    # Get the joint limits
    for idx, name in enumerate(self._joint_names):
      joint = self._urdf.joint_map[name]
      self._q_min[idx] = joint.limit.lower
      self._q_max[idx] = joint.limit.upper
      # Set the current jnt_positions in the middle of the joint limits
      self._jnt_positions[idx] = (joint.limit.upper + joint.limit.lower) / 2.0
    # KDL Solvers
    self._fk_p_kdl = PyKDL.ChainFkSolverPos_recursive(self._chain)
    self._fk_v_kdl = PyKDL.ChainFkSolverVel_recursive(self._chain)
    self._ik_v_kdl = PyKDL.ChainIkSolverVel_pinv(self._chain)
    if ik_solver=='NR':
      self._ik_p_kdl = PyKDL.ChainIkSolverPos_NR_JL(self._chain, self._q_min, self._q_max, self._fk_p_kdl, 
                        self._ik_v_kdl, self._iterations, self._epsilon)
    elif ik_solver=='LMA':
      self._ik_p_kdl = PyKDL.ChainIkSolverPos_LMA(self._chain, self._epsilon, self._iterations)
    elif ik_solver=='CCD':
      rospy.logerr('CCD IK Solver needs to be implemented' % ik_solver)
    else:
      rospy.logerr('Unknown IK solver: %s' % ik_solver)
    self._jac_kdl = PyKDL.ChainJntToJacSolver(self._chain)
    self._dyn_kdl = PyKDL.ChainDynParam(self._chain, PyKDL.Vector.Zero())

    # Print useful info
    rospy.loginfo('Base Link:   %s' % self._base_link)
    rospy.loginfo('Tip Link:    %s' % self._tip_link)
    rospy.loginfo('Nr Joints:   %s' % self._num_joints)
    rospy.loginfo('IK Solver:   %s' % IK_SOLVERS[ik_solver])

  def _joints_in_limits(self, q):
    lower_lim = self._q_min
    upper_lim = self._q_max
    return np.all([q >= lower_lim, q <= upper_lim], 0)
    
  def _random_joint_positions(self):
    zip_lims = zip(self._q_min, self._q_max)
    return build_kdl_array([np.random.uniform(min_lim, max_lim) for min_lim, max_lim in zip_lims])
    
  def _valid_positions_size(self, q):
    valid = True
    if type(q) == PyKDL.JntArray:
      q_len = q.rows() 
    else:
      q_len = len(q)
    if q_len != self._num_joints:
      rospy.logwarn('Unexpected number of values. Received: %d, Expected: %d' % (q_len, self._num_joints))
      valid = False
    return valid

  def get_end_effector_transform(self, joint_positions=None):
    end_frame = PyKDL.Frame()
    if joint_positions:
      self._fk_p_kdl.JntToCart(build_kdl_array(joint_positions), end_frame)
    else:
      self._fk_p_kdl.JntToCart(self._jnt_positions, end_frame)
    return end_frame

  def get_joint_names(self):
    return self._joint_names
    
  def get_joint_positions(self):
    return self._jnt_positions

  def print_robot_description(self):
    nf_joints = 0
    for j in self._urdf.joints:
      if j.type != 'fixed':
        nf_joints += 1
    rospy.loginfo('URDF non-fixed joints: %d;' % nf_joints)
    rospy.loginfo('URDF total joints: %d' % len(self._urdf.joints))
    rospy.loginfo('URDF links: %d' % len(self._urdf.links))
    rospy.loginfo('KDL joints: %d' % self._kdl_tree.getNrOfJoints())
    rospy.loginfo('KDL segments: %d' % self._kdl_tree.getNrOfSegments())

  def search_ik(self, goal_pose, initial_seed, timeout=0.01):
    st_time = rospy.get_time()
    self._ik_solutions = []
    found = 0
    change_seed = False
    while (rospy.get_time() - st_time < timeout) and not rospy.is_shutdown():
      if change_seed:
        seed = self._random_joint_positions()
      else:
        seed = initial_seed
        change_seed = True
      ik_found = self.set_from_ik(goal_pose, seed)
      if ik_found >= 0:
        self._ik_solutions.append(self.get_joint_positions())
        found += 1
    return found

  def set_from_ik(self, goal_pose, seed=None):
    # Populate seed with current angles if not provided
    if seed != None and self._valid_positions_size(seed):
      seed_array = build_kdl_array(seed)
    else:
      seed_array = self._random_joint_positions()
    # Make IK Call
    result_angles = PyKDL.JntArray(self._num_joints)
    ik_found = self._ik_p_kdl.CartToJnt(seed_array, goal_pose, result_angles)
    if ik_found >= 0:
      if self._ik_solver == 'LMA':
        result_angles = harmonize(result_angles)
        self._jnt_positions = result_angles
      if self._joints_in_limits(result_angles):
        self._jnt_positions = result_angles
      else:
        ik_found = -4
    return ik_found

  def set_joint_positions(self, joint_positions):
    if self._valid_positions_size(joint_positions):
      self._jnt_positions = build_kdl_array(joint_positions)
