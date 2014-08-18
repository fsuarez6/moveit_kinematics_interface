#! /usr/bin/env python
import rospy, math
import numpy as np
import PyKDL

RAD_TOLERANCE = 0.0001

# Helper methods
def harmonize(q_in):
  q_out = q_in
  for idx in range(q_in.rows()):
    while (q_in[idx] > 2*math.pi):
      q_out[idx] -= 2*math.pi;
    while (q_in[idx] < -2*math.pi):
      q_out[idx] += 2*math.pi;
  return q_out

def kdl_to_mat(data):
  mat =  np.mat(np.zeros((data.rows(), data.columns())))
  for i in range(data.rows()):
    for j in range(data.columns()):
      mat[i,j] = data[i,j]
  return mat

def nearly_equal(a,b,sig_fig=5):
  return (a==b or int(a*10**sig_fig) == int(b*10**sig_fig))

def obeys_limits(q, q_min, q_max):
  obeys_limits = True
  for i in range(q.rows()):
    if (q[i] < (q_min[i]-RAD_TOLERANCE)) or (q[i] > (q_max[i]+RAD_TOLERANCE)):
      obeys_limits = False
      break
  return obeys_limits

def print_kdl_chain(chain):
  for idx in range(chain.getNrOfSegments()):
    print '* ' + chain.getSegment(idx).getName()

def read_parameter(name, default):
  if not rospy.has_param(name):
    rospy.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
  return rospy.get_param(name, default)

def split_pose_msg(pose):
  pos = [pose.position.x, pose.position.y, pose.position.z]
  rot = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
  return pos, rot

# Classes
class JointLimits(object):
  def __init__(self, joint):
    self.lower = joint.limit.lower
    self.upper = joint.limit.upper
    self.effort = joint.limit.effort
    self.velocity = joint.limit.velocity
  
  def __str__(self):
    msg = 'lower: [%s] upper: [%s] effort: [%s] velocity: [%s]' % (self.lower, self.upper, self.effort, self.velocity)
    return msg


