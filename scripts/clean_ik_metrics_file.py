#! /usr/bin/env python
import rospy
# Messages
from moveit_kinematics_interface.srv import GetPoseMetrics, GetPoseMetricsRequest, GetPoseMetricsResponse
from geometry_msgs.msg import Pose
# HDF5
import h5py, os
# Math
import numpy as np

class DatabaseMetrics:
  def __init__(self): 
    self.reachabilitystats = None
    self.reachabilitydensity3d = None
    self.reachability3d = None
    self.pointscale = None
    self.xyzdelta = None
    self.quatdelta = None
    # New data
    self.metrics = None
    self.joint_states = None
    self.file_id = None
    # Read parameters
    #~ self.filename = self.read_parameter('~filename', '~/.openrave/robot.6575ecddff8e44245a6d08f7e6a232f6/ik_metrics.119f22cd837e05b6686bbe6d990b6eb4.pp')
    self.filename = self.read_parameter('~filename', '~/.openrave/robot.6575ecddff8e44245a6d08f7e6a232f6/ik_metrics.dc12ea999aabf684e4dab37176899cd2.pp')
    self.filename = os.path.expanduser(self.filename)
  
  def load_hdf5(self):
    filename = self.filename
    self.loginfo('Loading file:\n' + filename)
    f = None
    try:
      f = h5py.File(filename, 'r')
      if f['version'].value != self.get_version():
        self.logerror('File version is wrong %s!=%s ', f['version'], self.get_version())
        return False
      self.joint_states = f['joint_states'].value
      self.metrics = f['metrics'].value
      self.orientations = f['orientations'].value
      self.positions = f['positions'].value
      self.file_id = f
      f = None
      return self.has()
    except Exception, e:
      self.logerror('LoadHDF5 for %s: %s' % (filename, str(e)))
      return False
    finally:
      if f is not None:
        f.close()
  
  def save_hdf5(self):
    filename = self.filename
    # Try to make the dir (It should exists although)
    try:
      os.makedirs(os.path.split(filename)[0])
    except OSError, e:
      pass
    f=h5py.File(filename,'w')
    try:
      f['joint_states'] = self.joint_states[self.mask]
      f['metrics'] = self.metrics[self.mask]
      f['positions'] = self.positions[self.mask]
      f['orientations'] = self.orientations[self.mask]
      f['version'] = self.get_version()
    finally:
      if f:
        f.close()
        
  def get_version(self):
    return 5
    
  def has(self):
    return len(self.joint_states) > 0 and len(self.metrics) > 0 and len(self.orientations) > 0 and len(self.positions) > 0
  
  def read_parameter(self, name, default):
    if not rospy.has_param(name):
      self.logwarn('Parameter [%s] not found, using default: %s' % (name, default))
    return rospy.get_param(name, default)
  
  def loginfo(self, msg):
    rospy.loginfo(msg)
    
  def logwarn(self, msg):
    rospy.logwarn(msg)
  
  def logerror(self, msg):
    rospy.logerr(msg)
    
  def min_max_nonzero(self, a):
    return np.min(a[np.nonzero(a)]), np.max(a[np.nonzero(a)])
  
  def run(self):
    try:
      self.loginfo('Cleaning Database')
      if not self.load_hdf5():
        self.logerror('Failed loading:\n' + self.filename)
        return
      total_metrics = len(self.metrics)
      self.loginfo('Metrics before: %d' % len(self.metrics))
      self.mask = np.ones(total_metrics, dtype=bool)
      removed_metrics = 0      
      for i, row in enumerate(self.metrics):
        if np.allclose(row, np.array([0.0,0.0,0.0])):
          self.mask[i] = False
          removed_metrics += 1
        if (i % 10000 == 0):
          self.loginfo('Evaluated: %d/%d' % (i, total_metrics))
        # Check for shutdowns
        if rospy.is_shutdown():
          return
    finally:
      if self.file_id is not None:
        self.file_id.close()
    
    # Save the calculated metrics
    self.loginfo('Metrics after: %d' % (len(self.metrics) - removed_metrics))
    self.save_hdf5()
    
if __name__ == '__main__':
  rospy.init_node('generate_ik_metrics', anonymous=True)
  database = DatabaseMetrics()
  database.run()
