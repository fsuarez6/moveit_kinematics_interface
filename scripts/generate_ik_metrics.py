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
    self.folder_key = self.read_parameter('~folder_key', '6575ecddff8e44245a6d08f7e6a232f6')
    self.file_key = self.read_parameter('~file_key', '119f22cd837e05b6686bbe6d990b6eb4')
    self.ref_frame = self.read_parameter('~ref_frame', 'left_arm_mount')
    self.dofs = int(self.read_parameter('~dofs', 7))
    # Subscribe to kinematic services
    self.metrics_srv_name = self.read_parameter('~metrics_service', '/baxter/kinematics_services/get_ik_metrics')
    self.loginfo('Reference frame: %s' % self.ref_frame)
    self.loginfo('Waiting for %s service' % self.metrics_srv_name)
    rospy.wait_for_service(self.metrics_srv_name)
    self.metrics_srv = rospy.ServiceProxy(self.metrics_srv_name, GetPoseMetrics)
  
  def load_hdf5(self):
    filename = self.get_filename('reachability')
    self.loginfo('Loading file:\n' + filename)
    f = None
    try:
      f = h5py.File(filename, 'r')
      if f['version'].value != self.get_version():
        self.logerror('File version is wrong %s!=%s ', f['version'], self.get_version())
        return False
      self.reachabilitystats = f['reachabilitystats'].value
      self.reachabilitydensity3d = f['reachabilitydensity3d']
      self.reachability3d = f['reachability3d']
      self.pointscale = f['pointscale'].value
      self.xyzdelta = f['xyzdelta'].value
      self.quatdelta = f['quatdelta'].value
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
    filename = self.get_filename('ik_metrics')
    # Try to make the dir (It should exists although)
    try:
      os.makedirs(os.path.split(filename)[0])
    except OSError, e:
      pass
    f=h5py.File(filename,'w')
    try:
      f['version'] = self.get_version()
      f['orientations'] = self.reachabilitystats[:, :4]
      f['positions'] = self.reachabilitystats[:, 4:7]
      f['metrics'] = self.metrics
      f['joint_states'] = self.joint_states
    finally:
      if f:
        f.close()
        
  def get_version(self):
    return 5
    
  def has(self):
    return len(self.reachabilitydensity3d) > 0 and len(self.reachability3d) > 0 and len(self.reachabilitystats) > 0
        
  def get_filename(self, database):
    # usage e.g: self.get_filename('reachability')
    #~ folder_key = '92388cabb79ce4a7e4498b08e3e28901'
    #~ file_key = '9511c593edd7a5f1f47a9ce38cdc9db9'
    filename = '~/.openrave/robot.%s/%s.%s.pp' % (self.folder_key, database, self.file_key)
    return os.path.expanduser(filename)
  
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
    
  def array2pose(self, data):
    #~ reachabilitystats
    #~ rotation     position  solutions 
    #~ [w x y z]    [x y z]   [n]
    pose = Pose()
    pose.orientation.w = data[0]
    pose.orientation.x = data[1]
    pose.orientation.y = data[2]
    pose.orientation.z = data[3]
    pose.position.x = data[4]
    pose.position.y = data[5]
    pose.position.z = data[6]
    return pose
  
  def run(self):
    if os.path.isfile(self.get_filename('ik_metrics')):
      answer = 'a'
      while answer not in ['y', 'Y', 'n', 'N']:
        print self.get_filename('ik_metrics')
        answer = raw_input('The ik_metrics file already exist, Overwrite? (y/n): ')
        if answer in ['n', 'N']:
          return
    try:
      self.loginfo('Running Database metrics')
      if not self.load_hdf5():
        self.logerror('Failed loading:\n' + self.get_filename('reachability'))
        return
      # Allocate the arrays
      self.num_poses = self.reachabilitystats.shape[0]
      self.metrics = np.zeros([self.num_poses, 3])
      self.joint_states = np.zeros([self.num_poses, self.dofs])
      self.loginfo('Processing [%d] poses. It may take a while' % self.num_poses)
      # Process all the poses in the file using the kinematic service
      req = GetPoseMetricsRequest()
      calculated = 0
      tmp_count = 0
      for i, row in enumerate(self.reachabilitystats):
        if (i % 1000 == 0):
          self.loginfo('Evaluated: %d/%d \t solved: %d/%d' % (i, self.num_poses, tmp_count, i))
        req.header.stamp = rospy.Time.now()
        req.header.frame_id = self.ref_frame
        req.link_name = 'left_gripper'
        req.pose = self.array2pose(row)
        try:
          res = self.metrics_srv(req)
          if res.found_ik and res.found_group:
            self.metrics[i, 0] = res.manipulability
            self.metrics[i, 1] = res.manipulability_index
            self.metrics[i, 2] = res.max_payload
            self.joint_states[i, :] = res.joint_states.position
            tmp_count += 1
        except rospy.ServiceException, e:
          self.logwarn('Service did not process request: %s' % str(e))
        # Check for shutdowns
        if rospy.is_shutdown():
          return
      # Show the result in the console
      calculated = len(self.metrics[:, 1].nonzero()[0])
      self.loginfo('Done: %d metrics were calculated' % calculated)
      if calculated > 0:
        minval, maxval = self.min_max_nonzero(self.metrics[:, 0])
        self.loginfo('Manipulability: Min [%f], Max [%f]' % (minval, maxval))
        minval, maxval = self.min_max_nonzero(self.metrics[:, 1])
        self.loginfo('Index: Min [%f], Max [%f]' % (minval, maxval))
      
    finally:
      if self.file_id is not None:
        self.file_id.close()

    # Save the calculated metrics
    if calculated > 0:
      self.save_hdf5()
    
if __name__ == '__main__':
  rospy.init_node('generate_ik_metrics', anonymous=True)
  database = DatabaseMetrics()
  database.run()
