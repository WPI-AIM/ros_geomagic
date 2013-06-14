import roslib
roslib.load_manifest("phantom_omni")
import rospy
import tf
import tf.transformations as tr

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Wrench

import math
import numpy as np

def tf2mat(tf_trans):
    (trans, rot) = tf_trans
    return np.matrix(tr.translation_matrix(trans)) * np.matrix(tr.quaternion_matrix(rot))

def mat2pose(m):
    trans = tr.translation_from_matrix(m)
    rot   = tr.quaternion_from_matrix(m)
    p = Pose()
    p.position.x = trans[0]
    p.position.y = trans[1]
    p.position.z = trans[2]
    p.orientation.x = rot[0]
    p.orientation.y = rot[1]
    p.orientation.z = rot[2]
    p.orientation.w = rot[3]
    return p

rospy.init_node("pr2_force_feedback")
pub = rospy.Publisher('pr2_master', PoseStamped)
wpub = rospy.Publisher('force_feedback', Wrench)
r = rospy.Rate(60)

listener = tf.TransformListener()
listener.waitForTransform("/world", "/omni1_link6", rospy.Time(), rospy.Duration(4.0))
print 'running.'
while not rospy.is_shutdown():
    w_T_6 = tf2mat(listener.lookupTransform('/world', '/omni1_link6', rospy.Time(0)))
    qm = np.matrix(tr.quaternion_matrix(tr.quaternion_from_euler(0, math.pi, 0)))
    tm = np.matrix(tr.translation_matrix([-.134, 0, 0]))
    tip_6 = tm * qm
    tip_world = w_T_6 * tip_6

    ps = PoseStamped()
    ps.header.frame_id = 'world'
    ps.header.stamp = rospy.get_rostime()
    ps.pose = mat2pose(tip_world)
    pub.publish(ps)
    r.sleep()

    #Input force in link6's frame
    #Output force in omni base frame
    #wr = Wrench()
    #wr.force.x = np.random.rand()*2 - 1
    #wr.force.y = np.random.rand()*2 - 1
    #wr.force.z = np.random.rand()*2 - 1
    #wpub.publish(wr)

    wr = Wrench()
    wr.force.x = 0 
    wr.force.y = 0 
    wr.force.z = 0 
    wpub.publish(wr)



