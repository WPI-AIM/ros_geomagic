import roslib
roslib.load_manifest("phantom_omni")
import rospy
import tf
import tf.transformations as tr
import hrl_lib.tf_utils as tfu

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Wrench

import math
import numpy as np

rospy.init_node("omni_potential_well")
wpub = rospy.Publisher('omni1_force_feedback', Wrench)

def pose_cb(ps):
    m_f, frame = tfu.posestamped_as_matrix(ps)
    m_o1 = tfu.transform('/omni1', frame, listener) * m_f
    ee_point = np.matrix(tr.translation_from_matrix(m_o1)).T
    center = np.matrix([-.10, 0, .30]).T
    dif = 30*(center - ee_point)
    #force_dir = dif / np.linalg.norm(dif)
    force_o1 = dif #force_dir * np.sum(np.power(dif, 2))

    force_s = tfu.transform('/omni1_sensable', '/omni1', listener) * np.row_stack((force_o1, np.matrix([1.])))
    print np.linalg.norm(center - ee_point)

    wr = Wrench()
    wr.force.x = force_s[0]
    wr.force.y = force_s[1]
    wr.force.z = force_s[2]
    wpub.publish(wr)

rospy.Subscriber('/omni1_pose', PoseStamped, pose_cb)
r = rospy.Rate(1)

listener = tf.TransformListener()
listener.waitForTransform("/omni1", "/omni1_sensable", rospy.Time(), rospy.Duration(4.0))
listener.waitForTransform("/omni1", "/omni1_link6", rospy.Time(), rospy.Duration(4.0))
print 'running.'


well_center = None
while not rospy.is_shutdown():
    r.sleep()





























    #get current pose of the tip, subtract it from a center, rotate this into the sensable frame
    #tip_omni1 = w_T_6 * tip_6

    #force_omni1 = (w_T_6[0:3,0:3] * np.matrix([-2., 0, 0]).T)
    #force_sensable = transform('/omni1_sensable', '/omni1', listener) * force_omni1

    #wr = Wrench()
    #wr.force.x = force_sensable[0]
    #wr.force.y = force_sensable[1]
    #wr.force.z = force_sensable[2]
    #print wr.force.z
    #wpub.publish(wr)
    #qm = tfu.quaternion_from_matrix(tr.quaternion_from_euler(0, math.pi, 0))
    #tm = tfu.translation_matrix([-.134, 0, 0])
    #tip_6 = tm * qm
    #tip_omni1 = w_T_6 * tip_6
#trans, rot = listener.lookupTransform('/omni1_sensable', '/omni1', rospy.Time(0))
#quat_mat = np.matrix(tr.quaternion_matrix(rot))
#force_sensable = quat_mat[0:3, 0:3] * force_omni1
#def tf2mat(tf_trans):
#    (trans, rot) = tf_trans
#    return np.matrix(tr.translation_matrix(trans)) * np.matrix(tr.quaternion_matrix(rot))

#def mat2pose(m):
#    trans = tr.translation_from_matrix(m)
#    rot   = tr.quaternion_from_matrix(m)
#    p = Pose()
#    p.position.x = trans[0]
#    p.position.y = trans[1]
#    p.position.z = trans[2]
#    p.orientation.x = rot[0]
#    p.orientation.y = rot[1]
#    p.orientation.z = rot[2]
#    p.orientation.w = rot[3]
#    return p
