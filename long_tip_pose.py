import roslib
roslib.load_manifest("phantom_omni")
import rospy
import tf
import math

from geometry_msgs.msg import PoseStamped
import pdb

rospy.init_node("long_tip_pose_publisher")
pub = rospy.Publisher('pr2_right', PoseStamped)
r = rospy.Rate(60)

while not rospy.is_shutdown():
    ps = PoseStamped()
    ps.header.frame_id = 'omni1_link6'
    ps.header.stamp = rospy.get_rostime()
    ps.pose.position.x = -.134 
    ps.pose.position.y = 0
    ps.pose.position.z = 0
    q = tf.transformations.quaternion_from_euler(0, math.pi, 0)
    ps.pose.orientation.x = q[0]
    ps.pose.orientation.y = q[1]
    ps.pose.orientation.z = q[2]
    ps.pose.orientation.w = q[3]
    pub.publish(ps)
    r.sleep()



