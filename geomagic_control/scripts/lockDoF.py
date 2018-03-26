#!/usr/bin/env python
# THIS SCRIPT WAS JUST TO TEST STUFF AND SHOULD NOT  BE USED ANY MORE
import rospy
import math
from sensor_msgs.msg import JointState
from geomagic_control.msg import DeviceFeedback
import tf
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Header
def PS(origin,position=[0,0,0],orientation=[0,0,0,1]):
    """
        Creates a PoseStamped()
        
        :param origin: frame id
        :type: origin: str
        :param position: position (default [0,0,0])
        :type: position: list
        :param oriention: orientation (default [0,0,0,1])
        :type: orientation: list
        :return: the created PoseStamped()
        :rtype: PoseStamped
    """
    h=Header()
    h.frame_id=origin
    h.stamp=rospy.Time().now()
    
    p=Pose()
    if type(position) == Point:
        p.position = position
    else:
        p.position=Point(*position)
    if type(orientation) == Quaternion:
        p.orientation = orientation 
    elif len(orientation) == 4:
        p.orientation=Quaternion(*orientation)
    elif len(orientation) == 3:
        p.orientation = Quaternion(*tf.transformations.quaternion_from_euler(*orientation))
    else:
        p.orientation = Quaternion(0,0,0,1)
    return PoseStamped(h,p)
class GeomagicInterface:
    def __init__(self):
#         self.listener = tf.TransformListener()
        self.of = OmniFeedback()
        self.of.lock = [False for i in xrange(3)]
        self.pub = rospy.Publisher("Geomagic/force_feedback",OmniFeedback,queue_size=1)
        self.sub = rospy.Subscriber("Geomagic/end_effector_pose",JointState,self.callback, queue_size=1)
        self.lock = 0.0
    
    def callback(self,js):
#         x = js.position[0]
#         self.of.force.x = 0.04 *(self.lock - x) - 0.001 * js.velocity[0];
        self.of.position.x = js.position[0]
        self.of.position.y = js.position[1]
        self.of.position.z = js.position[2]
        self.of.lock = [False,True,True]
        self.pub.publish(self.of) 
#     def start(self):
#         while not rospy.is_shutdown():
#             x = self.getPose("tip", "base").pose.position.x
#             print self.of.force.x
#             self.pub.publish(self.of)
#             rospy.sleep(0.1)

#     def getPose(self,target,source,timeout=1):
#         now = rospy.Time.now()
#         end = rospy.Time.now()+rospy.Duration(timeout)
#         while not rospy.is_shutdown() and now < end:
#             now = rospy.Time.now()
#             try:
#                 (trans,rot) = self.listener.lookupTransform(source,target, rospy.Time(0))
#                 return PS(source,trans,rot)
#             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#                 rospy.sleep(0.01)
#                 continue
#         raise Exception,"Transform %s -> %s never appeared"%(target,source)
#         return None
        

if __name__ == '__main__':
    
    rospy.init_node("geomagic_touch_dof_locker")
    gi = GeomagicInterface()
    
    
    


    rospy.spin()
