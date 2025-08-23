#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf_conversions
import tf2_ros
from message_filters import ApproximateTimeSynchronizer, Subscriber

def quaternion_to_yaw(q):
    return tf_conversions.transformations.euler_from_quaternion([
        q.x, q.y, q.z, q.w])[2]

def yaw_to_quaternion(yaw):
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)
    return Quaternion(*q)

class OdomFusion:
    def __init__(self):
        rospy.init_node('odom_fusion')
        # Fusion weight: 0 = all rtabmap, 1 = all wheels
        self.alpha = rospy.get_param('~alpha', 0.5)

        # Subscribers
        sub_wheel = Subscriber('/odom', Odometry)
        sub_rtab = Subscriber('/rtabmap/odom', Odometry)
        self.sync = ApproximateTimeSynchronizer([sub_wheel, sub_rtab], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.callback)

        # Publisher
        self.pub = rospy.Publisher('/fused_odom', Odometry, queue_size=10)
        # TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        rospy.loginfo("Odometry fusion node started with alpha=%.2f", self.alpha)

    def callback(self, wheel, rtab):
        # extract positions
        wx = wheel.pose.pose.position.x
        wy = wheel.pose.pose.position.y
        wyaw = quaternion_to_yaw(wheel.pose.pose.orientation)

        rx = rtab.pose.pose.position.x
        ry = rtab.pose.pose.position.y
        ryaw = quaternion_to_yaw(rtab.pose.pose.orientation)

        # fuse position
        fx = self.alpha * wx + (1-self.alpha) * rx
        fy = self.alpha * wy + (1-self.alpha) * ry

        # fuse yaw via circular mean
        sin_f = self.alpha * math.sin(wyaw) + (1-self.alpha) * math.sin(ryaw)
        cos_f = self.alpha * math.cos(wyaw) + (1-self.alpha) * math.cos(ryaw)
        fyaw = math.atan2(sin_f, cos_f)

        # fuse velocities
        fv = self.alpha * wheel.twist.twist.linear.x + (1-self.alpha) * rtab.twist.twist.linear.x
        fomega = self.alpha * wheel.twist.twist.angular.z + (1-self.alpha) * rtab.twist.twist.angular.z

        # prepare fused odometry
        fused = Odometry()
        fused.header.stamp = wheel.header.stamp if wheel.header.stamp > rtab.header.stamp else rtab.header.stamp
        fused.header.frame_id = 'odom'
        fused.child_frame_id = 'base_link'

        fused.pose.pose.position.x = fx
        fused.pose.pose.position.y = fy
        fused.pose.pose.position.z = 0.0
        fused.pose.pose.orientation = yaw_to_quaternion(fyaw)

        fused.twist.twist.linear.x = fv
        fused.twist.twist.angular.z = fomega

        # copy covariances (could be improved)
        fused.pose.covariance = wheel.pose.covariance
        fused.twist.covariance = wheel.twist.covariance

        # publish TF
        t = TransformStamped()
        t.header.stamp = fused.header.stamp
        t.header.frame_id = fused.header.frame_id
        t.child_frame_id = fused.child_frame_id
        t.transform.translation.x = fx
        t.transform.translation.y = fy
        t.transform.translation.z = 0.0
        t.transform.rotation = fused.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        # publish fused odom
        self.pub.publish(fused)

if __name__ == '__main__':
    try:
        fusion = OdomFusion()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass