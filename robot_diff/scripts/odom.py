#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf_conversions
import tf2_ros
import math

class FourWheelOdom:
    def __init__(self):
        # Constantes (identiques à votre code Arduino)
        self.WHEEL_BASE = 0.65  # m
        # ticks par mètre (Arduino: ticks/rev / (2πR))
        self.K_TICKS = 60.0 / (2.0 * math.pi * 0.084)

        # État pose
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = rospy.Time.now()

        # Vitesses reçues (ticks/s)
        self.vda = 0.0
        self.vdr = 0.0
        self.vga = 0.0
        self.vgr = 0.0

        # Souscriptions aux topics de vitesses de roue
        rospy.Subscriber('vel_droite_avant', Int16, self.cb_vel_da)
        rospy.Subscriber('vel_droite_arriere', Int16, self.cb_vel_dr)
        rospy.Subscriber('vel_gauche_avant', Int16, self.cb_vel_ga)
        rospy.Subscriber('vel_gauche_arriere', Int16, self.cb_vel_gr)

        # Publisher pour l'odométrie des roues
        self.odom_pub = rospy.Publisher('wheel_odom', Odometry, queue_size=10)

        # TF broadcaster pour odom -> base_link
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Timer à 50 Hz pour publication
        rospy.Timer(rospy.Duration(0.02), self.update)

    # Callbacks lecture des vitesses
    def cb_vel_da(self, msg): self.vda = msg.data
    def cb_vel_dr(self, msg): self.vdr = msg.data
    def cb_vel_ga(self, msg): self.vga = msg.data
    def cb_vel_gr(self, msg): self.vgr = msg.data

    # Mise à jour pose et publication
    def update(self, event):
        now = event.current_real
        dt = (now - self.last_time).to_sec()
        self.last_time = now
        if dt <= 0.0:
            return

        # Conversion ticks/s -> m/s
        vr = (self.vda + self.vdr) / (2.0 * self.K_TICKS)
        vl = (self.vga + self.vgr) / (2.0 * self.K_TICKS)

        # Calculs odométrie simple
        v = (vr + vl) / 2.0
        omega = (vr - vl) / self.WHEEL_BASE

        # Intégration Euler
        dx = v * math.cos(self.th) * dt
        dy = v * math.sin(self.th) * dt
        dth = omega * dt

        self.x += dx
        self.y += dy
        self.th += dth

        # Calcul quaternion
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.th)

        # Préparation du message Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*q)

        # Twist
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        # Covariances (mêmes dimensions que RTAB-Map)
        pose_cov = [0.0] * 36
        twist_cov = [0.0] * 36
        # exemple d'incertitude raisonnable
        pose_cov[0] = 0.01     # x
        pose_cov[7] = 0.01     # y
        pose_cov[35] = 0.01    # yaw
        twist_cov[0] = 0.01    # vx
        twist_cov[35] = 0.01   # v_yaw
        odom.pose.covariance = pose_cov
        odom.twist.covariance = twist_cov

        # Publication TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(*q)
        self.tf_broadcaster.sendTransform(t)

        # Publication odométrie sur /wheel_odom
        self.odom_pub.publish(odom)

if __name__ == '__main__':
    rospy.init_node('four_wheel_odom')
    FourWheelOdom()
    rospy.spin()