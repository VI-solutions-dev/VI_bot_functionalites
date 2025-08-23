#!/usr/bin/env python3
# diff_tf4.py  – 4-wheel odometry node (ticks → /wheel/odometry + TF)

import rospy
import tf
from math import sin, cos
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16


class DiffTf4Wheel:
    def __init__(self):
        rospy.init_node("odom_pub_4wheel")
        rospy.loginfo("Node %s started", rospy.get_name())

        # ---------------- parameters ----------------
        p = rospy.get_param
        self.rate_hz     = p('~rate',          10.0)
        self.ticks_meter = float(p('~ticks_meter', 113.0))
        self.base_width  = float(p('~base_width',   0.60))
        self.base_frame  = p('~base_frame_id', 'base_link')
        self.odom_frame  = p('~odom_frame_id', 'odom')

        enc_min = int(p('~encoder_min', -32768))
        enc_max = int(p('~encoder_max',  32767))
        span    = enc_max - enc_min
        self.low_wrap  = enc_min + 0.3 * span
        self.high_wrap = enc_min + 0.7 * span
        self.wrap_span = span

        # ---------------- wheel state --------------
        self.prev_enc = {'lf': 0, 'lr': 0, 'rf': 0, 'rr': 0}
        self.mult     = {'lf': 0, 'lr': 0, 'rf': 0, 'rr': 0}
        self.total    = {'lf': 0, 'lr': 0, 'rf': 0, 'rr': 0}

        # ---------------- pose state ---------------
        self.x = self.y = self.th = 0.0

        # ---------------- comms --------------------
        self.odom_pub = rospy.Publisher('wheel/odometry', Odometry, queue_size=10)
        self.tf_broad = tf.TransformBroadcaster()

        rospy.Subscriber('ticks_gauche_avant',   Int16, self.make_cb('lf'))
        rospy.Subscriber('ticks_gauche_arriere', Int16, self.make_cb('lr'))
        rospy.Subscriber('ticks_droite_avant',   Int16, self.make_cb('rf'))
        rospy.Subscriber('ticks_droite_arriere', Int16, self.make_cb('rr'))

        self.last_time = rospy.Time.now()
        self.rate = rospy.Rate(self.rate_hz)

    # ---------- encoder callbacks -----------------
    def make_cb(self, wheel):
        def _cb(msg):
            enc = msg.data
            if enc < self.low_wrap  and self.prev_enc[wheel] > self.high_wrap:
                self.mult[wheel] += 1
            if enc > self.high_wrap and self.prev_enc[wheel] < self.low_wrap:
                self.mult[wheel] -= 1
            self.total[wheel] = enc + self.mult[wheel]*self.wrap_span
            self.prev_enc[wheel] = enc
        return _cb

    # ---------- main loop -------------------------
    def spin(self):
        while not rospy.is_shutdown():
            self.update()
            self.rate.sleep()

    def update(self):
        now = rospy.Time.now()
        dt  = (now - self.last_time).to_sec()
        self.last_time = now
        if dt == 0:
            return

        # tick deltas since last cycle
        d_lf = self.total['lf'] - getattr(self, 'prev_lf', self.total['lf'])
        d_lr = self.total['lr'] - getattr(self, 'prev_lr', self.total['lr'])
        d_rf = self.total['rf'] - getattr(self, 'prev_rf', self.total['rf'])
        d_rr = self.total['rr'] - getattr(self, 'prev_rr', self.total['rr'])

        # store previous totals
        self.prev_lf, self.prev_lr = self.total['lf'], self.total['lr']
        self.prev_rf, self.prev_rr = self.total['rf'], self.total['rr']

        # average the two wheels per side
        d_left_ticks  = 0.5*(d_lf + d_lr)
        d_right_ticks = 0.5*(d_rf + d_rr)

        d_left  = d_left_ticks  / self.ticks_meter
        d_right = d_right_ticks / self.ticks_meter
        d_center = 0.5*(d_left + d_right)
        d_th     = (d_right - d_left) / self.base_width

        # integrate pose
        self.x  += cos(self.th) * d_center
        self.y  += sin(self.th) * d_center
        self.th += d_th

        # velocities
        vx  = d_center / dt
        vth = d_th     / dt

        # TF
        quat = Quaternion(0, 0, sin(self.th/2.0), cos(self.th/2.0))
        self.tf_broad.sendTransform(
            (self.x, self.y, 0.0),
            (quat.x, quat.y, quat.z, quat.w),
            now,
            self.base_frame,
            self.odom_frame
        )

        # odom message
        odom              = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.angular.z = vth

        # simple covariance (same as your original)
        for i in range(36):
            odom.pose.covariance[i] = 0.0
        odom.pose.covariance[0]  = odom.pose.covariance[7]  = 0.01
        odom.pose.covariance[14] = 0.1
        odom.pose.covariance[21] = odom.pose.covariance[28] = odom.pose.covariance[35] = 0.1

        self.odom_pub.publish(odom)


if __name__ == '__main__':
    try:
        DiffTf4Wheel().spin()
    except rospy.ROSInterruptException:
        pass
