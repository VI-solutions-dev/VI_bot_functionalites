#!/usr/bin/env python
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

        # ======= Params =======
        self.rate_hz     = rospy.get_param('~rate', 10.0)
        self.ticks_meter = float(rospy.get_param('~ticks_meter', 113.0))
        self.base_width  = float(rospy.get_param('~base_width', 0.60))
        self.base_frame  = rospy.get_param('~base_frame_id', 'base_link')
        self.odom_frame  = rospy.get_param('~odom_frame_id', 'wheel_odom')
        # NEW: control TF publication
        self.publish_tf  = rospy.get_param('~publish_tf', False)

        # Encoder wrap-around (16-bit)
        self.enc_min = int(rospy.get_param('~encoder_min', -32768))
        self.enc_max = int(rospy.get_param('~encoder_max',  32767))
        span = self.enc_max - self.enc_min
        self.low_wrap  = self.enc_min + 0.3 * span
        self.high_wrap = self.enc_min + 0.7 * span

        # Wheel state
        self.prev_enc_lf = self.prev_enc_lr = 0
        self.prev_enc_rf = self.prev_enc_rr = 0
        self.mult_lf = self.mult_lr = self.mult_rf = self.mult_rr = 0
        self.lf = self.lr = self.rf = self.rr = 0

        # Pose & velocity state
        self.x = self.y = self.th = 0.0
        self.dx = self.dr = 0.0

        # Publishers / TF
        self.odom_pub = rospy.Publisher('wheel/odom', Odometry, queue_size=10)
        #self.tf_broad = tf.TransformBroadcaster() if self.publish_tf else None

        # Subscribers
        rospy.Subscriber('ticks_gauche_avant',   Int16, self.cb_lf)
        rospy.Subscriber('ticks_gauche_arriere', Int16, self.cb_lr)
        rospy.Subscriber('ticks_droite_avant',   Int16, self.cb_rf)
        rospy.Subscriber('ticks_droite_arriere', Int16, self.cb_rr)

        self.last_time = rospy.Time.now()
        self.rate = rospy.Rate(self.rate_hz)

    # ======= Callbacks =======
    def cb_lf(self, msg):
        enc = msg.data
        if enc < self.low_wrap  and self.prev_enc_lf > self.high_wrap: self.mult_lf += 1
        if enc > self.high_wrap and self.prev_enc_lf < self.low_wrap:  self.mult_lf -= 1
        self.lf = enc + self.mult_lf * (self.enc_max - self.enc_min)
        self.prev_enc_lf = enc

    def cb_lr(self, msg):
        enc = msg.data
        if enc < self.low_wrap  and self.prev_enc_lr > self.high_wrap: self.mult_lr += 1
        if enc > self.high_wrap and self.prev_enc_lr < self.low_wrap:  self.mult_lr -= 1
        self.lr = enc + self.mult_lr * (self.enc_max - self.enc_min)
        self.prev_enc_lr = enc

    def cb_rf(self, msg):
        enc = msg.data
        if enc < self.low_wrap  and self.prev_enc_rf > self.high_wrap: self.mult_rf += 1
        if enc > self.high_wrap and self.prev_enc_rf < self.low_wrap:  self.mult_rf -= 1
        self.rf = enc + self.mult_rf * (self.enc_max - self.enc_min)
        self.prev_enc_rf = enc

    def cb_rr(self, msg):
        enc = msg.data
        if enc < self.low_wrap  and self.prev_enc_rr > self.high_wrap: self.mult_rr += 1
        if enc > self.high_wrap and self.prev_enc_rr < self.low_wrap:  self.mult_rr -= 1
        self.rr = enc + self.mult_rr * (self.enc_max - self.enc_min)
        self.prev_enc_rr = enc

    # ======= Main loop =======
    def spin(self):
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - self.last_time).to_sec()
            self.last_time = now

            # Deltas (handle first iteration)
            d_lf = self.lf - getattr(self, 'prev_lf', self.lf)
            d_lr = self.lr - getattr(self, 'prev_lr', self.lr)
            d_rf = self.rf - getattr(self, 'prev_rf', self.rf)
            d_rr = self.rr - getattr(self, 'prev_rr', self.rr)

            # Save previous totals
            self.prev_lf, self.prev_lr = self.lf, self.lr
            self.prev_rf, self.prev_rr = self.rf, self.rr

            # Average per side
            d_left_ticks  = 0.5 * (d_lf + d_lr)
            d_right_ticks = 0.5 * (d_rf + d_rr)

            # Convert to meters
            d_left   = d_left_ticks  / self.ticks_meter
            d_right  = d_right_ticks / self.ticks_meter
            d_center = 0.5 * (d_left + d_right)

            # Orientation change
            d_th = (d_right - d_left) / self.base_width

            # Integrate pose
            if d_center != 0.0:
                self.x += cos(self.th) * d_center
                self.y += sin(self.th) * d_center
            if d_th != 0.0:
                self.th += d_th

            # Velocities
            self.dx = d_center / dt if dt > 0.0 else 0.0
            self.dr = d_th     / dt if dt > 0.0 else 0.0

            # Quaternion
            quat = Quaternion(0, 0, sin(self.th * 0.5), cos(self.th * 0.5))

            # ---- TF (optional) ----
            if self.publish_tf and self.tf_broad is not None:
                self.tf_broad.sendTransform(
                    (self.x, self.y, 0.0),
                    (quat.x, quat.y, quat.z, quat.w),
                    now,
                    self.base_frame,
                    self.odom_frame
                )

            # ---- Odometry msg ----
            odom = Odometry()
            odom.header.stamp    = now
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id  = self.base_frame

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation = quat

            odom.twist.twist.linear.x  = self.dx
            odom.twist.twist.angular.z = self.dr

            # Simple covariance placeholder
            odom.pose.covariance = [0.0]*36
            odom.pose.covariance[0]  = 0.01
            odom.pose.covariance[7]  = 0.01
            odom.pose.covariance[14] = 0.1
            odom.pose.covariance[21] = 0.1
            odom.pose.covariance[28] = 0.1
            odom.pose.covariance[35] = 0.1

            self.odom_pub.publish(odom)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        DiffTf4Wheel().spin()
    except rospy.ROSInterruptException:
        pass
