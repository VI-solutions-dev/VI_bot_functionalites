#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Point-to-point controller for a differential robot
* Uses yaw from /imu/data when available (falls back to /odom)
* Phase A: rotate toward goal  (|yaw_err| > ang_tol)   — lin = 0
* Phase B: drive and steer     (dist  > xy_tol)        — lin > 0
"""

import math, sys, rospy, tf
from geometry_msgs.msg import Twist, Pose2D, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# ───────────────────────────────────────────────────────── helpers ──
def shortest_ang_diff(a, b):
    """Shortest signed angular distance b–a  ([-π, π])."""
    return (b - a + math.pi) % (2 * math.pi) - math.pi

def sat(val, limit):
    return max(-limit, min(limit, val))

# ───────────────────────────────────────────────────────── Control ──
class ControlBot:
    def __init__(self):
        rospy.init_node("controlador_de_robot")
        p = rospy.get_param

        # tunables
        self.k_lin  = p("~k_linear",   1.2)
        self.k_ang  = p("~k_angular",  3.0)
        self.v_max  = p("~v_max",      0.4)
        self.w_max  = p("~w_max",      1.5)
        self.xy_tol = p("~xy_tolerance", 0.20)
        self.ang_tol= p("~ang_tolerance", 0.10)   # rad ≈ 6 deg

        # comms
        self.cmd_pub  = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/odom", Odometry,      self.cb_odom, queue_size=1)
        rospy.Subscriber("/imu/data", Imu,       self.cb_imu,  queue_size=1)

        # state
        self.pose = Pose2D()
        self.use_imu = False            # becomes True once the first IMU msg arrives
        self.rate = rospy.Rate(15)
        rospy.on_shutdown(self.stop_robot)

    # ───────── callbacks
    def cb_imu(self, msg: Imu):
        # extract yaw from IMU quaternion (faster than odom)
        q = msg.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose.theta = yaw
        self.use_imu = True             # flag that yaw now comes from IMU

    def cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose.x, self.pose.y = p.x, p.y
        if not self.use_imu:            # only use odom yaw *until* IMU arrives
            self.pose.theta = yaw

    # ───────── main motion routine
    def move_to(self, goal: Pose2D):
        tw = Twist()

        # ---------- Phase A  : rotate in place ----------
        while (not rospy.is_shutdown() and
               abs(shortest_ang_diff(self.pose.theta,
                                      math.atan2(goal.y - self.pose.y,
                                                 goal.x - self.pose.x))) > self.ang_tol):
            yaw_err         = shortest_ang_diff(self.pose.theta,
                                                math.atan2(goal.y - self.pose.y,
                                                           goal.x - self.pose.x))
            tw.linear.x     = 0.0
            tw.angular.z    = sat(self.k_ang * yaw_err, self.w_max)
            self.cmd_pub.publish(tw)
            self.rate.sleep()

        # ---------- Phase B  : drive toward goal ----------
        while (not rospy.is_shutdown() and
               math.hypot(goal.x - self.pose.x,
                          goal.y - self.pose.y) > self.xy_tol):

            dx, dy   = goal.x - self.pose.x, goal.y - self.pose.y
            rho      = math.hypot(dx, dy)
            goal_yaw = math.atan2(dy, dx)
            yaw_err  = shortest_ang_diff(self.pose.theta, goal_yaw)

            tw.linear.x  = sat(self.k_lin * rho,     self.v_max)
            tw.angular.z = sat(self.k_ang * yaw_err, self.w_max)
            self.cmd_pub.publish(tw)
            self.rate.sleep()

        self.stop_robot()     # reached

    # ───────── helpers
    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    # ───────── CLI loop
    def run(self):
        while not rospy.is_shutdown():
            try:
                user = input("New target (x,y) or 'q' → ").strip()
            except EOFError:
                break
            if user.lower() in ("q", "quit", "exit"):
                break
            try:
                x_s, y_s = user.split(",")
                goal = Pose2D(float(x_s), float(y_s), 0.0)
            except ValueError:
                print("✗ format.  Example: 1.0,2.5")
                continue

            rospy.loginfo("➡️  Goal (%.2f, %.2f)", goal.x, goal.y)
            self.move_to(goal)

        rospy.signal_shutdown("Finished")

# ────────────────────────────────────────────────────────────── main
if __name__ == "__main__":
    try:
        ControlBot().run()
    except rospy.ROSInterruptException:
        pass
