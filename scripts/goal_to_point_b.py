#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Minimal point-to-point controller for a differential robot.
 • Tested on ROS 1 (Melodic/Noetic) with Python 3
 • Subscribes to /odom and publishes /cmd_vel
 • Prompts the user for successive (x, y) goals in the odom frame
"""

import math, sys, rospy, tf
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry

# ─────────────────────────────────────────────────────────────────────────────
# Helper functions
# ─────────────────────────────────────────────────────────────────────────────
def shortest_ang_diff(a, b):
    """Shortest signed angular distance b-a (rad, in [-π, π])."""
    return (b - a + math.pi) % (2 * math.pi) - math.pi


def sat(value, limit):
    """Clip *value* to ±*limit*."""
    return max(-limit, min(limit, value))

# ─────────────────────────────────────────────────────────────────────────────
class ControlBot:
    def __init__(self):
        rospy.init_node("controlador_de_robot")            # node name left as-is

        # Tunable gains & limits ------------------------------------------------
        self.k_lin   = rospy.get_param("~k_linear", 1.2)   # (m s-1) per metre error
        self.k_ang   = rospy.get_param("~k_angular", 3.0)  # (rad s-1) per rad error
        self.v_max   = rospy.get_param("~v_max", 0.4)      # m s-1
        self.w_max   = rospy.get_param("~w_max", 1.5)      # rad s-1
        self.xy_tol  = rospy.get_param("~xy_tolerance", 0.20)  # m

        # Communications --------------------------------------------------------
        self.cmd_pub  = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry,
                                         self.odom_cb, queue_size=1)

        # Internal state --------------------------------------------------------
        self.pose = Pose2D()      # current robot pose
        self.rate = rospy.Rate(15)

        rospy.on_shutdown(self.stop_robot)

    # ─────────────────────────────────── Callbacks
    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])

        # Keep full precision – do not round.
        self.pose.x, self.pose.y, self.pose.theta = p.x, p.y, yaw

    # ─────────────────────────────────── Motion
    def move_to(self, goal: Pose2D):
        tw = Twist()

        # Position-plus-bearing phase
        while (not rospy.is_shutdown()
               and math.hypot(goal.x - self.pose.x,
                              goal.y - self.pose.y) > self.xy_tol):

            # Positional and bearing errors
            dx, dy   = goal.x - self.pose.x, goal.y - self.pose.y
            rho      = math.hypot(dx, dy)
            goal_yaw = math.atan2(dy, dx)
            yaw_err  = shortest_ang_diff(self.pose.theta, goal_yaw)

            # Simple proportional control
            tw.linear.x  = sat(self.k_lin * rho,     self.v_max)
            tw.angular.z = sat(self.k_ang * yaw_err, self.w_max)

            self.cmd_pub.publish(tw)
            self.rate.sleep()

        # XY tolerance reached – stop immediately
        self.stop_robot()

    # ─────────────────────────────────── Helpers
    def stop_robot(self):
        self.cmd_pub.publish(Twist())   # publish zero velocities

    # ─────────────────────────────────── Interactive loop
    def run(self):
        while not rospy.is_shutdown():
            try:
                user = input("New target (x,y) or 'q' to quit → ").strip()
            except EOFError:            # e.g. Ctrl-D
                break

            if user.lower() in ("q", "quit", "exit"):
                break

            try:
                x_s, y_s = user.split(",")
                goal = Pose2D(float(x_s), float(y_s), 0.0)   # final heading ignored
            except ValueError:
                print("✗ Incorrect format. Example: 1.0, 2.5")
                continue

            rospy.loginfo("➡️  Goal received: (%.2f, %.2f)", goal.x, goal.y)
            self.move_to(goal)

        rospy.signal_shutdown("Program finished")

# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    try:
        ControlBot().run()
    except rospy.ROSInterruptException:
        pass
