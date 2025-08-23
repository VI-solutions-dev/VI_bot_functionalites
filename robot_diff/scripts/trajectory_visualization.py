#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

def callback(data):
    global xAnt, yAnt

    # Create the pose msg, it's necessary to do it each time because Python manages objects by reference
    pose = PoseStamped()
    
    # Set the attributes of the msg
    pose.header.frame_id = "odom"
    pose.pose.position.x = float(data.pose.pose.position.x)
    pose.pose.position.y = float(data.pose.pose.position.y)
    pose.pose.orientation.x = float(data.pose.pose.orientation.x)
    pose.pose.orientation.y = float(data.pose.pose.orientation.y)
    pose.pose.orientation.z = float(data.pose.pose.orientation.z)
    pose.pose.orientation.w = float(data.pose.pose.orientation.w)

    # To avoid repeating the values, ensure that the received values are different
    if xAnt != pose.pose.position.x or yAnt != pose.pose.position.y:
        pose.header.seq = path.header.seq + 1
        path.header.frame_id = "odom"
        path.header.stamp = rospy.Time.now()
        pose.header.stamp = path.header.stamp
        path.poses.append(pose)
        pub.publish(path)

    # Save the last position
    xAnt = pose.pose.position.x
    yAnt = pose.pose.position.y

if __name__ == '__main__':
    # Variable initialization
    global xAnt, yAnt
    xAnt = 0.0
    yAnt = 0.0

    # Node and msg initialization
    rospy.init_node('path_odom_plotter')
    nodename = rospy.get_name()
    rospy.loginfo("Nodo %s Iniciado" % nodename)
    pub = rospy.Publisher('/odompath', Path, queue_size=10)
    path = Path()
    msg = Odometry()
    # Subscription to the topic
    rospy.Subscriber('/odom', Odometry, callback)
    rate = rospy.Rate(30) # 30hz

    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
