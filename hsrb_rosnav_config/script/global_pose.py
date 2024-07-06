#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped

published_msg = PoseStamped()

def callback(data):
    global published_msg
    # Convert PoseWithCovarianceStamped to PoseStamped
    msg = PoseStamped()
    # Header
    msg.header = data.header
    msg.header.stamp = rospy.Time.now()
    # Pose
    msg.pose.position.x = data.pose.pose.position.x
    msg.pose.position.y = data.pose.pose.position.y
    msg.pose.position.z = data.pose.pose.position.z
    # Quaternion
    msg.pose.orientation.x = data.pose.pose.orientation.x
    msg.pose.orientation.y = data.pose.pose.orientation.y
    msg.pose.orientation.z = data.pose.pose.orientation.z
    msg.pose.orientation.w = data.pose.pose.orientation.w
    # Debug value
    rospy.logdebug(rospy.get_caller_id() + "output:  %s", msg)
    # Publish message
    published_msg = msg

if __name__ == '__main__':
    rospy.init_node('global_pose_publisher', anonymous=False)
    # Publisher for topic /global_pose
    pub = rospy.Publisher('global_pose', PoseStamped, queue_size=10)
    # Subscriber for topic /amcl_pose
    sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callback)
    rate = rospy.Rate(10) # 10Hz

    rospy.loginfo("waiting for initial msgs...")
    rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
    rospy.loginfo("got initial msgs. now spinning")

    while not rospy.is_shutdown():
        pub.publish(published_msg)
        rate.sleep()
