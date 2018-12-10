#!/usr/bin/env python
import rospy, cv2, sys
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, Point
from select import select

class CollectorNode:
    def __init__(self):
        self.bbb = False
        rospy.Subscriber("/rtabmap/odom", Odometry, self.callback)

    def callback(self, data):
        if self.bbb:
            rospy.signal_shutdown("sad")
        else:
            x = data.pose.pose.position.x
            y = data.pose.pose.position.y
            z = data.pose.pose.position.z
            print "(" + str(x) + ", " + str(y) + ", " + str(z) + ")"
            qx = data.pose.pose.orientation.x
            qy = data.pose.pose.orientation.y
            # This is the one we want. 0 is for original, and 1 is 180 degrees from that.
            # clockwise is negative, and counterclockwise is positive.
            qz = data.pose.pose.orientation.z
            qw = data.pose.pose.orientation.w
            print "(" + str(qx) + ", " + str(qy) + ", " + str(qz) + ", " + str(qw) + ")"
            self.bbb = True
#     timeout = 1
    # print "Enter something:",
    # rlist, _, _ = select([sys.stdin], [], [], timeout)
    # if rlist:
    #     s = sys.stdin.readline()
    #     if s.strip() == "m":
    #         print data.pose.pose
    # else:
#         print "No input. Moving on..."


if __name__ == "__main__":
    node = CollectorNode()
    rospy.init_node("collector")
    rospy.spin()
