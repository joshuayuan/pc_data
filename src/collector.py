#!/usr/bin/env python
import rospy, cv2
from pypcd import pypcd
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, Point

class CollectorNode:
    def __init__(self):
        self.bbb = False
        self.aaa = False
        self.path = "/home/aaron/catkin_ws/src/pc_data/data/"
        rospy.Subscriber("/rtabmap/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/rtabmap/cloud_map", PointCloud2, self.pc_cb)

    def pc_cb(self, point_cloud):
        if self.aaa and self.bbb:
            rospy.signal_shutdown("shutdown from pc_cb")
        else:
            pc = pypcd.PointCloud.from_msg(point_cloud)
            pc.save(self.path + "final_cloud.pcd")
            print "saved point cloud"
            self.aaa = True

    def odom_cb(self, data):
        if self.bbb and self.aaa:
            rospy.signal_shutdown("shutdown from odom_cb")
        else:
            f = open(self.path + "final_position.txt", "w")
            x = data.pose.pose.position.x
            y = data.pose.pose.position.y
            z = data.pose.pose.position.z
            f.write(str(x) + "\n")
            f.write(str(y) + "\n")
            f.write(str(z) + "\n")
            print "(" + str(x) + ", " + str(y) + ", " + str(z) + ")"
            qx = data.pose.pose.orientation.x
            qy = data.pose.pose.orientation.y
            # This is the one we want. 0 is for original, and 1 is 180 degrees from that.
            # clockwise is negative, and counterclockwise is positive.
            qz = data.pose.pose.orientation.z
            qw = data.pose.pose.orientation.w
            f.write(str(qx) + "\n")
            print "(" + str(qx) + ", " + str(qy) + ", " + str(qz) + ", " + str(qw) + ")"
            f.close()
            self.bbb = True

if __name__ == "__main__":
    node = CollectorNode()
    rospy.init_node("collector")
    rospy.spin()
