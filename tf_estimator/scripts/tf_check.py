#!/usr/bin/env python3
import rospy
import rosbag
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import nav_msgs.msg
import tf_conversions

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
from rigid_transform_3D import rigid_transform_3D


class Checker:
    tfBuffer = None
    common_frame = None
    msgs = (list(), list())
    last_closest = [0, 0]
    error_sum = 0
    diff_sum = np.array([0.0,0.0,0.0])
    n_msgs = 0
    max_dt = 0.5


    def to_np(self, point):
        return np.array((point.x, point.y, point.z))


    def find_closest_time(self, time, times, prev_it):
        if len(times) == 0:
            return None
        closest_it = 0
        closest_diff = abs(time - times[closest_it][1])
        for it in range(prev_it, len(times)):
            cur_diff = abs(time - times[it][1])
            if cur_diff <= closest_diff:
                closest_it = it
                closest_diff = cur_diff
            else: # assume time monotonicity
                break
        return closest_it


    def update(self, msg, stamp, id):
        idx = id-1
        other_idx = int(not idx)

        if len(self.msgs[idx]) > 0:
            if np.abs((stamp - self.msgs[idx][-1][1]).to_sec()) > 1.0:
                rospy.logwarn("Detected jump in time, resetting.")
                rospy.loginfo("Average diff:  {}m over {} total messages".format(self.diff_sum/self.n_msgs, self.n_msgs))
                rospy.loginfo("Average error: {}m over {} total messages".format(self.error_sum/self.n_msgs, self.n_msgs))
                self.msgs = (list(), list())
                last_closest = (0, 0)
                self.error_sum = 0
                self.diff_sum = np.array([0.0,0.0,0.0])
                self.n_msgs = 0

        self.msgs[idx].append((msg, stamp))
        cur_closest_idx = self.find_closest_time(stamp, self.msgs[other_idx], self.last_closest[idx])

        if cur_closest_idx is not None:
            closest_stamp = self.msgs[other_idx][cur_closest_idx][1]
            if np.abs((closest_stamp - stamp).to_sec()) <= self.max_dt:
                closest_point = self.msgs[other_idx][cur_closest_idx][0]
                diff = self.to_np(msg) - self.to_np(closest_point)
                dist = np.linalg.norm(diff)
                self.diff_sum += diff
                self.error_sum += dist
                self.n_msgs += 1
                print("Diff: {}m (average: {}m)".format(diff, self.diff_sum/self.n_msgs))
                print("Error: {:f}m (average: {:f}m)".format(dist, self.error_sum/self.n_msgs))
            self.last_closest[idx] = cur_closest_idx


    common_time_point1 = None
    common_time_point2 = None
    def callback1(self, msg):
        try:
            trans = self.tfBuffer.lookup_transform(self.common_frame, msg.header.frame_id, msg.header.stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Couldn't transform msg 1")
        tfd = tf2_geometry_msgs.do_transform_pose(msg.pose, trans).pose.position
        self.update(tfd, msg.header.stamp - self.common_time_point1, 1)


    def callback2(self, msg):
        try:
            trans = self.tfBuffer.lookup_transform(self.common_frame, msg.header.frame_id, msg.header.stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Couldn't transform msg 2")
        tfd = tf2_geometry_msgs.do_transform_pose(msg.pose, trans).pose.position
        self.update(tfd, msg.header.stamp - self.common_time_point2, 2)


    def main(self):
        rospy.init_node('tf_checker', anonymous=True)

        self.topic1_name = rospy.get_param('~topic1_name', "/uav7/drone_client/detection")
        self.topic2_name = rospy.get_param('~topic2_name', "/uav7/odometry/rtk_local_odom")

        self.common_time_point1 = rospy.Time(rospy.get_param('~common_time_point1', 1622034060.447095155+0.300))
        self.common_time_point2 = rospy.Time(rospy.get_param('~common_time_point2', 1622034067.216992855))

        self.common_frame = rospy.get_param('~common_frame', "a8v")

        self.plot = rospy.get_param('~plot', True)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        rospy.Subscriber(self.topic1_name, nav_msgs.msg.Odometry, self.callback1)
        rospy.Subscriber(self.topic2_name, nav_msgs.msg.Odometry, self.callback2)

        rospy.spin()


if __name__ == '__main__':
    ckr = Checker()
    ckr.main()

