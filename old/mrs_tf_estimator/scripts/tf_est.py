#!/usr/bin/env python3
import rospy
import rosbag
import tf2_ros
import geometry_msgs.msg
import tf_conversions

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
from rigid_transform_3D import rigid_transform_3D


def load_rosbag_msgs(bag_fname, topic, skip_time=0, skip_time_end=0):
    rospy.loginfo("Using rosbag \"{:s}\"".format(bag_fname))
    if not os.path.isfile(bag_fname):
        rospy.logerr("File \"{:s}\" doesn't exist".format(bag_fname))
        return None
    bag = rosbag.Bag(bag_fname)
    n_msgs = bag.get_message_count(topic_filters=topic)
    if n_msgs == 0:
        rospy.logerr("No messages from topic {:s} in bag".format(topic))
    else:
        rospy.loginfo("Loading {:d} messages".format(n_msgs))
    msgs = n_msgs*[None]

    skip = rospy.Duration.from_sec(skip_time)
    start_time = rospy.Time.from_sec(bag.get_start_time()) + skip
    skip_end = rospy.Duration.from_sec(skip_time_end)
    end_time = rospy.Time.from_sec(bag.get_end_time()) - skip_end
    it = 0
    for topic, msg, cur_stamp in bag.read_messages(topics=topic, start_time=start_time, end_time=end_time):
        if rospy.is_shutdown():
            break
        msgs[it] = msg
        it += 1
    return msgs[0:it]


def msgs_to_pos_time(msgs):
    it = 0
    ret = len(msgs)*[None]
    times = len(msgs)*[None]
    for msg in msgs:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        t = msg.header.stamp
        xyz = np.array([x, y, z])
        ret[it] = xyz
        times[it] = t.to_sec()
        it += 1
    return np.matrix(ret).transpose(), np.array(times)


def apply_transform(poss, R, t):
    return R*poss + t


def prealign_poss(poss1, poss2):
    R, t = rigid_transform_3D(poss1, poss2)
    poss1 = apply_transform(poss1, R, t)
    return poss1, poss2


def find_common_time(ts1, ts2):
    common_start_t = np.max((ts1[0], ts2[0]))
    common_end_t = np.min((ts1[-1], ts2[-1]))
    return (common_start_t, common_end_t)


def time_clamp(poss, ts, t_start, t_end):
    idcs = np.logical_and(ts >= t_start, ts <= t_end)
    poss = poss[:, idcs]
    print("orig t from {}s to {}s".format(ts[0], ts[-1]))
    ts = ts[idcs]
    print("new t from {}s to {}s".format(ts[0], ts[-1]))
    return (poss, ts)


def find_closest_time(time, times, prev_it):
    closest_it = 0
    closest_diff = abs(time - times[closest_it])
    for it in range(prev_it, len(times)):
        cur_diff = abs(time - times[it])
        if cur_diff <= closest_diff:
            closest_it = it
            closest_diff = cur_diff
        else: # assume time monotonicity
            break
    return closest_it

        
def time_align(times1, positions2, times2):
    max_dt = 0.5
    positions_time_aligned = np.matrix(np.zeros((3, len(times1))))
    prev_it = 0
    for it in range(0, len(times1)):
        time1 = times1[it]
        closest_it = find_closest_time(time1, times2, prev_it)
        prev_it = closest_it
        if abs(times2[closest_it] - time1) > max_dt:
            positions_time_aligned[:, it] = np.array([None, None, None])
        else:
            positions_time_aligned[:, it] = positions2[:, closest_it]
    return positions_time_aligned, times1


def calc_errors(poss1, poss2):
    errors = np.linalg.norm(poss1 - poss2, axis=0)
    return errors


def calc_avg_error(poss1, poss2, FP_error=np.inf):
    errors = calc_errors(poss1, poss2)
    tot_error = 0
    N = 0
    for err in errors:
        if err is not None and err < FP_error:
            tot_error += err
            N += 1
    return tot_error/float(N)


def iterative_align(positions, to_positions, R_init, t_init, max_its=50, max_err=0.00005):
    FP_error = 0.5
    transf_tot = (R_init, t_init)
    for it in range(0, max_its):
        if rospy.is_shutdown():
            break
        print("align iteration {:d}:".format(it))
        cur_poss = apply_transform(positions, transf_tot[0], transf_tot[1])
        errors = calc_errors(cur_poss, to_positions)
        idcs = errors < FP_error
        N = np.sum(idcs, axis=0)
        E0 = np.mean(errors[idcs])/N
        print("avg. error: {:f} (from {} points)".format(E0, N))
        if E0 < max_err:
            break
        R, t = rigid_transform_3D(cur_poss[:, idcs], to_positions[:, idcs])
        transf_tot = (R*transf_tot[0], R*transf_tot[1] + t)
    return transf_tot


def main():
    rospy.init_node('tf_estimator', anonymous=True)

    bag_fname = rospy.get_param('~bag_name', "/home/matous/bag_files/eagle/tf_est/_2021-05-26-17-00-58.bag")
    skip_time_start = rospy.get_param('~skip_time_start', 0)
    skip_time_end = rospy.get_param('~skip_time_end', 0)

    topic1_name = rospy.get_param('~topic1_name', "/uav7/drone_client/detection")
    topic2_name = rospy.get_param('~topic2_name', "/uav7/odometry/rtk_local_odom")

    common_time_point1 = rospy.get_param('~common_time_point1', 1622034060.447095155+0.300)
    common_time_point2 = rospy.get_param('~common_time_point2', 1622034067.216992855)

    invert_tf = rospy.get_param('~invert_tf', False)
    plot = rospy.get_param('~plot', True)

    rospy.loginfo("Loading data from rosbags {:s}".format(bag_fname))

    loc_msgs1 = load_rosbag_msgs(bag_fname, topic1_name, skip_time=skip_time_start, skip_time_end=skip_time_end)
    loc_msgs2 = load_rosbag_msgs(bag_fname, topic2_name, skip_time=skip_time_start, skip_time_end=skip_time_end)

    if loc_msgs1 is None or loc_msgs2 is None:
        exit(1)

    frame_from = loc_msgs1[0].header.frame_id
    frame_to = loc_msgs2[0].header.frame_id

    rospy.loginfo("Loaded {:d} localization messages from source 1".format(len(loc_msgs1)))
    rospy.loginfo("Loaded {:d} localization messages from source 2".format(len(loc_msgs2)))

    [poss1, ts1] = msgs_to_pos_time(loc_msgs1)
    [poss2, ts2] = msgs_to_pos_time(loc_msgs2)

    time_compensation = common_time_point2 - common_time_point1
    print("adding {}s compensation to time of first trajectory".format(time_compensation))
    ts1 += time_compensation

    print("len1: {}".format(poss1.shape[1]))
    print("len2: {}".format(poss2.shape[1]))
    t_start, t_end = find_common_time(ts1, ts2)
    poss1, ts1 = time_clamp(poss1, ts1, t_start, t_end)
    poss2, ts2 = time_clamp(poss2, ts2, t_start, t_end)
    ts1 -= t_start
    ts2 -= t_start

    print("len1 (time-clamped): {}".format(poss1.shape[0]))
    print("len2 (time-clamped): {}".format(poss2.shape[0]))
    # poss2 = poss2 - poss2[-1, :] + poss1[-1, :]
    poss2, ts2 = time_align(ts1, poss2, ts2)
    print("len1 (aligned): {}".format(poss1.shape[0]))
    print("len2 (aligned): {}".format(poss2.shape[0]))
    R_init, t_init = rigid_transform_3D(poss1, poss2)
    init_err = calc_avg_error(apply_transform(poss1, R_init, t_init), poss2)
    rospy.loginfo("Done pre-processing positions")

    R, t = iterative_align(poss1, poss2, R_init, t_init, max_its = 1000)

    print("Initial transformation between positions:")
    print("R: {}".format(R_init))
    print("t: {}".format(t_init))

    print("Corrected transformation between positions:")
    print("R: {}".format(R))
    print("t: {}".format(t))
    poss1 = apply_transform(poss1, R, t)
    final_err = calc_avg_error(poss1, poss2)

    print("Error after initial alignment: {}m".format(init_err))
    print("Error after final alignment:   {}m".format(final_err))
    print("Error decrease:                {}m".format(init_err - final_err))

    if plot:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        x1 = np.array(poss1[0, :]).flatten()
        y1 = np.array(poss1[1, :]).flatten()
        z1 = np.array(poss1[2, :]).flatten()
        ax.plot(x1, y1, z1, 'g.')

        x2 = np.array(poss2[0, :]).flatten()
        y2 = np.array(poss2[1, :]).flatten()
        z2 = np.array(poss2[2, :]).flatten()
        ax.plot(x2, y2, z2, 'b.')

        plt.legend((topic1_name, topic2_name))

        ax.plot([poss1[0, 0]], [poss1[1, 0]], [poss1[2, 0]], 'gx', ms=10)
        ax.plot([poss1[0, -1]], [poss1[1, -1]], [poss1[2, -1]], 'go', ms=10)

        ax.plot([poss2[0, 0]], [poss2[1, 0]], [poss2[2, 0]], 'bx', ms=10)
        ax.plot([poss2[0, -1]], [poss2[1, -1]], [poss2[2, -1]], 'bo', ms=10)

        prev_it = 0
        for it in range(0, ts1.shape[0]):
            if it % 50 == 0:
                t1 = ts1[it]
                closest_it = find_closest_time(t1, ts2, prev_it)
                prev_it = closest_it
                ax.plot([poss1[0, it]], [poss1[1, it]], [poss1[2, it]], 'rx', ms=10)
                ax.plot([poss2[0, it]], [poss2[1, it]], [poss2[2, it]], 'rx', ms=10)
                ax.plot([poss1[0, it], poss2[0, it]], [poss1[1, it], poss2[1, it]], [poss1[2, it], poss2[2, it]], 'y')

        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_zlabel('z (m)')
        plt.show()

    # invert the transformation if required
    if invert_tf:
        print("Inverting tranformation!")
        R = R.transpose()
        t = -R*t
        frame_to, frame_from = frame_from, frame_to
        print("R: {}".format(R))
        print("t: {}".format(t))

    # convert t to a plain 1D array for easier manipulation
    t = np.array(t).flatten()
    # convert R to homogeneous coordinates so that ROS TF can understand it
    tmp = np.identity(4)
    tmp[:3, :3] = R
    R = tmp

    q = tf_conversions.transformations.quaternion_from_matrix(R)
    print("----- TO PUBLISH THIS TRANSFORM, ISSUE THE FOLLOWING COMMAND -----")
    print("rosrun tf2_ros static_transform_publisher {:f} {:f} {:f} {:f} {:f} {:f} {:f} {:s} {:s}".format(t[0], t[1], t[2], q[0], q[1], q[2], q[3], frame_to, frame_from))


if __name__ == '__main__':
    main()
