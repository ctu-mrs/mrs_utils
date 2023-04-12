#!/usr/bin/python3

# #{ imports

import rospy
import rosnode
import os
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms
import mpl_toolkits.mplot3d

# #} end of imports

# #{ class Vector4d

class Vector4d:
    def __init__(self, x, y, z, heading):
        self.x = x
        self.y = y
        self.z = z
        self.heading = heading

# #} end of class Vector4d

# #{ class Trajectory

class Trajectory:
    def __init__(self, waypoint_list, trajectory_name, delimiter):
        self.poses = waypoint_list
        self.trajectory_name = trajectory_name
        self.delimiter = delimiter

# #} end of class Trajectory 

# #{ class TrajectoryTransformer

class TrajectoryTransformer:

    # #{ __init__()

    def __init__(self):

        rospy.init_node('trajectory_checker', anonymous=True)

        trajectory_folder_in = rospy.get_param('~trajectory_folder_in')
        trajectory_folder_out = rospy.get_param('~trajectory_folder_out')
        trajectory_files = rospy.get_param('~trajectory_files')
        process_all = rospy.get_param('~process_all')
        translation_x = rospy.get_param('~transformation/translation/x')
        translation_y = rospy.get_param('~transformation/translation/y')
        translation_z = rospy.get_param('~transformation/translation/z')
        rotation_heading = rospy.get_param('~transformation/rotation/heading')
        rotation_origin_x = rospy.get_param('~transformation/rotation/origin/x')
        rotation_origin_y = rospy.get_param('~transformation/rotation/origin/y')

        if not os.path.exists(trajectory_folder_out):
            rospy.logerr('Output trajectory folder does not exist. Transformation process aborted.')
            return

        if not os.path.exists(trajectory_folder_in):
            rospy.logerr('Input trajectory folder does not exist. Transformation process aborted.')
            return

        if trajectory_folder_in == trajectory_folder_out:
            rospy.logwarn('The folder with trajectories is the same as output folder. The trajectories will be overwritten. Do you want to continue? (y/n)')
            char_in = input()
            if char_in !=  "y" and char_in != "Y":
                rospy.loginfo('Transformation process was aborted.')
                return

        if process_all:
            trajectory_files = [f for f in os.listdir(trajectory_folder_in) if os.path.isfile(os.path.join(trajectory_folder_in, f))]

        translation = [translation_x, translation_y, translation_z]
        origin_of_rotation = [rotation_origin_x, rotation_origin_y]

        if len(trajectory_files) > 0: 
            self.file_extension = "." + trajectory_files[0].split(".")[-1]

        input_trajectories = self.loadTrajectories(trajectory_folder_in, trajectory_files)

        if len(input_trajectories) == len(trajectory_files):
            rospy.loginfo("[TrajectoryTransformer] All trajectories loaded successfully. Number of trajectories = %lu.", len(input_trajectories))
        else:
            rospy.logwarn("[TrajectoryTransformer] Only %lu out of %lu were loaded.", len(input_trajectories), len(trajectory_files))

        if len(input_trajectories) == 0:
            rospy.logwarn("[TrajectoryTransformer] No valid trajectory loaded. Nothing to check.")
            return

        transformed_trajectories = []
        for trajectory in input_trajectories:
            transformed_trajectories.append(self.transformTrajectory(trajectory, translation, rotation_heading, origin_of_rotation))

        self.saveTrajectories(trajectory_folder_out, transformed_trajectories)

        self.plotTrajectories(input_trajectories, transformed_trajectories)


    # #} end of __init__()

    def transformTrajectory(self, trajectory, translation, rotation_heading, origin_of_rotation):
        waypoints_t = []
        for w in trajectory.poses:
            xo = (w.x - origin_of_rotation[0])
            yo = (w.y - origin_of_rotation[1])
            x = xo * np.cos(rotation_heading) - yo * np.sin(rotation_heading) + origin_of_rotation[0] + translation[0]
            y = xo * np.sin(rotation_heading) + yo * np.cos(rotation_heading) + origin_of_rotation[1] + translation[1]
            z = w.z + translation[2]
            heading = w.heading + rotation_heading
            waypoints_t.append(Vector4d(x, y, z, heading))
        
        return Trajectory(waypoints_t, trajectory.trajectory_name, trajectory.delimiter)
          
    # #{ saveTrajectories()

    def saveTrajectories(self, folder, trajectories):
        rospy.loginfo('Start trajectory saving process.')
        for trajectory in trajectories:
            filename = trajectory.trajectory_name + self.file_extension 
            if os.path.exists(folder):
                self.writeTrajectoryToFile(os.path.join(folder, filename), trajectory)   
            else:
                rospy.logerr('[TrajectoryTransformer] Output trajectory folder %s not found. Excluding file %s from saving.', folder, os.path.join(folder, filename))

        return trajectories

    # #} end of saveTrajectories()
    
    # #{ writeTrajectoryToFile()

    def writeTrajectoryToFile(self, file_path, trajectory):
        rospy.loginfo('Saving transformed trajectory %s to file %s.', trajectory.trajectory_name, file_path)
        with open(file_path, 'w') as f:
            for w in trajectory.poses:
                f.write(str(w.x) + trajectory.delimiter + str(w.y) + trajectory.delimiter + str(w.z) + trajectory.delimiter + str(w.heading) + "\n")

    # #} end of writeTrajectoryToFile()

    # #{ loadTrajectories()

    def loadTrajectories(self, folder, filenames):

        trajectories = []
        for filename in filenames:
            if os.path.exists(os.path.join(folder, filename)):
                trajectories.append(self.loadTrajectory(os.path.join(folder, filename), filename.rpartition('.')[0]))
            else:
                rospy.logerr('[TrajectoryTransformer] Trajectory file %s not found. Excluding file from checking.', os.path.join(folder, filename))

        return trajectories

    # #} end of loadTrajectories()

    # #{ loadTrajectory()

    def loadTrajectory(self, filepath, trajectory_name):
        f = open(filepath, 'r')
        lines = f.readlines()
        waypoints = []
        ext = filepath.split(".")[-1]
        for line in lines:
            x, y, z, heading = line.split(',' if ',' in line else ' ')
            waypoints.append(Vector4d(float(x), float(y), float(z), float(heading)))

        delimiter = ' '
        if ',' in lines[0]: 
            delimiter = ','

        return Trajectory(waypoints, trajectory_name, delimiter)

    # #} end of loadTrajectory()

    # #{ plotPaths()
    def plotTrajectories(self, trajectories, transformed_trajectories):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.set_title("Trajectories")

        for trajectory in trajectories:
            xs = [pose.x for pose in trajectory.poses]
            ys = [pose.y for pose in trajectory.poses]
            zs = [pose.z for pose in trajectory.poses]
            ax.plot(xs, ys, zs, linestyle='dotted')
            idx_middle = math.floor(len(xs)*0.05)
            ax.scatter([xs[0], xs[idx_middle]], [ys[0], ys[idx_middle]], [zs[0], zs[idx_middle]], marker='o')
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
            ax.set_zlabel("z (m)")

        for trajectory in transformed_trajectories:
            xs = [pose.x for pose in trajectory.poses]
            ys = [pose.y for pose in trajectory.poses]
            zs = [pose.z for pose in trajectory.poses]
            ax.plot(xs, ys, zs)
            idx_middle = math.floor(len(xs)*0.05)
            ax.scatter([xs[0], xs[idx_middle]], [ys[0], ys[idx_middle]], [zs[0], zs[idx_middle]], marker='o')
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
            ax.set_zlabel("z (m)")

        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        plt.show()

    # #} end of plotPaths()

# #} end of TRAJECTORY TRANSFORMER

if __name__ == '__main__':
    try:
        trajectory_transformer = TrajectoryTransformer()
    except rospy.ROSInterruptException:
        pass
