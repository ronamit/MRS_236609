#!/usr/bin/env python

'''
run this in another terminal:
$ roscore


run this in another terminal:
$ roscore


1. roslaunch MRS_236609 turtlebot3_closed_room_with_2_spheres.launch
2. roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/my_ws/src/MRS_236609/maps/closed_room.yaml
3. rosrun MRS_236609 assignment2.py


'''
#!/usr/bin/env python

import rospy
import numpy as np
import argparse

from geometry_msgs.msg import PoseWithCovarianceStamped
import dynamic_reconfigure.client
import time


from nav_msgs.msg import Odometry, OccucpancyGrid
from map_msgs.msg import OccupancyGridUpdate
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
import random
import matplotlib.pyplot as plt


def euler_from_quaternion(quaternion):
    q0, q1, q2, q3 = quaternion
    roll = np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 ** 2 + q2 ** 2))
    pitch = np.arcsin(2 * (q0 * q2 - q3 * q1))
    yaw = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 ** 2 + q3 ** 2))
    return roll, pitch, yaw


class Pose:
    x = None
    y = None
    yaw = None

    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0

    def update(self, data):
        self.x, self.y = data.pose.pose.position.x, data.pose.pose.position.y
        (_, _, self.yaw) = euler_from_quaternion(
            [data.pose.pose.orientation.x, data.pose.pose.orientation.z, data.pose.pose.orientation.z,
             data.pose.pose.orientation.w])


class Map:
    map = None
    shape = None

    def __init__(self):
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.init_map)
        rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, self.update)

    def init_map(self, msg):
        print('Initialising map...')
        self.shape = msg.info.height, msg.info.width
        self.map = np.array(msg.data).reshape(self.shape)

    def update(self, msg):
        shape = msg.height, msg.width
        new_data = np.array(msg.data).reshape(shape)
        self.map[msg.y:msg.y + shape[0], msg.x: msg.x + shape[1]] = new_data

    def show(self):
        plt.imshow(self.map)
        plt.show()


def server_unavailable(start_time):
    if time.time() - start_time > 100:
        rospy.logerr("Action server is unavailable")
        rospy.signal_shutdown("Action server is unavailable")
        return True
    return False


# save the turtlebot's place as global
pose = Pose()


class TurtleBot:
    map = Map()
    initial_position = None
    action_client = None

    def __init__(self):
        # subscribes pose update to messages of type odometry -> pose holds the updated pose of the robot
        sub = rospy.Subscriber("odom", Odometry, pose.update)

        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, callback=self.init_position)

        print("Waiting for an initial position...")
        while self.initial_position is None:
            continue
        print("The initial position is {}".format(self.initial_position))

    def init_position(self, msg):
        initial_pose = msg.pose.pose
        self.initial_position = np.array([initial_pose.position.x, initial_pose.position.y])

    def add_goal(self, x, y, yaw=1, text=""):
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = yaw

        self.action_client.send_goal(goal)

        print("new" + text + "goal command")

    def move(self, point, threshold, start_time):
        global pose
        while np.lingal.norm(np.array(point[0] - pose.x, point[1] - pose.y)) > threshold:
            wait = self.action_client.wait_for_result(rospy.Duration(0.5))
            if server_unavailable(start_time):
                return False
        return True

    def step(self, object_center):
        # go in the point direction, stop
        self.action_client.wait_for_server()

        self.add_goal(object_center[0], object_center[1])
        if not self.move(object_center, threshold=1.1, start_time=time.time()):
            exit(1)

        # circle the object
        pose_dist = object_center - np.array(pose.x, pose.y)
        new_point = object_center + pose_dist
        self.add_goal(new_point[0], new_point[1], text="opposite")
        start_time = time.time()

        while np.linalg.norm(np.array(new_point[0] - pose.x, new_point[1] - pose.y)) > 1.3:
            pose_dist = np.array(new_point[0] - pose.x, new_point[1] - pose.y)
            total_dist = np.linalg.norm(pose_dist)
            pose_sign = np.sign(pose_dist)

            if total_dist > 1.5:
                last_point = (new_point[0] + 0.5 * pose_sign[0], new_point[1] - 0.5 * pose_sign[1])

            wait = self.action_client.wait_for_result(rospy.Duration(0.5))
            if server_unavailable(start_time):
                exit(1)

        self.add_goal(last_point[0], last_point[1], text="opposite")
        if not self.move(last_point, threshold=0.25, start_time=start_time):
            exit(1)

        return self.action_client.get_result()

    @staticmethod
    def random_step(self):
        print("Moving randomly...")
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        vel_msg = Twist()
        vel_msg.linear.x = random.uniform(-0.1, 0.1)
        vel_msg.angular.z = random.uniform(-2, 2)
        pub.publish(vel_msg)

    def run(self, obj_cen):
        # if there are non goal objects, make random steps until the robot finds one
        while len(obj_cen) == 0:
            self.random_step()
            # update the obj_cen list
            args = CLI.parse_args()
            flat_centers = args.centers_list
            for i in range(len(flat_centers) // 2):
                obj_cen.append([flat_centers[2 * i], flat_centers[2 * i + 1]])

        for i, obj in enumerate(obj_cen):
            self.step(obj)
            print("object {} is at: {}".format(i, obj))
            self.map.show()


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    # init
    rospy.init_node('assignment2')

    CLI = argparse.ArgumentParser()
    CLI.add_argument(
        "--centers_list",
        nargs="*",
        type=float,
        default=[],
    )
    args = CLI.parse_args()
    flat_centers = args.centers_list
    centers = []
    for i in range(len(flat_centers) // 2):
        centers.append([flat_centers[2*i], flat_centers[2*i+1]])

    # update cost maps
    gcm_client = dynamic_reconfigure.client.Client("/move_base/global_costmap/inflation_layer")
    gcm_client.update_configuration({"inflation_radius": 0.3})
    lcm_client = dynamic_reconfigure.client.Client("/move_base/local_costmap/inflation_layer")
    lcm_client.update_configuration({"inflation_radius": 0.3})

    tb3 = TurtleBot()
    tb3.run(centers)

    # show the mapped map
    tb3.map.show()
