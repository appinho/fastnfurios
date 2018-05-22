#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree
from scipy.interpolate import interp1d
import numpy as np
import math

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
MAX_ACC = 10  # Maximum acceleration
KMH_TO_MS = 1.0 / 3.6

class WaypointUpdater(object):

    def __init__(self):
        rospy.init_node('waypoint_updater')

        #  subscribe to current pose and base_waypoints
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher(
            'final_waypoints', Lane, queue_size=1)

        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.traffic_waypoint = None
        self.closest_idx = None

        self.loop()

    def loop(self):
        # loop implemented rather than spin to control the frequency precisely
        rate = rospy.Rate(50)  # 50Hz

        while not rospy.is_shutdown():
            # if pose and base_waypoints are filled
            if self.pose and self.base_waypoints:
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.closest_idx = closest_waypoint_idx
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()

    def get_closest_waypoint_idx(self):

        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def publish_waypoints(self, closest_idx):
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[
            closest_idx:closest_idx + LOOKAHEAD_WPS]
        self.adjust_velocity(lane.waypoints)
        self.final_waypoints_pub.publish(lane)

    def adjust_velocity(self, waypoints):
        if self.traffic_waypoint is None:
            return
        if self.traffic_waypoint - self.closest_idx >= LOOKAHEAD_WPS:
            return
        num_published = min(len(waypoints), LOOKAHEAD_WPS)
        if self.traffic_waypoint < 0:
            start_velocity = self.get_waypoint_velocity(waypoints[0])
            end_velocity = rospy.get_param('/waypoint_loader/velocity') * KMH_TO_MS
            start_index = 0
            stop_index = len(waypoints)-1
            distance_to_max_vel = self.distance(waypoints,start_index,stop_index)
            acc_time = (end_velocity - start_velocity)/MAX_ACC
            while distance_to_max_vel > 0.5*MAX_ACC*acc_time**2:
                stop_index -= 1
                distance_to_max_vel = self.distance(waypoints,start_index,stop_index)
            index_diff = stop_index-start_index
            vel_diff = end_velocity-start_velocity
            if index_diff < 4:
                pos = [0,1,2,3,4]
            else:
                pos = [0, index_diff/4, index_diff/2, index_diff*3/4, stop_index]
            vel = [start_velocity+1, start_velocity+vel_diff/4, start_velocity+vel_diff/2, start_velocity+vel_diff*3/4, end_velocity]
            cubic_fn = interp1d(pos,vel,kind='cubic',bounds_error=False,fill_value="extrapolate")
            for i in range(0,stop_index):
                self.set_waypoint_velocity(waypoints,i,cubic_fn(i))
            for i in range(stop_index + 1, num_published):
                self.set_waypoint_velocity(waypoints, i, end_velocity)
        if self.traffic_waypoint - self.closest_idx > 8:
            local_waypoint = self.traffic_waypoint - self.closest_idx
            start_velocity = self.get_waypoint_velocity(waypoints[0])
            end_velocity = 0
            start_index = 0
            stop_index = local_waypoint - 8
            index_diff = stop_index-start_index
            vel_diff = end_velocity-start_velocity
            if start_index < 0:
                start_index = 0
            if index_diff < 4:
                pos = [0,1,2,3,4]
            else:
                pos = [start_index, start_index+index_diff/4, start_index+index_diff/2, start_index+index_diff*3/4, stop_index]
            vel = [start_velocity, start_velocity+vel_diff/4, start_velocity+vel_diff/2, start_velocity+vel_diff*3/4, end_velocity]
            cubic_fn = interp1d(pos,vel,kind='cubic',bounds_error=False,fill_value="extrapolate")
            for i in range(start_index,stop_index):
                self.set_waypoint_velocity(waypoints,i,min(cubic_fn(i),waypoints[i].twist.twist.linear.x))
            for i in range(stop_index + 1, num_published):
                self.set_waypoint_velocity(waypoints, i, end_velocity)

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints

        # the if statement does the evaluation just at the starting
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x,
                                  waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.traffic_waypoint = msg.data

    def obstacle_cb(self, msg):
        # TODO
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt(
            (a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')