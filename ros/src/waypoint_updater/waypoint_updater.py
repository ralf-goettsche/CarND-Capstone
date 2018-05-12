#!/usr/bin/env python

import rospy
import math
import json
import os

from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

from common.waypoints import Waypoints

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
MAX_DECEL = 1.0
SAFE_STOP_GAP = 3

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped,
                         self.update_current_pose)
        rospy.Subscriber('/base_waypoints', Lane, self.update_base_waypoints)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.update_traffic_light_waypoint)

        self.final_waypoints_pub = rospy.Publisher(
            'final_waypoints', Lane, queue_size=1)

        self.current_pose = None
        self.waypoints = None
        self.stopline_wp_index = -1

        self.update_loop()

    def update_loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.current_pose and self.waypoints:
                closest_wp_idx = self.closest_waypoint_index()
                self.publish_waypoints(closest_wp_idx)
            rate.sleep()

    def publish_waypoints(self, nearest_wp_index):
        lane = self.generate_lane(nearest_wp_index)
        self.final_waypoints_pub.publish(lane)

    def generate_lane(self, nearest_wp_index):
        lane = Lane()
        farthest_wp_index = nearest_wp_index + LOOKAHEAD_WPS
        final_waypoints = self.waypoints.get_waypoints_in_range(nearest_wp_index, farthest_wp_index)
        if (self.stopline_wp_index < 0) or (self.stopline_wp_index > farthest_wp_index):
            lane.waypoints = final_waypoints
        else:
            stop_wp_index = max(self.stopline_wp_index - nearest_wp_index - SAFE_STOP_GAP, 0)
            lane.waypoints = self.decelerate(final_waypoints, stop_wp_index)
        return lane

    def decelerate(self, base_waypoints, stop_wp_index):
        result = []
        for i, base_wp in enumerate(base_waypoints):
            new_wp = Waypoint()
            new_wp.pose = base_wp.pose
            dist = self.distance(base_waypoints, i, stop_wp_index)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1: 
                vel = 0
            new_wp.twist.twist.linear.x = min(vel, base_wp.twist.twist.linear.x)
            result.append(new_wp)
        
        return result

    def closest_waypoint_index(self):
        return self.waypoints.find_closest_ahead(self.current_pose.position.x,
                                                 self.current_pose.position.y)

    def update_current_pose(self, msg):
        self.current_pose = msg.pose

    def update_base_waypoints(self, lane):
        self.waypoints = Waypoints(lane.waypoints)

    def update_traffic_light_waypoint(self, msg):
        self.stopline_wp_index = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0

        def dl(a, b): return math.sqrt(
            (a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
