#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import numpy as np
import time

import math

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

LOOKAHEAD_WPS = 25 #100  # was: 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = .5

INTERVAL_THRESHOLD_MS_POSE = 10
INTERVAL_THRESHOLD_MS_TRAFFIC = 250
#STOP_COUNTER_LIMIT_LOW = 1100
#STOP_COUNTER_LIMIT_HIGH = 1120

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=0)

        self.last_processed_time_pose = -1
        self.last_processed_time_traffic = -1
        
        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = -1
#        self.stop_counter = -1
        
	# for optimization
        self.prev_pose_x = None
        self.prev_pose_y = None
        self.prev_lane = None
        self.prev_stopline_wp_idx = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(20) # was: 50
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints and self.waypoint_tree:
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest is in front or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # hyperplane
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        if (val > 0):
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def publish_waypoints(self):
        lane = self.generate_lane()
        self.final_waypoints_pub.publish(lane)

    def generate_lane(self):
        lane = Lane()
        
        light_changed = True
        if self.prev_stopline_wp_idx is not None:
            light_changed = self.prev_stopline_wp_idx != self.stopline_wp_idx
                
        eps = 1E-4 # position precision is up to 2 digits after comma   
        # part of optimization. no need to recalculate waypoints if position didn't change
        if self.prev_pose_x is not None and self.prev_pose_y is not None:
                if np.fabs(self.prev_pose_x - self.pose.pose.position.x) < eps and np.fabs(self.prev_pose_y - self.pose.pose.position.y) < eps and not light_changed:
                    # rospy.logerr("using previous waypoints"):
                    return self.prev_lane 
                    
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]
        
#        self.stop_counter = self.stop_counter + 1
#        rospy.loginfo(self.stop_counter)        
        if self.stopline_wp_idx == -1 or self.stopline_wp_idx >= farthest_idx:
            lane.waypoints = base_waypoints
#            if self.stop_counter < STOP_COUNTER_LIMIT_LOW:
#                lane.waypoints = base_waypoints
#                rospy.logerr(self.stop_counter)
#            if self.stop_counter <= STOP_COUNTER_LIMIT_HIGH and self.stop_counter >= STOP_COUNTER_LIMIT_LOW:
#                lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
#                rospy.logerr("I'm braking for the sake of simulator stability")
#            if self.stop_counter > STOP_COUNTER_LIMIT_HIGH:
#                self.stop_counter = 0
#                rospy.logerr("release .... .......... ..................")
        else:
            rospy.logerr("I'm in this hell")
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        lane.header = self.base_waypoints.header
        self.prev_lane = lane
        self.prev_pose_x = self.pose.pose.position.x
        self.prev_pose_y = self.pose.pose.position.y
        self.prev_stopline_wp_idx = self.stopline_wp_idx
        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < .1:
                vel = 0.
            p.twist.twist.linear.x = min(vel, p.twist.twist.linear.x)
            temp.append(p)
        return temp

    def pose_cb(self, msg):
        rospy.loginfo("Hello!!!!!")

        time_start_pose = time.time()
        if self.last_processed_time_pose > 0 and (time_start_pose - self.last_processed_time_pose) * 1000 < INTERVAL_THRESHOLD_MS_POSE:
            rospy.loginfo("Skipping pose msg waypoint_updater")
            return
        self.last_processed_time_pose = time_start_pose
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in
                                 waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        time_start_traffic = time.time()
        if self.last_processed_time_traffic > 0 and (time_start_traffic - self.last_processed_time_traffic) * 1000 < INTERVAL_THRESHOLD_MS_TRAFFIC:
            rospy.loginfo("Skipping traffic msg waypoint_updater")
            return

        self.last_processed_time_traffic = time_start_traffic
        
        rospy.loginfo("Traffic light: %s", msg)
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
