#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import numpy as np
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

LOOKAHEAD_WPS = 50 #200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.actual_pose = None
        self.base_lane_waypoints = None
        self.waypoints = None
        self.waypoint_tree = None
        self.actual_index = 0

        rospy.Timer(rospy.Duration(0.02), self.update_waypoints_cb)

        

    def pose_cb(self, msg):
        # Just update the internal pose variable with the latest
        self.actual_pose = msg


    def waypoints_cb(self, msg):
        # Get the base waypoints, this is just updated once by the publisher
        self.base_lane_waypoints = msg
        self.waypoints = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in msg.waypoints ]
        self.waypoint_tree = KDTree(self.waypoints)
        rospy.logwarn("Base waypoints Loaded!")

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def get_closest_waypoint_index(self):
        #maybe this function can be enhanced searching from the last waypoint
        #index = self.actual_index
        #for i in range(len(self.waypoints)):

        actual_pose = [self.actual_pose.pose.position.x,self.actual_pose.pose.position.y]
        index = self.waypoint_tree.query(actual_pose,1)[1]

        closest = np.array(self.waypoints[index])
        prev_closest = np.array(self.waypoints[index-1])
        actual_pose = np.array(actual_pose)

        if np.dot(closest-prev_closest,actual_pose-closest) > 0:
            return (index+1)%len(self.waypoints)
        return index


    def update_waypoints_cb(self,event):
        if self.waypoint_tree and self.actual_pose:

            self.actual_index = self.get_closest_waypoint_index()

            msg = Lane()
            msg.header = self.base_lane_waypoints.header
            msg.waypoints = self.base_lane_waypoints.waypoints[self.actual_index:self.actual_index+LOOKAHEAD_WPS]
            self.final_waypoints_pub.publish(msg)



if __name__ == '__main__':
    try:
        rospy.init_node('waypoint_updater')
        WaypointUpdater()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
