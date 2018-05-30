#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import numpy as np
import math

from std_msgs.msg import Int32


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
MAX_DECEL = .75


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # DONE: Add other member variables you need below
        self.base_lane = None
        self.pose = None
        self.stopline_wp_idx = -1
        self.waypoints = None
        self.waypoint_tree = None
        self.initialized = False
        #self.actual_index = 0

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # DONE: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)



        self.loop()
        # rospy.Timer(rospy.Duration(0.02), self.update_waypoints_cb)

    # DONE: Get waypoints at 50 Hz
    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose is not None and self.waypoint_tree is not None:
                self.publish_waypoints()
            rate.sleep()

    # DONE
    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x,y],1)[1]

        # Check if closest is ahead or behind
        closest_coord = self.waypoints[closest_idx]
        prev_coord = self.waypoints[closest_idx-1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])

        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)

        if val > 0:
            closest_idx = (closest_idx+1) % len(self.waypoints)
        return closest_idx

    # DONE
    def publish_waypoints(self):
        final_lane = self.generate_lane()
        if self.initialized:
            self.final_waypoints_pub.publish(final_lane)
        else:
            rospy.logwarn("waiting camera feed to start.. until then do not calculate path...")

    # DONE
    def generate_lane(self):
        lane = Lane()

        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
            #rospy.logwarn("Speed up! stopline_wp_idx %d | farthest_idx %d | stopline>fatherst %d"%(self.stopline_wp_idx,farthest_idx, self.stopline_wp_idx >= farthest_idx) )

        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
            #rospy.logwarn("Decelerating! stopline_wp_idx %d | farthest_idx %d | stopline>fatherst %d"%(self.stopline_wp_idx,farthest_idx, self.stopline_wp_idx >= farthest_idx) )


        return lane

    # DONE
    # TODO: use linear instead of sqrt
    # TODO: slice the list to prevent latency if list is long while enumerating
    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints[:-1][::-1]):
            p = Waypoint()
            p.pose = wp.pose

            stop_idx = max(self.stopline_wp_idx-closest_idx-3,0) # 3 points back from line so car stops before line
            dist = self.distance(waypoints,i,stop_idx)
            vel = math.sqrt(2*MAX_DECEL*dist)
            if vel < 1.:
                vel = 0.

            p.twist.twist.linear.x = min(vel,wp.twist.twist.linear.x)
            temp.append(p)
        #rospy.logwarn("Temp waypoints appended!")
        return temp


    def pose_cb(self, msg):
        # Just update the internal pose variable with the latest
        self.pose = msg

    def waypoints_cb(self, msg):
        # DONE: Get the base waypoints, this is just updated once by the publisher
        self.base_lane = msg
        if not self.waypoints:
            self.waypoints = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in msg.waypoints ]
            self.waypoint_tree = KDTree(self.waypoints)
            rospy.logwarn("Base waypoints loaded in waypoint updater!")

    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg.data
        if self.initialized==False:
            self.initialized = True
            rospy.logwarn("All nodes initialized! Start calculating path..")
        #rospy.logwarn("Traffic callback -> %d "%msg.data)

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

'''
    def update_waypoints_cb(self,event):
        if self.waypoint_tree and self.actual_pose:

            self.actual_index = self.get_closest_waypoint_idx()

            msg = Lane()
            msg.header = self.base_lane_waypoints.header
            msg.waypoints = self.base_lane_waypoints.waypoints[self.actual_index:self.actual_index+LOOKAHEAD_WPS]
            self.final_waypoints_pub.publish(msg)
'''

if __name__ == '__main__':
    try:
        rospy.init_node('waypoint_updater')
        WaypointUpdater()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
