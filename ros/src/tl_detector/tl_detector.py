#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from scipy.spatial import KDTree
import tf
import cv2
import yaml
import collections
from datetime import datetime as dt


STATE_VECTOR_SIZE = 6
STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.counter_frames=0
        self.ignore_frames=1

        self.pose = None
        self.waypoints = None
        self.waypoint_tree = None
        self.camera_image = None
        self.lights = []
        self.state_vector = []





        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, msg):
        self.waypoints = msg
        self.waypoints_vector = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in msg.waypoints ]
        self.waypoint_tree = KDTree(self.waypoints_vector)

        rospy.logwarn("Base waypoints loaded in tl_detector!")
        rospy.Subscriber('/image_color', Image, self.image_cb)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        #rospy.logwarn("image callback")
        if self.counter_frames> self.ignore_frames: self.counter_frames=0

        if self.counter_frames==0:
            
        
            self.has_image = True
            self.camera_image = msg
            light_wp, state = self.process_traffic_lights()
            
            '''
            Publish upcoming red lights at camera frequency.
            Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
            of times till we start using it. Otherwise the previous stable state is
            used.
            '''




            
            if len(self.state_vector)>STATE_VECTOR_SIZE: self.state_vector.pop(0)
            self.state_vector.append(state)

            C = collections.Counter(self.state_vector)
            k_state = -1
            frequency = -1
            for state in C:
                if (C[state]> frequency):
                    frequency = C[state]
                    k_state = state

            #print self.state_vector, C, frequency, k_state


            if frequency> STATE_COUNT_THRESHOLD:
                if ((k_state == TrafficLight.RED) or (k_state == TrafficLight.YELLOW)):
                    if self.last_wp== -1:
                        rospy.logwarn("state: STOP")
                        self.last_wp = light_wp
                        self.timeout_start_time = dt.now()

                elif (k_state == TrafficLight.GREEN):
                    if self.last_wp != -1: 
                        rospy.logwarn("state: GO")
                        self.last_wp = -1

            if self.last_wp != -1:
                rospy.logwarn("timeout time %f"%(dt.now() - self.timeout_start_time).total_seconds())
                if ((dt.now() - self.timeout_start_time).total_seconds() > 20):
                    rospy.logwarn("timeout reach!")
                    rospy.logwarn("state: GO")
                    self.last_wp = -1



            #TODO: create a timeout in case it enter in a deadlock


            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            


            '''
            if self.state != state:
                self.state_count = 0
                self.state = state
            elif self.state_count >= STATE_COUNT_THRESHOLD:
                self.last_state = self.state
                light_wp = light_wp if state == TrafficLight.RED else -1
                self.last_wp = light_wp
                self.upcoming_red_light_pub.publish(Int32(light_wp))
            else:
                self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            self.state_count += 1
            '''
            

        self.counter_frames +=1




    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #DONE
        closest_idx = self.waypoint_tree.query([x,y],1)[1]
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # For testing return light state
        # return light.state

        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #cv2.imshow("image",cv_image)
        #cv2.waitKey(0)

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closest to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None  # closest traffic light
        line_wp_idx = None  # index of the nearest waypoint of the stop line

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

        #DONE: find the closest visible traffic light (if one exists)
        diff = len(self.waypoints.waypoints)
        for i,light in enumerate(self.lights):
            #Get the stop line waypoint index
            line = stop_line_positions[i]   #each traffic light corresponds to a stop line
            temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
            #Find the closest stop line waypoint index
            d = temp_wp_idx - car_wp_idx
            if d >= 0 and  d < diff:
                diff = d
                closest_light = light
                line_wp_idx = temp_wp_idx

        if closest_light:
            state = self.get_light_state(closest_light)
            return line_wp_idx, state
        
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
