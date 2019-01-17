#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from scipy.spatial import KDTree
import time
import os

STATE_COUNT_THRESHOLD = 3
INTERVAL_THRESHOLD_MS_IMAGE = 300
INTERVAL_THRESHOLD_MS_POSE = 10
INTERVAL_THRESHOLD_MS_TRAFFIC = 300

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.camera_image = None
        self.lights = []
        self.last_processed_time = -1
        self.last_processed_time_pose = -1
        self.last_processed_time_traffic = -1

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)
        sub7 = rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb)        #########

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        model_file = rospy.get_param("/traffic_light_model")
        if model_file is None or model_file == "" or not os.path.exists(model_file):
            rospy.loginfo("Can't use tl model: %s", model_file)
            self.light_classifier = None
        else:
            rospy.loginfo("Using tl model: %s", model_file)
            self.light_classifier = TLClassifier(model_file)

        self.bridge = CvBridge()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.image_ctr = 0

        rospy.spin()


    def final_waypoints_cb(self, final_waypoints):
# test 1: Can we detect a deviation from waypints: --> NO. The simulator knows more than it wants to reveal on ros nodes ...:(    
#        deviation = ((final_waypoints.waypoints[0].pose.pose.position.x - self.pose.pose.position.x)**2 + (final_waypoints.waypoints[0].pose.pose.position.y - self.pose.pose.position.y)**2)**.5
#        rospy.loginfo(deviation)
# test 2: Can we go for a full stop from tme to time? Lag seems to add up.
# Adding a function in waypoint_updater did not help (self.stop_counter)

        return


    def pose_cb(self, msg):
        time_start_pose = time.time()
        if self.last_processed_time_pose > 0 and (time_start_pose - self.last_processed_time_pose) * 1000 < INTERVAL_THRESHOLD_MS_POSE:
#            rospy.loginfo("Skipping pose msg tl_detector")
            return

        self.last_processed_time_pose = time_start_pose
        self.pose = msg

    def waypoints_cb(self, waypoints):
#        rospy.loginfo(self.pose.pose.position.x)
# y-position        rospy.loginfo(self.pose.pose.position.x)
        self.waypoints = waypoints       
#        rospy.loginfo("Waypoint 0 x")
#        rospy.loginfo(self.waypoints.waypoints[0].pose.pose.position.x)
#        rospy.loginfo("Waypoint 0 y")
#        rospy.loginfo(self.waypoints.waypoints[0].pose.pose.position.x)
#        rospy.loginfo("Waypoint 1")
#        rospy.loginfo(self.waypoints.waypoints[1].pose.pose.position.x)
#        rospy.loginfo("Waypoint 24")
#        rospy.loginfo(self.waypoints.waypoints[24].pose.pose.position.x)
#        rospy.loginfo("Waypoint 25")
#        rospy.loginfo(self.waypoints.waypoints[25].pose.pose.position.x)
#        rospy.loginfo("Waypoint 1000")
#        rospy.loginfo(self.waypoints.waypoints[1000].pose.pose.position.x)
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in
                                 waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)


    def traffic_cb(self, msg):
        time_start_traffic = time.time()
        if self.last_processed_time_traffic > 0 and (time_start_traffic - self.last_processed_time_traffic) * 1000 < INTERVAL_THRESHOLD_MS_TRAFFIC:
#            rospy.loginfo("Skipping traffic msg tl_detector")
            return

        self.last_processed_time_traffic = time_start_traffic
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera
        """

        if self.process_traffic_lights()[0] < 0:
#            rospy.loginfo("No traffic light around")
            return
        
        time_start = time.time()
        if self.last_processed_time > 0 and (time_start - self.last_processed_time) * 1000 < INTERVAL_THRESHOLD_MS_IMAGE:
#            rospy.loginfo("Skipping frame")
            return

        self.last_processed_time = time_start

        rospy.loginfo('Processing image. State %s, last %s, count %s', self.state, self.last_state, self.state_count)

        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            rospy.loginfo('Publishing new red light state: %s, wp %s', self.state, self.last_wp)
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            rospy.loginfo('Publishing last red light state: %s, wp %s', self.state, self.last_wp)
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        return self.waypoint_tree.query([x, y], 1)[1]

    def get_light_state(self, light):
        """Determines the current color of the traffic light
        Args:
            light (TrafficLight): light to classify
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        if self.light_classifier is None:
            # Just return the state from the light
            return light.state

        if not self.has_image:
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        closest_light = None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if self.pose:
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                d = temp_wp_idx - car_wp_idx
		dist_sq = (self.pose.pose.position.x - line[0])**2 + (self.pose.pose.position.y - line[1])**2 # distance square
                if d >= 0 and d < diff and dist_sq < 1E4:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx

        # TODO find the closest visible traffic light (if one exists)

        if closest_light:
            state = self.get_light_state(closest_light)
            return line_wp_idx, state

        # self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
