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

from common.waypoints import Waypoints

STATE_COUNT_THRESHOLD = 3
LIGHT_NOT_FOUND = (-1, TrafficLight.UNKNOWN)


class StopLines:
    def __init__(self, waypoint_positions, stopline_positions):
        self.waypoints = Waypoints(waypoint_positions)
        self.stopline_waypoints = self._calc_stopline_waypoints(stopline_positions)

    def _calc_stopline_waypoints(self, stopline_positions):
        return [self.waypoints.find_closest(x, y) for x, y in stopline_positions]

    def find_nearest_stopline(self, my_position):
        my_waypoint = self.waypoints.find_closest(my_position.x, my_position.y)
        min_distance = len(self.waypoints.waypoints)
        nearest_so_far = None

        for light_index, stopline_waypoint in enumerate(self.stopline_waypoints):
            distance = stopline_waypoint - my_waypoint
            if (distance >= 0) and (distance < min_distance):
                min_distance = distance
                nearest_so_far = (light_index, stopline_waypoint)

        return nearest_so_far


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.stoplines = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber(
            '/current_pose', PoseStamped, self.update_current_pose)
        sub2 = rospy.Subscriber('/base_waypoints', Lane,
                                self.update_base_waypoints)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights',
                                TrafficLightArray, self.update_traffic_lights)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher(
            '/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def update_current_pose(self, msg):
        self.pose = msg
        self.image_cb(None)

    def update_base_waypoints(self, lane):
        self.waypoints = Waypoints(lane.waypoints)
        self.stoplines = StopLines(
            lane.waypoints, self.config['stop_line_positions'])

    def update_traffic_lights(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
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
            self.publish_upcoming_light(light_wp)
        else:
            self.publish_upcoming_light(self.last_wp)

        self.state_count += 1

    def publish_upcoming_light(self, light_wp):
        self.upcoming_red_light_pub.publish(Int32(light_wp))

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        return light.state
        # if(not self.has_image):
        #     self.prev_light_loc = None
        #     return False

        # cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # # Get classification
        # return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if not (self.pose and self.waypoints and len(self.lights) > 0):
            return LIGHT_NOT_FOUND

        nearest_stopline = self.stoplines.find_nearest_stopline(
            self.pose.pose.position)
        if nearest_stopline:
            light_index, waypoint = nearest_stopline
            nearest_light = self.lights[light_index]
            state = self.get_light_state(nearest_light)
            return waypoint, state
        else:
            return LIGHT_NOT_FOUND


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
