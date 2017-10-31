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
import math

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

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
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

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

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
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
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, stop_line_position):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            stop_line_position : position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        closest_wp_dist = 10000.0;
        closest_wp_idx  = -1;
        for idx,waypt in enumerate(self.waypoints.waypoints):
            delta_x = stop_line_position[0] - waypt.pose.pose.position.x
            delta_y = stop_line_position[1] - waypt.pose.pose.position.y
            dist_to_wp = math.sqrt(delta_x*delta_x + delta_y*delta_y)

            if (dist_to_wp < closest_wp_dist):
                closest_wp_dist = dist_to_wp
                closest_wp_idx  = idx

        # Javi_note: perhaps the inex before closest_wp_idx need to be taken 'cause
        # this position of the car probably is not the front of the car

        return closest_wp_idx


    def get_next_stop_line(self, pose, stop_line_positions):
        # receive the car position and the stop lane positions array
        # and return the closest dist and the index ahead of the vehicle

        closest_dist = 10000.0;
        closest_idx  = -1;
        for i in range(len(stop_line_positions)):
            dx = pose.position.x - stop_line_positions[i][0]
            dy = pose.position.y - stop_line_positions[i][1]
            dist = math.sqrt(dx*dx + dy*dy)

            if (dist < closest_dist):
                closest_dist = dist
                closest_idx  = i

        heading = math.atan2((stop_line_positions[closest_idx][1] - pose.position.y),
                             (stop_line_positions[closest_idx][0] - pose.position.x))

        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)

        angle = abs(yaw - heading)
        if angle > (math.pi / 4):
            closest_idx = (closest_idx + 1) % len(stop_line_positions)
            dx = pose.position.x - stop_line_positions[closest_idx][0]
            dy = pose.position.y - stop_line_positions[closest_idx][1]
            closest_dist = math.sqrt(dx*dx + dy*dy)

        return closest_dist, closest_idx

    def get_light_patch(self):
        # TODO team: implemet a function that take a patch of the self.camera_image. Here is where
        # could be useful self.lights to take a more precise patch.

        return None


    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_wp = -1
        state = -1

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose and self.waypoints):
            #TODO find the closest visible traffic light (if one exists)

            # find the next probable stop
            next_stop_line_dist, next_stop_line_idx = self.get_next_stop_line(self.pose.pose, stop_line_positions)

            #rospy.loginfo("Javi: next probable stop dist = %s" %next_stop_line_dist)

            # if the next probable stop is near of 50 mts (tweak this param if needed)
            if next_stop_line_dist < 50:
                # Get the waypoint index that we are going to send to the next node
                light_wp = self.get_closest_waypoint(stop_line_positions[next_stop_line_idx])

                # light = self.get_light_patch() # TODO

                state_closest_traffic_light = self.lights[next_stop_line_idx].state

            if (light_wp==-1):
                state = TrafficLight.UNKNOWN
            else:
                state = state_closest_traffic_light
                
        return light_wp,state

        # if light:
        #     state = self.get_light_state(light)
        #     return light_wp, state
        # self.waypoints = None
        # return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

