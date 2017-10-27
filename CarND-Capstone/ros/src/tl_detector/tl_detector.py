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
            rospy.loginfo('MK/TL Publish light waypt: %s' % light_wp)
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            rospy.loginfo('MK/TL Publish last waypt: %s' % self.last_wp)
        self.state_count += 1


    def get_closest_waypoint(self, ego_pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        #TODO implement
        # Find the closest lightpost to our current position
        closest_light_dist = 10000.0;
        closest_light_idx  = -1;

        for idx,p_light in enumerate(self.lights):
            delta_x = ego_pose.position.x - p_light.pose.pose.position.x
            delta_y = ego_pose.position.y - p_light.pose.pose.position.y

            dist_to_light = math.sqrt(delta_x*delta_x + delta_y*delta_y)

            if (dist_to_light < closest_light_dist):
                closest_light_dist = dist_to_light
                closest_light_idx  = idx

        # closest_light_idx is the index of the traffic light closest to us.
        # Calculate the closest waypoint, only if the traffic light is within 100 meters
        # TODO: This should be 100 meters in front of us. Behind doesn't matter.

        light_x = self.lights[closest_light_idx].pose.pose.position.x
        light_y = self.lights[closest_light_idx].pose.pose.position.y

        # rospy.loginfo('closest traffic light idx: %s,dist: %s, light_x: %s, light_y: %s, ego_x: %s , ego_y: %s'% (closest_light_idx,closest_light_dist,light_x,light_y,ego_pose.position.x,ego_pose.position.y))

        heading = math.atan2((light_y - ego_pose.position.y), (light_x - ego_pose.position.x))

        quaternion = (ego_pose.orientation.x, ego_pose.orientation.y, ego_pose.orientation.z, ego_pose.orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)

        angle = abs(yaw - heading)
        # rospy.loginfo('angle: %s\n' % angle)

        #closest_wp_idx = -1
        # if (True) :
        if (angle < (math.pi / 4) and closest_light_dist <100 ) :
            # As asked, now find the which waypoint does this lie closest to.
            closest_wp_dist = 10000.0;
            closest_wp_idx  = -1;
            for idx,waypt in enumerate(self.waypoints.waypoints):
                delta_x = light_x - waypt.pose.pose.position.x
                delta_y = light_y - waypt.pose.pose.position.y
                dist_to_wp = math.sqrt(delta_x*delta_x + delta_y*delta_y)

                if (dist_to_wp < closest_wp_dist):
                    closest_wp_dist = dist_to_wp
                    closest_wp_idx  = idx
        else:
            closest_wp_idx = -1

        rospy.loginfo('closest waypt to light idx: %s, light_x: %s, light_y: %s, wp_x: %s , wp_y: %s'% (closest_wp_idx,light_x,light_y, self.waypoints.waypoints[closest_wp_idx].pose.pose.position.x,
                                                     self.waypoints.waypoints[closest_wp_idx].pose.pose.position.y))

        return closest_wp_idx, closest_light_idx


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

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        # rospy.loginfo('in process_traffic_lights')

        #TODO find the closest visible traffic light (if one exists)
        light_wp = -1
        state = -1

        #state_closest_traffic_light = TrafficLight.UNKNOWN
        #waypt_closest_to_light = -1

        if(self.pose and self.waypoints):
            waypt_closest_to_light, idx_lightpost = self.get_closest_waypoint(self.pose.pose)
            state_closest_traffic_light = self.lights[idx_lightpost].state
            # rospy.loginfo('Closest waypt: %s, state: %s' % (waypt_closest_to_light,state_closest_traffic_light))
            #TODO: Delete this next publish line.. Using only for debugging
            # if (state_closest_traffic_light==0):
                # rospy.loginfo('Traffic light red')
                # self.upcoming_red_light_pub.publish(Int32(waypt_closest_to_light))

        # Use this after the image processing is done.

        # if light:
        #     state = self.get_light_state(light)
        #     return light_wp, state
        # self.waypoints = None
        # return -1, TrafficLight.UNKNOWN

            if(waypt_closest_to_light==-1):
                state = TrafficLight.UNKNOWN
                light_wp = -1
            else:
            	rospy.loginfo('Closest light waypt: %s, state: %s' % (waypt_closest_to_light,state_closest_traffic_light))
                state  = state_closest_traffic_light
                light_wp = waypt_closest_to_light

        rospy.loginfo('Closest light waypt: %s, state: %s' % (light_wp,state))
        return light_wp, state


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
