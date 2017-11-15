#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import tf

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

LOOKAHEAD_WPS = 100  # Number of waypoints we will publish. You can change this number
#TARGET_VELOCITY = 45
ENABLE_TRAFFIC_LIGHTS = True
DESIRED_DECEL = -2.0 # m/s2
#DESIRED_DECEL =rospy.get_param('~decel_limit',-5.0)

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.loginfo('abhishek -Node waypoint_updated started.')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        self.target_velocity_mps = self.kmph2mps(rospy.get_param('~/waypoint_loader/velocity'))

        # Read the below statement as
        # waypoint_updater pubishing on topic: 'final_waypoints' of type: 'Lane', with queue_size of : 1
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.master_lane_data = Lane()
        self.current_pose_msg = PoseStamped()

        self.traffic_wp = -1
        self.light_status = None
        self.current_twist_msg = TwistStamped()

        # Logging data only once for debugging current_pose topic
        self.log_once_done = False
        self.received_waypoints = False

        self.loop()
        # rospy.spin()

    def loop(self):
        rate = rospy.Rate(50)  # 50Hz
        while not rospy.is_shutdown():
            self.publish_waypoints()
            rate.sleep()

    def pose_cb(self, msg):
        # TODO: Implement
        if not self.log_once_done:
            rospy.loginfo("Logging data for the full current pose message as reference:")
            rospy.loginfo(msg)
            rospy.loginfo("Logging only position data, curret_pose. x: %s,y: %s, z: %s" % (
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
            self.log_once_done = True

        # Update current pose msg
        self.current_pose_msg = msg

    def velocity_cb(self,msg):
        self.current_twist_msg = msg

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

    def accelerate(self,next_wp_idx,array_final_waypoints):

        #velocity_increment = self.target_velocity_mps/LOOKAHEAD_WPS

        for idx_waypt in range(LOOKAHEAD_WPS):
            idx_waypt_to_append = (next_wp_idx + idx_waypt) % len(self.master_lane_data.waypoints)

            #Get the information about the waypoint that we will append
            waypt_to_append = self.master_lane_data.waypoints[idx_waypt_to_append]
            #current_wp_vel = self.get_waypoint_velocity(waypt_to_append)
            current_vel = self.current_twist_msg.twist.linear.x

            #new_waypt_vel = min(current_vel+velocity_increment,target_velocity_mps)
            new_waypt_vel = self.target_velocity_mps

            waypt_to_append.twist.twist.linear.x = new_waypt_vel
            # Append the waypoint to the array of waypoints.
            array_final_waypoints.waypoints.append(waypt_to_append)

        # Publish the Lane info to the /final_waypoints topic
        self.final_waypoints_pub.publish(array_final_waypoints)

    def decelerate(self,next_wp_idx,array_final_waypoints,num_waypts_to_light):
            current_vel = self.current_twist_msg.twist.linear.x
            rospy.loginfo('num_waypts_to_light: %s, curr_vel: %.2f'%(num_waypts_to_light, current_vel) )
            # Start creating waypoints in reverse order.
            # WIll be much simpler to do velocity calcuations.

            buffer_points = 10
            if num_waypts_to_light> buffer_points:

                # FOr points ahead of traffic light, set velocity value to 0.
                for idx_waypt in range(LOOKAHEAD_WPS-1,num_waypts_to_light-buffer_points,-1):
                    idx_waypt_to_append = (next_wp_idx + idx_waypt) % len(self.master_lane_data.waypoints)
                    #Get the information about the waypoint that we will append
                    waypt_to_append = self.master_lane_data.waypoints[idx_waypt_to_append]

                    waypt_to_append.twist.twist.linear.x = 0.0
                    # Append the waypoint to the array of waypoints.
                    array_final_waypoints.waypoints.append(waypt_to_append)

                #traffic light wp is 0 velocity. all others slowly increase till current velocity
                end_vel = 0

                for idx_waypt in range(num_waypts_to_light-buffer_points,-1,-1) :

                    idx_waypt_to_append = (next_wp_idx + idx_waypt) % len(self.master_lane_data.waypoints)
                    #Get the information about the waypoint that we will append
                    waypt_to_append = self.master_lane_data.waypoints[idx_waypt_to_append]

                    #next_waypt = self.master_lane_data.waypoints[idx_waypt_to_append+1]

                    dist = self.distance(self.master_lane_data.waypoints,idx_waypt_to_append, idx_waypt_to_append+1)

                    wp_vel = math.sqrt(end_vel*end_vel - 2*DESIRED_DECEL*dist)
                    #new_waypt_vel = min(wp_vel,current_vel)
                    new_waypt_vel = min(self.target_velocity_mps, wp_vel)


                    waypt_to_append.twist.twist.linear.x = new_waypt_vel
                    # Append the waypoint to the array of waypoints.
                    array_final_waypoints.waypoints.append(waypt_to_append)

                    end_vel = wp_vel

            else:

                # Set all points to 0.....
                for idx_waypt in range(LOOKAHEAD_WPS-1,-1,-1):
                    idx_waypt_to_append = (next_wp_idx + idx_waypt) % len(self.master_lane_data.waypoints)
                    #Get the information about the waypoint that we will append
                    waypt_to_append = self.master_lane_data.waypoints[idx_waypt_to_append]

                    waypt_to_append.twist.twist.linear.x = 0.0
                    # Append the waypoint to the array of waypoints.
                    array_final_waypoints.waypoints.append(waypt_to_append)


            array_final_waypoints.waypoints.reverse()
            # Publish the Lane info to the /final_waypoints topic
            self.final_waypoints_pub.publish(array_final_waypoints)

            ###print data
#            rospy.loginfo('####****####')
#            rospy.loginfo('array_final size: %s'%len(array_final_waypoints.waypoints))
#            for i in range(LOOKAHEAD_WPS):
#                rospy.loginfo('adb -wpt: %s, speed: %.2f' %(i, array_final_waypoints.waypoints[i].twist.twist.linear.x))
#            rospy.loginfo('@@@@@@@@')


    def publish_waypoints(self):

        # Preventing case when pose data comes before the waypoint data for the first iteration. In that case, we wont have any waypoint info.
        if (self.received_waypoints):

            # 1) Find the closest waypoint to the current position:
            next_wp_idx = self.get_next_waypoint(self.current_pose_msg.pose, self.master_lane_data.waypoints)
            # rospy.loginfo("Calculating closest waypoint: current_x: %s, current_y: %s"% (self.current_pose_msg.pose.position.x,self.current_pose_msg.pose.position.y))

            # 2) Now that we have the closest waypoint, create a new list for the next LOOKAHEAD_WPS waypoints.
            array_final_waypoints = Lane()

            #  calculate target velocity increment to accelerate from stop state
            #target_velocity_mps = TARGET_VELOCITY * 0.44704
            #velocity_increment = target_velocity_mps / LOOKAHEAD_WPS  # mph to m/s conversion factor

            #flag_first_waypt = True
            #velocity_decrement = 0
            #flag_calc_decrement = True

            num_waypts_to_light = (self.traffic_wp - next_wp_idx)
            #num_waypts_to_light = (self.traffic_wp - next_wp_idx)% len(self.master_lane_data.waypoints)

            if (self.light_status==0 and (num_waypts_to_light< LOOKAHEAD_WPS) and ENABLE_TRAFFIC_LIGHTS) :
                # We have a red light coming up
                self.decelerate(next_wp_idx,array_final_waypoints,num_waypts_to_light)
            else:
                # No red light increment velocity to target velocity
                self.accelerate(next_wp_idx,array_final_waypoints)


    def get_next_waypoint(self, ego_pose, waypoints):

        next_wp_idx = self.get_closest_waypoint(ego_pose, waypoints)

        map_x = waypoints[next_wp_idx].pose.pose.position.x
        map_y = waypoints[next_wp_idx].pose.pose.position.y

        heading = math.atan2((map_y - ego_pose.position.y), (map_x - ego_pose.position.x))

        quaternion = (ego_pose.orientation.x, ego_pose.orientation.y, ego_pose.orientation.z, ego_pose.orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)

        # https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
        # quaternion as [w,x,y,z]
        # quaternion = (ego_pose.orientation.w,ego_pose.orientation.x, ego_pose.orientation.y, ego_pose.orientation.z)
        # q[0] = cos(r/2);
        # q[1] = sin(r/2)*x;
        # q[2] = sin(r/2)*y;
        # q[3] = sin(r/2)*z;

        angle = abs(yaw - heading)
        if angle > (math.pi / 4):
            next_wp_idx += 1

        return next_wp_idx

    def get_closest_waypoint(self, ego_pose, waypoints):

        closest_wp_dist = 10000.0;
        closest_wp_idx = -1;

        for idx, waypt in enumerate(waypoints):
            delta_x = ego_pose.position.x - waypt.pose.pose.position.x
            delta_y = ego_pose.position.y - waypt.pose.pose.position.y
            delta_z = ego_pose.position.z - waypt.pose.pose.position.z
            dist_to_wp = math.sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z)

            if (dist_to_wp < closest_wp_dist):
                closest_wp_dist = dist_to_wp
                closest_wp_idx = idx

        return closest_wp_idx

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # Save waypoints in a variable, since the sender node /waypoint_loader publishes these only once.
        rospy.loginfo("abhishek - Received waypoints: %s" % str(len(waypoints.waypoints)))
        self.master_lane_data.waypoints = waypoints.waypoints
        rospy.loginfo("abhishek - waypoints in master_lane_data: %s" % len(self.master_lane_data.waypoints))
        rospy.loginfo("Logging data for first waypoint as reference...")
        rospy.loginfo(self.master_lane_data.waypoints[0])
        # rospy.loginfo("Logging data for 272th waypoint as reference...")
        # rospy.loginfo(self.master_lane_data.waypoints[272])
        # rospy.loginfo('First waypoint info:  %s' % str(waypoints.waypoints[0]))
        self.received_waypoints = True

    def traffic_cb(self, msg):

        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_wp = msg.data
        rospy.loginfo("Madhu/WU - Received traffic WP: %s" % self.traffic_wp)

        # 06 Nov,abhishek: Adding logic for detecting yellow light, since the image classifier isn't giving us that information.
        # Looking for when the light changes from green to red. Then assuming that light is yellow for 10 loops. Since the traffic data is coming in at 10 Hz
        #self.light_status = 0,1,2 : green,yellow,red

        if (self.traffic_wp>0):
            self.light_status= 0
        else:
            self.light_status=2

        #rospy.loginfo('abhishek: self.light_loop_idx: %s, self.light_status= %s'%(self.light_loop_idx,self.light_status))



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
