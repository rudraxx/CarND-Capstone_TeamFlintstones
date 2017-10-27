#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
TARGET_VELOCITY = 20


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.loginfo('abhishek -Node waypoint_updated started.')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
	rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
	rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        # Read the below statement as
        # waypoint_updater pubishing on topic: 'final_waypoints' of type: 'Lane', with queue_size of : 1
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.master_lane_data = Lane()
        self.current_pose_msg = PoseStamped()

        self.traffic_wp = Int32()

        # Logging data only once for debugging current_pose topic
        self.log_once_done = False;
        self.received_waypoints = False;

        # Additional data
        # Ego pose data
        self.x = 0.0 # world frame, meters
        self.y = 0.0 # world frame, meters
        self.heading = 0.0 # world frame, radians

        # Publisher related
        # self.publish_rate = 10; # 10 Hz

        self.loop()
        # rospy.spin()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            self.publish_waypoints()
            rate.sleep()

    def pose_cb(self, msg):
        # TODO: Implement
        if not self.log_once_done:
            rospy.loginfo("Logging data for the full current pose message as reference:")
            rospy.loginfo(msg)
            rospy.loginfo("Logging only position data, curret_pose. x: %s,y: %s, z: %s" % (msg.pose.position.x,msg.pose.position.y,msg.pose.position.z))
            self.log_once_done = True

        # Update current pose
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.current_pose_msg = msg
        # current_pose orientation is in quaternions. Not sure we need this just yet.
        # If needed, will have to include a method to convert quaternion to euler angles.
        # self.heading = msg.pose.orientation.z

        # # Preventing case when pose data comes before the waypoint data for the first iteration. In that case, we wont have any waypoint info.
        # if(self.received_waypoints):
        #     self.publish_waypoints()
        #     # self.received_waypoints = False # temp for debugging. delete this line.
        # # pass

    def publish_waypoints(self):

        # Preventing case when pose data comes before the waypoint data for the first iteration. In that case, we wont have any waypoint info.
        if(self.received_waypoints):

        # 1) Find the closest waypoint to the current position:
            # Get distance of this waypt to the current pose
            # rospy.loginfo("Calculating closest waypoint: current_x: %s, current_y: %s"% (self.current_pose_msg.pose.position.x,self.current_pose_msg.pose.position.y))

            next_wp_idx = self.get_next_waypoint(self.current_pose_msg.pose,self.master_lane_data.waypoints)

            # Log some information about the closest waypt.
            # rospy.loginfo("Closest waypoint - idx: %s, dist: %s"% (closest_wp_idx,closest_wp_dist))
            # rospy.loginfo(self.master_lane_data.waypoints[closest_wp_idx])

       # 2) Now that we have the closest waypoint, create a new list for the final waypoints
            # Get the next LOOKAHEAD_WPS waypoints.
            # How to check if the closest waypoint is ahead or behind us?
            array_final_waypoints = Lane()
            # rospy.loginfo("array_final_waypoints size: %s"% (len(array_final_waypoints.waypoints)))
            # rospy.loginfo("LOOKAHEAD_WPS: %s"% LOOKAHEAD_WPS)

	    #  calculate target velocity increment to accelerate from stop state
	    target_velocity_mps = TARGET_VELOCITY * 0.44704
	    velocity_increment = target_velocity_mps / LOOKAHEAD_WPS  #mph to m/s conversion factor

	    first = 1
            for idx_waypt in range(LOOKAHEAD_WPS):
                idx_waypt_to_append = (next_wp_idx + idx_waypt)% len(self.master_lane_data.waypoints)

		waypt_to_append = self.master_lane_data.waypoints[idx_waypt_to_append]

		if(first):
			velocity = self.get_waypoint_velocity(waypt_to_append)
			start_velocity = velocity
			first = 0
	        velocity_decrement = 0
		rospy.loginfo("MK/WP1 Waypoint %s velocity %s idx %s traffic_wp %s"% (idx_waypt_to_append, velocity, idx_waypt, self.traffic_wp.data))
		#if(self.traffic_wp.data != -1): #RED light coming up
		waypt_to_append.twist.twist.linear.x = target_velocity_mps;

		if(self.traffic_wp >= 0): #RED light coming up
			rospy.loginfo("MK/WP2 red light coming up for waypoint %s"% idx_waypt_to_append)
			
			if(velocity != 0):
				rospy.loginfo("MK/WP21 red light coming up for waypoint %s idx_waypt %s"% (idx_waypt_to_append, idx_waypt))
				#vehicle is in motion, initiate deceleration
				#if(idx_waypt == 0):
				rospy.loginfo("MK/WP22 red light coming up for waypoint %s (velocity %s)"% (idx_waypt_to_append, velocity))
					#num_waypts_to_light = self.traffic_wp.data - idx_waypt_to_append;
				#TODO:: calculate num_waypts_to_light 
				num_waypts_to_light = 10
				rospy.loginfo("MK/WP23 red light coming up for waypoint %s number of waypoints to light %s"% (idx_waypt_to_append, num_waypts_to_light))
            			#rospy.loginfo("MK/WP3 Red light coming up, initiate deceleration from waypoint %s to waypoint %s"% (self.traffic_wp, idx_waypt_to_append))

				#calculate velocity decrement during first waypoint iteration
				velocity_decrement = start_velocity / num_waypts_to_light

				velocity -= velocity_decrement
				if(velocity < 0):
					#rest of the waypoints have zero velocity
					velocity = 0 

				rospy.loginfo("MK/WP24 decremented velocity for waypoint %s to %s (velocity_decrement %s)"% (idx_waypt_to_append, velocity, velocity_decrement))
				waypt_to_append.twist.twist.linear.x = velocity;
		elif ((self.traffic_wp.data < 0) and (velocity == 0)):
		        rospy.loginfo("MK/WP4 Accelerate after red light stop for waypoint %s"% idx_waypt_to_append)
			#no red traffic light ahead and car is stopped 
			# => light just turned green and car needs to accelerate from zero mph
			velocity += velocity_increment
			if(velocity > target_velocity_mps):
				#clamp to max velocity
				velocity = target_velocity_mps
			waypt_to_append.twist.twist.linear.x = velocity;
			rospy.loginfo("MK/WP5 Red light turned green, accelerate from waypoint %s"% idx_waypt_to_append)
		else:
		        rospy.loginfo("MK/WP6 Set waypoint %s to target velocity %s"% (idx_waypt_to_append, target_velocity_mps))
			waypt_to_append.twist.twist.linear.x = target_velocity_mps;
			
		# log 10 waypoints for debug
		if (idx_waypt < 10):
                	rospy.loginfo("MK/WP7 final_waypoint[%s] %s velocity = %s"% 
					(idx_waypt, idx_waypt_to_append,waypt_to_append.twist.twist.linear.x))

                array_final_waypoints.waypoints.append(waypt_to_append)

            rospy.loginfo("MK/WP8 array_final_waypoints after for loop : %s"% (len(array_final_waypoints.waypoints)))

            # Publish the Lane info to the /final_waypoints topic
            self.final_waypoints_pub.publish(array_final_waypoints)

    def get_next_waypoint(self,ego_pose,waypoints):

        next_wp_idx = self.get_closest_waypoint(ego_pose,waypoints)

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


    def get_closest_waypoint(self,ego_pose,waypoints):

        closest_wp_dist = 10000.0;
        closest_wp_idx  = -1;

        for idx,waypt in enumerate(waypoints):
            delta_x = ego_pose.position.x - waypt.pose.pose.position.x
            delta_y = ego_pose.position.y - waypt.pose.pose.position.y
            delta_z = ego_pose.position.z - waypt.pose.pose.position.z
            dist_to_wp = math.sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z)

            if (dist_to_wp < closest_wp_dist):
                closest_wp_dist = dist_to_wp
                closest_wp_idx  = idx

        return closest_wp_idx


    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # Save waypoints in a variable, since the sender node /waypoint_loader publishes these only once.
        rospy.loginfo("abhishek - Received waypoints: %s" % str(len(waypoints.waypoints)))
        self.master_lane_data.waypoints = waypoints.waypoints
        rospy.loginfo("abhishek - waypoints in master_lane_data: %s" % len(self.master_lane_data.waypoints) )
        rospy.loginfo("Logging data for first waypoint as reference...")
        rospy.loginfo(self.master_lane_data.waypoints[0])
        # rospy.loginfo("Logging data for 272th waypoint as reference...")
        # rospy.loginfo(self.master_lane_data.waypoints[272])
        # rospy.loginfo('First waypoint info:  %s' % str(waypoints.waypoints[0]))
        self.received_waypoints = True

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_wp = Int32(msg)
        rospy.loginfo("Madhu/WU - Received traffic WP: %s" % self.traffic_wp)
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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

