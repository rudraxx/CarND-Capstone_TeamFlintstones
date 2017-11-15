#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import TwistController, GAS_DENSITY, ONE_MPH
from yaw_controller import YawController

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')
        rospy.loginfo('abhishek - Starting dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)


        # TODO: Create `TwistController` object
        # specify the controller gain parameters
        #1) Velocity controller
        #self.velocity_control_params = [0.3, 0.01, 0.02]
        self.velocity_control_params = [0.5, 0.0,0.02]

        #2 Yaw controller
        yaw_controller = YawController(wheel_base, steer_ratio, ONE_MPH, max_lat_accel, max_steer_angle)
        total_vehicle_mass = vehicle_mass + fuel_capacity * GAS_DENSITY

        # Initalize the TwistController object
        self.controller = TwistController(self.velocity_control_params,total_vehicle_mass,
                                            accel_limit, decel_limit,wheel_radius, yaw_controller)

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/current_velocity',TwistStamped,self.cv_callback, queue_size=1)
        rospy.Subscriber('/twist_cmd',TwistStamped,self.twist_callback, queue_size=1)
        rospy.Subscriber('/vehicle/dbw_enabled',Bool,self.dbw_callback, queue_size=1)

        self.dbw_enabled = False

        # Init Twist values
        self.twist_cmd_twist = TwistStamped().twist
        self.current_velocity_twist = TwistStamped().twist

        # self.desired_linear_vel = 0.0 # only x direction required
        # self.desired_ang_vel    = 0.0 # only z direction required

        # self.actual_linear_vel  = 0.0
        # self.actual_ang_vel     = 0.0

        # Timers for the logitudinal controller. Not sure if we can rely on 50Hz rate.
        self.prev_vel_msg_time = 0.0

        # Specify the loop rate for the node
        self.loop_rate = 50.0
        self.loop()

    def cv_callback(self,msg):
        # Maybe it is to keep it ad twist object as we need to pass it to twist_controller for yaw_controller
        self.current_velocity_twist = msg.twist

        # self.actual_linear_vel = msg.twist.linear.x
        # self.actual_ang_vel    = msg.twist.angular.z
        # rospy.loginfo('abhishek - cv_callback: act_vel: %s' % str(self.actual_linear_vel))

    def twist_callback(self,msg):
        # Maybe it is to keep it ad twist object as we need to pass it to twist_controller for yaw_controller
        self.twist_cmd_twist = msg.twist

        # self.desired_linear_vel = msg.twist.linear.x
        # self.desired_ang_vel    = msg.twist.angular.z
        # rospy.loginfo('abhishek - twist_callback: desired_linear_vel: %s' % str(self.desired_linear_vel))

    def dbw_callback(self,msg):
        self.dbw_enabled = msg.data # data is a boolean value
        rospy.loginfo('abhishek - dbw_received: %s'% self.dbw_enabled)

    def loop(self):
        rate = rospy.Rate(int(self.loop_rate)) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled

            current_time = rospy.get_time()
            delta_time =  current_time - self.prev_vel_msg_time

            self.prev_vel_msg_time  = current_time

            # calculate the velocity error . in meter / sec
            # velocity_error_mps =  self.desired_linear_vel - self.actual_linear_vel

            # Use dbw value to reset the controller states if needed.
            throttle, brake, steering = self.controller.control(self.dbw_enabled,
                                                                self.twist_cmd_twist,
                                                                self.current_velocity_twist,
                                                                delta_time)
            # rospy.loginfo('dbw_enable: %s, vel_error: %s , throttle: %s \n '% (self.dbw_enabled, velocity_error_mps, throttle))
            rospy.loginfo('throttle: %s, brake: %s, steering: %s  \n '% (throttle, brake, steering))

            if self.dbw_enabled:
                self.publish(throttle, brake, steering)
            #   self.publish(1.0, 0.0, 0.0)

            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    try:
        DBWNode()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start DBW node.')
