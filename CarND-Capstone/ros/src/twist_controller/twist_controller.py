
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


import rospy
from pid import PID

class TwistController(object):
    def __init__(self, pid_velocity_params,vehicle_mass, accel_limit, decel_limit,wheel_radius, yaw_controller):

        # TODO: Implement
        self.pid_velocity_params = pid_velocity_params
        self.vehicle_mass = vehicle_mass
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        self.wheel_radius = wheel_radius
        self.yaw_controller = yaw_controller

        # Create a pid controller of the velocity control
        # Changed mn and mx to values provided in dbw_node
        # Pawel Cisek 23.10
        self.velocity_controller = PID(self.pid_velocity_params, mn = decel_limit, mx = accel_limit)

        # Abhishek - Note:  output of the velocity controller is acceleration value. Need to normalize that for throttle,
        # since we should be sending out 0 - 1 value.

        # Per Pawel, we can use YawController instead of another pid for steering.

        # Timers for the logitudinal controller. Not sure if we can rely on 50Hz rate.
        self.prev_vel_msg_time = 0.0

    # def control(self, *args, **kwargs):
    def control(self, dbw_enabled,  loop_rate, twist_cmd_twist, current_velocity_twist):

        # TODO: Change the arg, kwarg list to suit your needs
        throttle = brake = steering = 0.0

        if dbw_enabled:

            # Velocity controller
            velocity_error_mps =  twist_cmd_twist.linear.x - current_velocity_twist.linear.x
            # Call the step function of the velocity controller to get new control value
            sample_time = 1.0/loop_rate
            throttle = self.velocity_controller.step(velocity_error_mps, sample_time)
            # rospy.loginfo('in twist_controller, throttle = %s'% throttle)

            # If the throttle value is negative, that means we need to brake
            if (throttle<0.0):
                # TODO : Need to convert brake signal value into torque. PID gives deceleration value.
                # Convert that to torque
                brake = -1.0*throttle # m/s2

                # Convert brake value from 0 to max_decel  --> 0 to Max torque
                brake  = brake * self.vehicle_mass * self.wheel_radius

                # Set the throttle to 0, since we are braking.
                throttle = 0.0
            else:
                # Normalize throttle 0-1
                throttle = throttle/self.accel_limit
                # Don't apply brakes when we are accelerating. You are not Ken Block
                brake = 0.0

            # Steering controller - Using the YawController
            steering = self.calculate_steering(twist_cmd_twist, current_velocity_twist)


        else:
            self.velocity_controller.reset()

        # Return throttle, brake, steer
        # return 1., 0., 0.
        return throttle, brake, steering

    def calculate_steering(self, twist_cmd_twist, current_velocity_twist):
        linear_velocity = twist_cmd_twist.linear.x
        angular_velocity = twist_cmd_twist.angular.z
        current_velocity = current_velocity_twist.linear.x
        return self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)

