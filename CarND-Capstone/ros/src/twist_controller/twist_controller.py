from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# some initial values randoms
K_p = 0.2
K_i = 0.003
K_d = 3.0

class Controller(object):
    def __init__(self,
                 wheel_base,
                 steer_ratio,
                 min_speed,
                 max_lat_accel,
                 max_steer_angle,
                 accel_limit,
                 decel_limit,
                 wheel_radius,
                 vehicle_mass,
                 fuel_capacity):
        # TODO: Implement
        self.set_total_mass(vehicle_mass, fuel_capacity)
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.wheel_radius = wheel_radius
        # CTE and delta_t needed for PID
        self.pid = PID(K_p, K_i, K_d, mn=decel_limit, mx=accel_limit)
        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.

    def torque(self, acceleration):
        return self.total_mass() * acceleration * self.wheel_radius

    def set_total_mass(self, vehicle_mass, fuel_capacity):
        self.total_mass = GAS_DENSITY * fuel_capacity + vehicle_mass
