import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# Controller parameters for throttle control
KP_THROTTLE = 0.8
KI_THROTTLE = 0.002
KD_THROTTLE = 0.0

# Maximum brake torque
MAX_BRAKE = 400.0


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.vehicle_mass = kwargs["vehicle_mass"]
        self.fuel_capacity = kwargs["fuel_capacity"]
        self.brake_deadband = kwargs["brake_deadband"]
        self.decel_limit = kwargs["decel_limit"]
        self.accel_limit = kwargs["accel_limit"]
        self.wheel_radius = kwargs["wheel_radius"]
        self.wheel_base = kwargs["wheel_base"]
        self.steer_ratio = kwargs["steer_ratio"]
        self.max_lat_accel = kwargs["max_lat_accel"]
        self.max_steer_angle = kwargs["max_steer_angle"]

        # Calculate brake torque
        self.torque_constant = (self.vehicle_mass + self.fuel_capacity*GAS_DENSITY)* self.wheel_radius

        # PID for controlling linear velocity
        self.throttle_ctrl = PID(KP_THROTTLE, KI_THROTTLE, KD_THROTTLE, self.decel_limit, self.accel_limit)

        # Yaw controller
        self.yaw_controller = YawController(wheel_base = self.wheel_base,steer_ratio = self.steer_ratio, min_speed = 0.5, max_lat_accel = self.max_lat_accel, max_steer_angle = self.max_steer_angle)


        # Set-up the low pass filter for velocity
        self.vel_lpf = LowPassFilter(tau = 0.5, ts = 0.02)

        # Get the time stamp
        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, target_vel, target_angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # If twist controller is disabled
        if not dbw_enabled:
            self.throttle_ctrl.reset()
            return 0.0, 0.0, 0.0

        # Check sample time
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        # Difference between target and current velocity
        current_vel = self.vel_lpf.filt(current_vel)
        vel_error = target_vel - current_vel

        # Use PID controller to determine the desired acceleration
        desired_accel = self.throttle_ctrl.step(vel_error, sample_time)
        steering = self.yaw_controller.get_steering(target_vel, target_angular_vel, current_vel)

        # Stop the car at traffic lights
        if target_vel == 0.0 and current_vel < 0.1:
            self.throttle_ctrl.reset()
            throttle = 0.0
            brake = MAX_BRAKE
        # To slow down
        elif desired_accel < 0.1 and vel_error < 0:
            throttle = 0.0
            decel = max(vel_error, self.decel_limit)
            brake = min(MAX_BRAKE, (abs(decel) * self.torque_constant))
        # To speed up
        else: 
            throttle = desired_accel
            brake = 0.0

        # Return values
        return throttle, brake, steering
