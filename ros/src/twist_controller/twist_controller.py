import rospy
import os 

from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter


GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MIN_SPEED = 0


class BrakeController:
    def __init__(self, decel_limit, vehicle_mass, wheel_radius, brake_deadband):
        self.decel_limit = decel_limit
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.brake_deadband = brake_deadband

    def get_brake(self, velocity_error):
        decel = abs(max(velocity_error, self.decel_limit))
        if decel < self.brake_deadband: 
            decel = 0
        return decel * self.vehicle_mass * self.wheel_radius


class Controller(object):
    def __init__(self, wheel_base, steer_ratio,
                 max_lat_accel, max_steer_angle, decel_limit, 
                 vehicle_mass, wheel_radius, brake_deadband):
        self.brake_controller = BrakeController(decel_limit, vehicle_mass, wheel_radius, brake_deadband)        
        self.yaw_controller = YawController(wheel_base, steer_ratio, MIN_SPEED, max_lat_accel, max_steer_angle)
        self.vel_filter = LowPassFilter(tau=0.5, ts=0.02)
        self.throttle_pid = PID(kp=0.5, ki=0.01, kd=0.2, mn=-0.8, mx=0.8)

        self.last_time = rospy.get_time()
        self.is_enabled = False
        self.velocities = []

    def control(self, lin_velocity, angular_velocity, cur_velocity):
        if not self.is_enabled:
            self.throttle_pid.reset()
            return 0, 0, 0

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        cur_velocity = self.vel_filter.filt(cur_velocity)
        velocity_error = lin_velocity - cur_velocity
        steer = self.yaw_controller.get_steering(lin_velocity, angular_velocity, cur_velocity)
        throttle = self.throttle_pid.step(velocity_error, sample_time)
        self.velocities.append([lin_velocity, cur_velocity, velocity_error])

        brake = 0.0
        if lin_velocity == 0.0 and cur_velocity < 1:
            throttle = 0
            brake = 400
        elif throttle < 0:
            throttle = 0
            brake = self.brake_controller.get_brake(velocity_error)

        return throttle, brake, steer
    def dump(self, dump_dir):
        dump_file = os.path.join(dump_dir, 'velocities.csv')
        with open(dump_file, 'w') as f:
            for v in self.velocities:
                f.write("%f;%f;%f\n" % tuple(v))
