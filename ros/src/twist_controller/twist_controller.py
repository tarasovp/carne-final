import time

from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.params = kwargs
        
        self.yaw_controller = YawController(self.params['wheel_base'], 
         									self.params['steer_ratio'], 
         									1, 
         									self.params['max_lat_accel'], 
         									self.params['max_steer_angle'])
        self.t=None
        self.throttle_controller = PID(0.5,0.5,0.5)

    def control(self, velocity, steering, twist):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        
        if self.t is None:
            self.t = time.time()
            return 0.0, 0.0, 0.0

        dt = time.time() - self.t
        self.t = time.time()

        error_v = min(twist.linear.x, 50*ONE_MPH) - velocity.linear.x
        #error_v = max(self.decel_limit*dt, min(self.accel_limit*dt, error_v))
        
        throttle = self.throttle_controller.step(error_v, dt)
        throttle = max(0.0, min(1.0, throttle))
       
        brake = 0
        steer = self.yaw_controller.get_steering(twist.linear.x, twist.angular.z, velocity.linear.x)
        #steer = self.filter.filt(steer)
        return throttle, brake, steer
