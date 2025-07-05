import random
import math
import time

class PhysicsEngine:
    gravity = 9.8 #m/s^2
    damping_coefficient = 1.25
    offset_bias = 1

    @staticmethod

    def find_acceleration(thrust, mass, velocity):
        weight = mass * PhysicsEngine.gravity
        drag = PhysicsEngine.damping_coefficient * velocity * abs(velocity)

        net_force = thrust - weight - drag - PhysicsEngine.offset_bias

        acceleration = net_force / mass
        
        return acceleration
    
    @staticmethod

    def find_velocity(velocity, acceleration):
        
        velocity += acceleration * PID.dt

        return velocity

    @staticmethod
    def find_position(position, velocity):

        position += velocity * PID.dt

        return position
    
class PID:

    dt = 0.01
    target = 10

    def __init__(self, kp, ki, kd):
        self.count = 0
        self.max_amplitude = 0

        self.mass = 1.0 #kg
        self.position = 0.0 #m
        self.velocity = 0.0 #m/s
        self.acceleration = 0.0 #m/s^2
        self.base_force = self.mass * PhysicsEngine.gravity #N
        self.thrust = 0.0 #N

        self.alpha = 0.25

        self.current = 0.0
        self.previous_current = 0.0
        self.error = 0.0
        self.previous_error = 0.0

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral = 0.0
        self.derivative = 0.0
        self.previous_error = 0.0
        self.MAX_INTEGRAL = 100.0

    def calculate_integral(self):
        self.integral += self.error * PID.dt

        if self.integral > self.MAX_INTEGRAL:
            self.integral = self.MAX_INTEGRAL
        elif self.integral < -self.MAX_INTEGRAL:
            self.integral = -self.MAX_INTEGRAL

    def calculate_derivative(self):
        self.derivative = (self.error - self.previous_error) / PID.dt

    @staticmethod
    
    def noise():
        return random.uniform(-0.1, 0.1)
    
    def smooth(self, input_val):
        return self.alpha * input_val + (1 - self.alpha) * self.previous_current
    
    def run_PID(self):
        self.count += 1

        self.acceleration = PhysicsEngine.find_acceleration(self.thrust, self.mass, self.velocity)
        self.velocity = PhysicsEngine.find_velocity(self.velocity, self.acceleration)
        self.position = PhysicsEngine.find_position(self.position, self.velocity)

        self.current = self.position + PID.noise()

        self.current = self.smooth(self.current)

        self.error = self.target - self.current

        self.calculate_integral()
        self.calculate_derivative()

        self.thrust = self.base_force + self.kp * self.error + self.ki * self.integral + self.kd * self.derivative

        self.previous_current = self.current
        self.previous_error = self.error

        return self.current

def main():
    p = PID(1, 0, 0)
    pd = PID(1, 0, 1)
    pid = PID(1, 1, 1)

    i = 0
    while True:
        print(f"Count: {p.count} P: {p.run_PID():.3f} PD: {pd.run_PID():.3f} PID: {pid.run_PID():.3f} Target: {PID.target:.3f}")
        
        PID.target = 10 + math.sin(i * PID.dt)

        time.sleep(0.001)
        i += 1


if __name__ == "__main__":
    main()

    
