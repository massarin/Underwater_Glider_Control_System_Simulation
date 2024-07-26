
from ControlSystem import ControlSystem

import SimMath
from SimMath import Vector

import Inlet



class GliderBody:

    def __init__(self, mass: float = 27, drag_coefficient: float = 0.3) -> None:
        self.mass = mass
        self.drag_coefficient = drag_coefficient

        self.max_force = 5

    

    def compute_acceleration(self, applied_force: Vector) -> Vector:

        # Limit force
        mag = applied_force.magnitude()
        if mag > self.max_force:
            applied_force *= self.max_force / mag


        return applied_force / self.mass
    


    def compute_drag(self, velocity: Vector) -> Vector:

        # TODO: Compute projected area
        area = 0.1
        
        
        # Drag equation
        return velocity.normalized() * (-1 * 0.5 * Inlet.density * velocity.dot(velocity) * self.drag_coefficient * area)




        





class Glider:

    def __init__(self, body: GliderBody, control_system: ControlSystem, \
                  initial_position: Vector, initial_velocity: Vector, initial_acceleration: Vector) -> None:
        
        self.body: GliderBody = body

        self.control_system :ControlSystem = control_system

        self.position: Vector = initial_position
        self.velocity: Vector = initial_velocity
        self.acceleration: Vector = initial_acceleration

        self.time: float = 0

    
    
    def sim_timestep(self, time: float) -> None:

        time_delta = time - self.time

        self.time = time

        control_force = self.control_system.calc_acc(self.position, self.velocity, self.acceleration, time)

        total_force = control_force + self.body.compute_drag(self.velocity)

        self.acceleration = self.body.compute_acceleration(total_force)
        self.velocity += self.acceleration * time_delta
        self.position += self.velocity * time_delta

        # The Glider cannot leave the water
        if self.position.z() > 0:
            print(self.position.z())
            self.position.z(-0.1)
            print(self.position.z())

            if self.velocity.z() > 0:
                self.velocity.z(0)
            
            if self.acceleration.z() > 0:
                self.acceleration.z(0)






