
from ControlSystem import ControlSystem

import SimMath
from SimMath import Vector

import Inlet



class GliderBody:

    def __init__(self, mass: float, volume: float, drag_coefficient: float) -> None:

        self.mass: float = mass
        self.volume: float = volume
        self.drag_coefficient: float = drag_coefficient

        self.max_force: float = 10

    

    def compute_acceleration(self, applied_force: Vector) -> Vector:

        # Limit force
        mag = applied_force.magnitude()
        if mag > self.max_force:
            applied_force *= self.max_force / mag


        return applied_force / self.mass
    


    def compute_drag_force(self, velocity: Vector) -> Vector:

        # TODO: Compute projected area
        area = 0.1
        
        
        # Drag equation
        return -velocity.normalized() * (0.5 * Inlet.density * self.drag_coefficient * area * velocity.dot(velocity))


    
    def compute_buoyancy_force(self) -> Vector:

        return Inlet.gravity * (-1 * Inlet.density * self.volume)
    


    def compute_gravity_force(self) -> Vector:

        return Inlet.gravity * self.mass





class BuoyancyEngine:

    def __init__(self, tank_volume: float, pump_rate: float, proportion_full: float) -> None:
        
        self.tank_volume: float = tank_volume
        self.pump_rate: float = pump_rate
        self.proportion_full: float = proportion_full
    


    def compute_tank_change(self, time_step: float) -> None:

        self.proportion_full = SimMath.clamp(self.proportion_full + (self.pump_rate * time_step), 0, 1)


    
    def compute_buoyancy_force(self) -> Vector:

        buoyancy_volume = self.tank_volume * (1 - self.proportion_full)

        return Inlet.gravity * (-1 * Inlet.density * buoyancy_volume)
    


    def compute_gravity_force(self) -> Vector:

        gravity_volume = self.tank_volume * self.proportion_full

        return Inlet.gravity * (Inlet.density * gravity_volume)






class Glider:

    def __init__(self, body: GliderBody, buoyancy_engine: BuoyancyEngine, control_system: ControlSystem,
                  initial_position: Vector, initial_velocity: Vector, initial_acceleration: Vector) -> None:
        
        self.body: GliderBody = body

        self.buoyancy_engine: BuoyancyEngine = buoyancy_engine

        self.control_system: ControlSystem = control_system

        self.position: Vector = initial_position
        self.velocity: Vector = initial_velocity
        self.acceleration: Vector = initial_acceleration

        self.time: float = 0



    def integrate_forces(self, time_step: float) -> None:

        total_buoyancy = self.body.compute_buoyancy_force() + self.buoyancy_engine.compute_buoyancy_force()

        total_drag = self.body.compute_drag_force(self.velocity)

        total_gravity = self.body.compute_gravity_force() + self.buoyancy_engine.compute_gravity_force()

        total_force = total_buoyancy + total_drag + total_gravity


        self.acceleration = self.body.compute_acceleration(total_force)
        self.velocity += self.acceleration * time_step
        self.position += self.velocity * time_step

    
    
    def sim_timestep(self, time: float) -> None:

        time_step = time - self.time

        self.time = time

        command = self.control_system.calc_acc(self.position, self.velocity, self.acceleration,
                                               self.buoyancy_engine.proportion_full, time)

        self.buoyancy_engine.pump_rate = command

        self.buoyancy_engine.compute_tank_change(time_step)

        self.integrate_forces(time_step)

        # The Glider cannot leave the water
        if self.position.z() > 0:
            self.position.z(-0.1)

            if self.velocity.z() > 0:
                self.velocity.z(0)
            
            if self.acceleration.z() > 0:
                self.acceleration.z(0)






