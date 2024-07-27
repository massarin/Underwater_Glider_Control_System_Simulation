
from ControlSystem import ControlSystem

import SimMath
from SimMath import Vector

import Inlet



class GliderBody:
    """
    Represents the body of a glider.

    Attributes:
        mass (float): The mass of the glider body.
        volume (float): The volume of the glider body.
        drag_coefficient (float): The drag coefficient of the glider body.
        max_force (float): The maximum force that can be applied to the glider body.

    Methods:
        compute_acceleration(applied_force: Vector) -> Vector:
            Computes the acceleration of the glider body based on the applied force.
        compute_drag_force(velocity: Vector) -> Vector:
            Computes the drag force acting on the glider body based on its velocity.
        compute_buoyancy_force() -> Vector:
            Computes the buoyancy force acting on the glider body.
        compute_gravity_force() -> Vector:
            Computes the gravity force acting on the glider body.
    """

    def __init__(self, mass: float, volume: float, drag_coefficient: float) -> None:
        self.mass: float = mass
        self.volume: float = volume
        self.drag_coefficient: float = drag_coefficient
        self.max_force: float = 10



    def compute_acceleration(self, applied_force: Vector) -> Vector:
        """
        Computes the acceleration of the glider body based on the applied force.

        Args:
            applied_force (Vector): The force applied to the glider body.

        Returns:
            Vector: The acceleration of the glider body.
        """

        # Limit force
        mag = applied_force.magnitude()
        if mag > self.max_force:
            applied_force *= self.max_force / mag

        return applied_force / self.mass



    def compute_drag_force(self, velocity: Vector) -> Vector:
        """
        Computes the drag force acting on the glider body based on its velocity.

        Args:
            velocity (Vector): The velocity of the glider body.

        Returns:
            Vector: The drag force acting on the glider body.
        """

        # TODO: Compute projected area
        area = 0.1

        # Drag equation
        return -velocity.normalized() * (0.5 * Inlet.density * self.drag_coefficient * area * velocity.dot(velocity))



    def compute_buoyancy_force(self) -> Vector:
        """
        Computes the buoyancy force acting on the glider body.

        Returns:
            Vector: The buoyancy force acting on the glider body.
        """

        return Inlet.gravity * (-1 * Inlet.density * self.volume)



    def compute_gravity_force(self) -> Vector:
        """
        Computes the gravity force acting on the glider body.

        Returns:
            Vector: The gravity force acting on the glider body.
        """

        return Inlet.gravity * self.mass





class BuoyancyEngine:
    """
    The BuoyancyEngine class represents an engine that calculates buoyancy and gravity forces for a tank.

    Attributes:
        tank_volume (float): The volume of the tank.
        pump_rate (float): The rate at which the tank is being pumped.
        proportion_full (float): The proportion of the tank that is filled.

    Methods:
        compute_tank_change(time_step: float) -> None:
            Computes the change in tank proportion based on the pump rate and time step.

        compute_buoyancy_force() -> Vector:
            Computes the buoyancy force based on the tank volume and proportion full.

        compute_gravity_force() -> Vector:
            Computes the gravity force based on the tank volume and proportion full.
    """

    def __init__(self, tank_volume: float, pump_rate: float, proportion_full: float) -> None:
        self.tank_volume: float = tank_volume
        self.pump_rate: float = pump_rate
        self.proportion_full: float = proportion_full



    def compute_tank_change(self, time_step: float) -> None:
        """
        Computes the change in tank proportion based on the pump rate and time step.

        Args:
            time_step (float): The time step for the computation.
        """
        
        self.proportion_full = SimMath.clamp(self.proportion_full + (self.pump_rate * time_step), 0, 1)



    def compute_buoyancy_force(self) -> Vector:
        """
        Computes the buoyancy force based on the tank volume and proportion full.

        Returns:
            Vector: The computed buoyancy force.
        """

        buoyancy_volume = self.tank_volume * (1 - self.proportion_full)
        return Inlet.gravity * (-1 * Inlet.density * buoyancy_volume)



    def compute_gravity_force(self) -> Vector:
        """
        Computes the gravity force based on the tank volume and proportion full.

        Returns:
            Vector: The computed gravity force.

        """

        gravity_volume = self.tank_volume * self.proportion_full
        return Inlet.gravity * (Inlet.density * gravity_volume)






class Glider:
    """
    Represents a glider object.

    Attributes:
        body (GliderBody): The body of the glider.
        buoyancy_engine (BuoyancyEngine): The buoyancy engine of the glider.
        control_system (ControlSystem): The control system of the glider.
        position (Vector): The position of the glider.
        velocity (Vector): The velocity of the glider.
        acceleration (Vector): The acceleration of the glider.
        time (float): The current time of the glider simulation.

    Methods:
        integrate_forces(time_step: float) -> None:
            Integrates the forces acting on the glider over a given time step.

        sim_timestep(time: float) -> None:
            Simulates a time step for the glider.

    """

    def __init__(self, body: GliderBody, buoyancy_engine: BuoyancyEngine, control_system: ControlSystem,
                  initial_position: Vector, initial_velocity: Vector, initial_acceleration: Vector) -> None:
        """
        Initializes a glider object.

        Args:
            body (GliderBody): The body of the glider.
            buoyancy_engine (BuoyancyEngine): The buoyancy engine of the glider.
            control_system (ControlSystem): The control system of the glider.
            initial_position (Vector): The initial position of the glider.
            initial_velocity (Vector): The initial velocity of the glider.
            initial_acceleration (Vector): The initial acceleration of the glider.
        """
        
        self.body: GliderBody = body

        self.buoyancy_engine: BuoyancyEngine = buoyancy_engine

        self.control_system: ControlSystem = control_system

        self.position: Vector = initial_position
        self.velocity: Vector = initial_velocity
        self.acceleration: Vector = initial_acceleration

        self.time: float = 0



    def integrate_forces(self, time_step: float) -> None:
        """
        Integrates the forces acting on the glider over a given time step.

        Args:
            time_step (float): The time step for the integration.
        """

        total_buoyancy = self.body.compute_buoyancy_force() + self.buoyancy_engine.compute_buoyancy_force()

        total_drag = self.body.compute_drag_force(self.velocity)

        total_gravity = self.body.compute_gravity_force() + self.buoyancy_engine.compute_gravity_force()

        total_force = total_buoyancy + total_drag + total_gravity


        self.acceleration = self.body.compute_acceleration(total_force)
        self.velocity += self.acceleration * time_step
        self.position += self.velocity * time_step

    
    
    def sim_timestep(self, time: float) -> None:
        """
        Simulates a time step for the glider.

        Args:
            time (float): The time for the simulation.
        """

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
