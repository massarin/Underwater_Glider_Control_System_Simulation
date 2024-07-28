
from ControlSystem import ControlSystem

import SimMath
from SimMath import Vector

import Inlet

import numpy as np



class GliderBody:
    """
    Represents the body of a glider. It is a capsule.

    Attributes:
        mass (float): The mass of the glider body.
        volume (float): The volume of the glider body.
        drag_multiplier (float): The drag coefficient of the glider body.
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

    def __init__(self, mass: float, length: float, radius: float, drag_multiplier: float) -> None:
        self.mass: float = mass

        self.length: float = length
        self.radius: float = radius

        # Volume of the cylinder + volume of the two hemispheres
        self.volume: float = (SimMath.pi * length * radius * radius) +\
            ((4.0 / 3.0) * SimMath.pi * radius * radius * radius)
        

        self.drag_multiplier: float = drag_multiplier
        self.max_force: float = 10.0


        # These are constants that speed up drag calculations
        self.__end_cap_proj_area: float = SimMath.pi * self.radius * self.radius
        self.__perp_cylinder_proj_area: float = 2.0 * self.radius * self.length


        self.front_drag_coefficient = 0.4
        self.side_drag_coefficient = 1.2



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



    def compute_drag_force(self, velocity: Vector, roll: float, pitch: float, yaw: float) -> Vector:
        """
        Computes the drag force acting on the glider body based on its velocity.

        Args:
            velocity (Vector): The velocity of the glider body.

        Returns:
            Vector: The drag force acting on the glider body.
        """

        direction = SimMath.euler_to_direction(roll, pitch, yaw)

        # Area of the shadow of a capsule
        cosine_angle = abs(velocity.normalized().dot(direction))

        area = self.__end_cap_proj_area + self.__perp_cylinder_proj_area * cosine_angle


        drag_coef = self.drag_multiplier *\
                SimMath.lerp(self.front_drag_coefficient, self.side_drag_coefficient, cosine_angle)


        # Drag equation
        return -velocity.normalized() * (0.5 * Inlet.density * area * drag_coef * velocity.dot(velocity))



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

    def __init__(self, tank_volume: float, initial_proportion_full: float,
                  initial_pump_rate: float, max_pump_rate: float) -> None:
    
        self.tank_volume: float = tank_volume
        self.proportion_full: float = initial_proportion_full

        self.pump_rate: float = initial_pump_rate
        self.max_pump_rate: float = max_pump_rate



    def compute_tank_change(self, time_step: float) -> None:
        """
        Computes the change in tank proportion based on the pump rate and time step.

        Args:
            time_step (float): The time step for the computation.
        """

        self.pump_rate = SimMath.clamp_mag(self.pump_rate, self.max_pump_rate)
        
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





class Hydrofoil:

    def __init__(self, reference_area: float, lift_multiplier: float,
                 lift_curve_slope: float, stall_angle: float) -> None:
        
        self.reference_area: float = reference_area

        self.lift_multiplier: float = lift_multiplier

        self.lift_curve_slope: float = lift_curve_slope

        self.stall_angle: float = stall_angle

    

    def compute_lift_coefficient(self, angle_of_attack: float) -> float:

        lift_coefficient = 0.0

        angle = abs(angle_of_attack)

        if angle < self.stall_angle:
            lift_coefficient = self.lift_curve_slope * angle
        
        elif angle < 0.7854: # pi / 4
            lift_coefficient = SimMath.lerp(self.lift_curve_slope * self.stall_angle, 0.0, angle - self.stall_angle)
        
        return SimMath.sign(angle_of_attack) * lift_coefficient * self.lift_multiplier
    


    def compute_lift_force(self, velocity: Vector, roll: float, pitch: float, yaw: float) -> Vector:

        rotation_matrix = SimMath.euler_to_rotation_matrix(roll, pitch, yaw)

        velocity_body = np.dot(rotation_matrix.T, velocity.vec)

        dynamic_pressure = 0.5 * Inlet.density * np.dot(velocity_body, velocity_body)

        angle_of_attack = np.arctan2(velocity_body[2], velocity_body[0])

        lift_coefficient = self.compute_lift_coefficient(angle_of_attack)

        lift_force_body = np.array([0, lift_coefficient * dynamic_pressure * self.reference_area, 0])

        lift_force_world = np.dot(rotation_matrix, lift_force_body)

        lift = Vector(lift_force_world[0], lift_force_world[1], lift_force_world[2])

        return lift






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
        orientation (Vector): The direction the glider is pointing.
        time (float): The current time of the glider simulation.

    Methods:
        integrate_forces(time_step: float) -> None:
            Integrates the forces acting on the glider over a given time step.

        sim_timestep(time: float) -> None:
            Simulates a time step for the glider.

    """

    def __init__(self, body: GliderBody, buoyancy_engine: BuoyancyEngine, hydrofoil: Hydrofoil,
                 control_system: ControlSystem,
                 initial_position: Vector, initial_velocity: Vector, initial_acceleration: Vector,
                 initial_roll: float, initial_pitch: float, initial_yaw: float) -> None:
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

        self.hydrofoil: Hydrofoil = hydrofoil

        self.control_system: ControlSystem = control_system

        self.position: Vector = initial_position
        self.velocity: Vector = initial_velocity
        self.acceleration: Vector = initial_acceleration
        
        self.roll: float = initial_roll
        self.pitch: float = initial_pitch
        self.yaw: float = initial_yaw

        self.time: float = 0.0



    def integrate_forces(self, time_step: float) -> None:
        """
        Integrates the forces acting on the glider over a given time step.

        Args:
            time_step (float): The time step for the integration.
        """

        total_buoyancy = self.body.compute_buoyancy_force() + self.buoyancy_engine.compute_buoyancy_force()

        total_drag = self.body.compute_drag_force(self.velocity, self.roll, self.pitch, self.yaw)

        total_gravity = self.body.compute_gravity_force() + self.buoyancy_engine.compute_gravity_force()

        total_lift = self.hydrofoil.compute_lift_force(self.velocity, self.roll, self.pitch, self.yaw)

        total_force = total_buoyancy + total_drag + total_gravity + total_lift


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

        # It cannot go too deep either
        if self.position.z() < -200:
            self.position.z(-199.9)

            if self.velocity.z() < 0:
                self.velocity.z(0)
            
            if self.acceleration.z() < 0:
                self.acceleration.z(0)
