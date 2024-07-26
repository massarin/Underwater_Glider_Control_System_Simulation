
import typing

import SimMath
from SimMath import Vector



"""
This is the Glider's control system module.

It provides a PID controller class, a state machine class, and logging.
"""





class PIDController:
    """
    Simple PID controller implementation.

    Includes derivative kickback, integral windup, and output protections.
    """
    
    def __init__(self, kp: float = 1, ki: float = 0, kd: float = 0, i_limit: float = 1_000, output_limit: float = 1_000) -> None:
        """
        Initialize the PID controller.

        Args:
            kp (float): Proportional gain (default: 1.0).
            ki (float): Integral gain (default: 0.0).
            kd (float): Derivative gain (default: 0.0).
            i_limit (float): Integral limit (default: 1_000.0).
            output_limit (float): Output limit (default: 1_000.0).
        """

        # Tuning parameters
        self.Kp: float = kp
        self.Ki: float = ki
        self.Kd: float = kd

        # Integral windup limit
        self.i_limit: float = i_limit

        # Output limit
        self.output_limit: float = output_limit

        # Previous values
        self.prev_error: float = 0
        self.prev_input: float = 0
        self.prev_time: float = 0

        # Integral term
        self.integral: float = 0



    def update(self, target: float, input: float, time: float) -> float:
        """
        Updates the control system based on the target, input, and current time.

        Args:
            target (float): The desired target value.
            input (float): The current input value.
            time (float): The current time.

        Returns:
            float: The output value of the control system.

        Raises:
            ValueError: If the current time is not after the previous time.
        """

        error = target - input

        # Don't return anything on first call
        if time == 0:
            self.prev_error = error
            self.prev_time = time
            self.prev_input = input
            return 0

        time_delta = time - self.prev_time

        if time_delta <= 0:
            raise ValueError("time must be larger than prev time")
        

        # Integral windup prevention
        self.integral = SimMath.clamp_mag(self.integral + (error * time_delta), self.i_limit)

        # Derivative kickback prevention
        derivative = (input - self.prev_input) / time_delta

        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # Store previous values
        self.prev_error = error
        self.prev_time = time
        self.prev_input = input

        return SimMath.clamp_mag(output, self.output_limit)





"""
Typedef for the StateMachine class

TODO: This may need to be a full class
"""
State = int
StateGraph = typing.Dict[State, typing.List[State]]
diving = State(0)
surfacing = State(1)




class StateMachine:
    """
    A class representing a state machine.

    TODO: Very limited, rules for state transition are not defined

    Attributes:
        state_graph (dict[State, list[State]]): A dictionary representing the state graph.
        initial_state (State): The initial state of the state machine.
        state (State): The current state of the state machine.
    """

    def __init__(self, state_graph: StateGraph, initial_state: State) -> None:
        """
        Initializes a new instance of the StateMachine class.

        Args:
            state_graph (dict[State, list[State]]): A dictionary representing the state graph.
            initial_state (State): The initial state of the state machine.
        """

        self.state: State = initial_state
        self.state_graph: StateGraph = state_graph
    

    def next(self) -> None:
        """
        Moves the state machine to the next state.
        """

        self.state = self.state_graph[self.state][0]





class ControlSystem:
    """
    This is all hardcoded for now
    """

    def __init__(self) -> None:

        # Initialize state machine
        state_graph: StateGraph = {
            diving : [surfacing],
            surfacing : [diving]
        }

        self.state_machine: StateMachine = StateMachine(state_graph, diving)



        # Create cascading PID controllers

        # Targets a depth
        self.pid_depth: PIDController = PIDController(0.1, 0, 0, 10)

        # Targets a vertical speed
        self.pid_v_vel: PIDController = PIDController(0.1, 0, 0, 100)

        # Targets a vertical acceleration
        self.pid_v_acc: PIDController = PIDController(0.1, 0, 0, 10, 100)


        # Glide path parameters

        self.min_depth: float = -20
        self.max_depth: float = -70
        self.target_depth: float = self.min_depth


    
    def calc_acc(self, position: Vector, velocity: Vector, acceleration: Vector, time: float) -> Vector:

        # Swap states when needed
        # TODO: There needs to be a better way to do this (the state machine should do it with a single method call)
        if self.state_machine.state == diving:
            if position.z() <= self.target_depth:
                self.target_depth = self.min_depth
                self.state_machine.next()
        else:
            if position.z() >= self.target_depth:
                self.target_depth = self.max_depth
                self.state_machine.next()

            


        # depth -> v_vel -> v_acc
        pid_depth_output = self.pid_depth.update(self.target_depth, position.z(), time)
        pid_v_vel_output = self.pid_v_vel.update(pid_depth_output, velocity.z(), time)
        pid_v_acc_output = self.pid_v_acc.update(pid_v_vel_output, acceleration.z(), time)



        
        return Vector(0.0, 0.0, pid_v_acc_output)
