
import typing

import numpy as np

import SimMath
from SimMath import Vector



"""
This is the Glider's control system module.

It provides a PID controller class, a state machine class, and logging.
"""



class Logger:

    def __init__(self) -> None:

        self.glider_log: list = []

        self.control_log: list = []





class PIDController:
    """
    Simple PID controller implementation.

    Includes derivative kickback, integral windup, and output protections.
    """
    
    def __init__(self, kp: float = 1, ki: float = 0, kd: float = 0,
                 i_limit: float = 1_000, output_limit: float = 1_000) -> None:
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
        self.kp: float = kp
        self.ki: float = ki
        self.kd: float = kd

        # Integral windup limit
        self.i_limit: float = i_limit

        # Output limit
        self.output_limit: float = output_limit

        # Previous values
        self.prev_error: float = 0
        self.prev_input: float = 0
        self.prev_time: float | None = None

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
        if self.prev_time is None:
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
        derivative = (error - self.prev_error) / time_delta

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

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

        self.frequency: int = 10

        self.period: float = 1.0 / self.frequency

        self.time: float = 0

        self.prev_update_time: float = self.time

        self.prev_command: float = 0



        # Create cascading PID controllers

        # Targets a depth
        self.pid_depth: PIDController = PIDController(1, 0, 0, 100, 100)

        # Targets a vertical speed
        self.pid_v_vel: PIDController = PIDController(1, 0, 0, 100, 10)

        # Targets a vertical acceleration
        self.pid_v_acc: PIDController = PIDController(0.02, 0.00033, 0.8, 100, 0.01)


        # Glide path parameters
        self.min_depth: float = -20
        self.max_depth: float = -70
        self.target_depth: float = self.min_depth

        # Logging
        self.logger = Logger()


    
    def calc_acc(self, position: Vector, velocity: Vector, acceleration: Vector, tank: float, time: float) -> float:

        self.time = time

        self.logger.glider_log.append([time, position.z(), velocity.z(), acceleration.z(), (tank - 1) * 10])

        if time < self.prev_update_time + self.period:
            return self.prev_command
        

        self.prev_update_time = time


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


        self.logger.control_log.append([time, self.target_depth, pid_depth_output, pid_v_vel_output, pid_v_acc_output])

        command = -pid_v_acc_output

        self.prev_command = command

        return command
