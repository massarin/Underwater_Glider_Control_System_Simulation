
import matplotlib.pyplot as plt

import numpy as np

import json

import sys

import Glider
import ControlSystem
import SimMath
from SimMath import Vector





    
def load_config(file_path: str) -> dict:
    """
    Load a configuration file and return the configuration as a dictionary.

    Args:
        file_path (str): The path to the configuration file.

    Returns:
        dict: The configuration loaded from the file.
    """
    
    with open(file_path, 'r') as file:
        config = json.load(file)
    
    return config





def do_sim() -> None:
    """
    Runs the glider simulation.

    This function sets up the glider simulation by initializing the glider body, buoyancy engine,
    control system, and other necessary parameters. It then runs the simulation for a specified
    duration of time, updating the glider's position, velocity, and acceleration at each time step.
    Finally, it creates a plot of the glider's variables and displays it.

    It is one big function that does too many things
    TODO: It too big

    Args:
        None

    Returns:
        None
    """

    body = Glider.GliderBody(mass = 28.9, volume = 0.03, drag_coefficient = 0.50)

    buoyancy_engine = Glider.BuoyancyEngine(tank_volume = 0.003, pump_rate = 0, proportion_full = 0)


    # Get the config path
    config_path = None
    for arg in sys.argv:
        if arg.endswith(".json"):
            config_path = arg

    if config_path is None:
        print(f"Usage: {"".join(sys.argv)} /path/to/config.json")

        while config_path is None:
            arg = input("Please enter config path: ")

            if arg.endswith(".json"):
                config_path = arg

    
    config = load_config(config_path)


    print("Setting Up")
    control_system = ControlSystem.ControlSystem(config)

    glider = Glider.Glider(body = body, buoyancy_engine = buoyancy_engine, control_system = control_system,\
                initial_position = Vector(), initial_velocity = Vector(), initial_acceleration = Vector())
    

    time: float = 0.0

    time_step: float = config["sim_timestep"]

    max_time: float = config["sim_end_time"]


    print("Setup Done")
    print("Running Simulation")
    while time < max_time:

        glider.sim_timestep(time)

        time += time_step

    print("Simulation Done")

    print("Creating Plot")
    
    glider_log = np.array(glider.control_system.logger.glider_log)
    control_log = np.array(glider.control_system.logger.control_log)

    glider_time = [row[0] for row in glider_log]
    glider_vars = list(zip(*(glider_log)))[1:]
    glider_labels = ["position", "velocity", "acceleration", "tank"]

    control_time = [row[0] for row in control_log]
    control_vars = list(zip(*control_log))[1:]
    control_labels = ["target depth", "depth pid", "vel pid", "acc pid"]


    fig, axs = plt.subplots(2, 1, sharex=True)

    for i, var in enumerate(glider_vars):
        axs[0].plot(glider_time, var, label=glider_labels[i])

    for i, var in enumerate(control_vars):
        axs[1].plot(control_time, var, label=control_labels[i])

    axs[0].set_ylabel("Glider")
    axs[0].legend(loc="lower left")
    
    axs[1].set_ylabel("Control")
    axs[1].legend(loc="lower left")

    fig.supxlabel("Time")
    plt.title(f"acc kp: {glider.control_system.pid_v_acc.kp},  ki: {glider.control_system.pid_v_acc.ki},  kd: {glider.control_system.pid_v_acc.kd}")
    plt.grid(True)

    print("Plot Created")

    print("Showing Plot")
    plt.show()

    print("Plot Exited")
    print("Finished!")




if __name__ == "__main__":
    do_sim()
