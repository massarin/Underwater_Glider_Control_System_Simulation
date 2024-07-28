
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


    glider_config = config["glider"]


    print("Setting Up")
    control_system = ControlSystem.ControlSystem(glider_config["control_system"])


    body = Glider.GliderBody(**glider_config["hull"])


    buoyancy_engine = Glider.BuoyancyEngine(**glider_config["buoyancy_engine"])


    hydrofoil = Glider.Hydrofoil(**glider_config["hydrofoil"])


    glider = Glider.Glider(body = body,
                           buoyancy_engine = buoyancy_engine,
                           hydrofoil = hydrofoil,
                           control_system = control_system,
                           initial_position = Vector(**glider_config["initial_position"]),
                           initial_velocity = Vector(**glider_config["initial_velocity"]),
                           initial_acceleration = Vector(**glider_config["initial_acceleration"]),
                           initial_roll = glider_config["initial_roll"],
                           initial_pitch = glider_config["initial_pitch"],
                           initial_yaw = glider_config["initial_yaw"])
    

    sim_config = config["sim"]

    time: float = 0.0

    time_step: float = sim_config["timestep"]

    max_time: float = sim_config["end_time"]


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
    glider_x_components = [row[1].x() for row in glider_log]  # Extracting x components
    glider_y_components = [row[1].y() for row in glider_log]  # Extracting y components
    glider_z_components = [row[1].z() for row in glider_log]  # Extracting z components

    control_time = [row[0] for row in control_log]
    control_vars = list(zip(*control_log))[1:]
    control_labels = ["target depth", "depth pid", "vel pid", "acc pid"]

    # Create a new figure for the x-y plot
    plt.figure()
    plt.plot(glider_x_components, glider_y_components, label="x vs y")
    plt.xlabel("X Component")
    plt.ylabel("Y Component")
    plt.legend(loc="lower left")
    plt.axis('equal')  # Automatic scaling
    plt.title("X vs Y Components")
    plt.grid(True)
    plt.show()

    # Create subplots for the remaining plots
    fig, axs = plt.subplots(2, 1, sharex=True)

    # Plotting z components against time
    axs[0].plot(glider_time, glider_z_components, label="z component")
    axs[0].set_ylabel("Z Component")
    axs[0].legend(loc="lower left")

    # Plotting control variables
    for i, var in enumerate(control_vars):
        axs[1].plot(control_time, var, label=control_labels[i])

    axs[1].set_ylabel("Control")
    axs[1].legend(loc="lower left")

    fig.supxlabel("Time")
    plt.title(f"Glider Dynamics")
    plt.grid(True)

    print("Plot Created")

    print("Showing Plot")
    plt.show()

    print("Plot Exited")
    print("Finished!")




if __name__ == "__main__":
    do_sim()
