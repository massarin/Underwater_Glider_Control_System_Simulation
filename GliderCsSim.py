
import matplotlib.pyplot as plt

import numpy as np

import sys

import Glider
import ControlSystem
import SimMath
from SimMath import Vector



def do_sim() -> None:
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

    control_system = ControlSystem.ControlSystem(config_path)

    glider = Glider.Glider(body = body, buoyancy_engine = buoyancy_engine, control_system = control_system,\
                initial_position = Vector(), initial_velocity = Vector(), initial_acceleration = Vector())
    

    time: float = 0

    time_step: float = 0.01

    max_time: float = 1000



    while time < max_time:

        glider.sim_timestep(time)

        time += time_step

    
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
    plt.show()

    # plt.xlabel("Time")
    # plt.title(f"acc kp: {glider.control_system.pid_v_acc.kp},  ki: {glider.control_system.pid_v_acc.ki},  kd: {glider.control_system.pid_v_acc.kd}")
    # plt.legend(loc="lower left")
    # plt.grid(True)
    # plt.show()




if __name__ == "__main__":
    do_sim()
