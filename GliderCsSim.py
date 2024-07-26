
import matplotlib.pyplot as plt

import numpy as np

import Glider
import ControlSystem
import SimMath
from SimMath import Vector



def do_sim() -> None:
    body = Glider.GliderBody(mass = 27.0, drag_coefficient = 0.03)

    control_system = ControlSystem.ControlSystem()

    glider = Glider.Glider(body = body, control_system = control_system,\
                initial_position = Vector(), initial_velocity = Vector(), initial_acceleration = Vector())
    

    time: float = 0

    time_step: float = 0.05

    max_time: float = 10000


    while time < max_time:

        glider.sim_timestep(time)

        # print(f"time: {time} \t depth: {-glider.position.z()}")

        time += time_step

    
    glider_log = np.array(glider.control_system.logger.glider_log)
    control_log = np.array(glider.control_system.logger.control_log)

    glider_time = [row[0] for row in glider_log]
    glider_vars = list(zip(*(glider_log)))[1:]
    glider_labels = ["position", "velocity", "acceleration"]

    control_time = [row[0] for row in control_log]
    control_vars = list(zip(*control_log))[1:]
    control_labels = ["target depth", "depth pid", "vel pid", "acc pid"]


    plt.figure()

    for i, var in enumerate(glider_vars):
        plt.plot(glider_time, var, label = glider_labels[i])

    for i, var in enumerate(control_vars):
        plt.plot(control_time, var, label = control_labels[i])

    plt.xlabel("Time")
    plt.title("Glider Variables")
    plt.legend(loc="lower left")
    plt.grid(True)
    plt.show()




if __name__ == "__main__":
    do_sim()
