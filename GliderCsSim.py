
import Glider
import ControlSystem
import SimMath
from SimMath import Vector



def do_sim() -> None:
    body = Glider.GliderBody(mass = 27.0, drag_coefficient = 0.3)

    control_system = ControlSystem.ControlSystem()

    glider = Glider.Glider(body = body, control_system = control_system,\
                initial_position = Vector(), initial_velocity = Vector(), initial_acceleration = Vector())
    

    time: float = 0

    time_step: float = 0.01

    max_time: float = 10_000


    while time < max_time:

        glider.sim_timestep(time)

        print(f"time: {time} \t depth: {-glider.position.z()}")

        time += time_step




if __name__ == "__main__":
    do_sim()
