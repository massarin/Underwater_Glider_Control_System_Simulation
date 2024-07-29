
# Glider Control System Simulation

## Intro

This program simulates the physics of an underwater glider, including drag, buoyancy, and lift. It also simulates a control system for the glider.

## Background

I wrote this program for [UVEEC](https://www.uveec.ca), a club at the University of Victoria.
The club's current project is to design and build an underwater glider to be deployed in Saanich Inlet to collect data. As one of the club's software team leads, my job is to design the control system. Part of that control system is designed to control the depth and movement of the glider.
The control system for the Glider must be tuned. This is a long process that, if done entirely in Saanich Inlet with an assembled Glider, would take a long time and involve risk of losing the Glider.

## Purpose

The control system can be simulated using this program, allowing for preliminary tuning parameters to be found.

## Running

1. Create a config file, an example is provided (`config.json`)

2. Run the program with `python3 GliderCsSim.py`

The program with show two plots, one is the glider's x - y position with time.

The second has 3 plots, the top is the roll, pitch, and yaw of the glider over time (in degrees, this is the only time degrees are used). Middle is the glider's depth, vertical speed, and vertical acceleration. Bottom is the control system's logging.

## TODO

1. Add sensors with noise that feed the control system

2. Redesign the control system to work better

## Future Plans

1. ROS 2 integration so parts of the Glider's actual CS may be tested.

## Config Options

> All units are metric base units, e.g. meters for length, seconds for time, radians for angles, kilograms for mass.

### `glider`

#### `control_system`

pid parameters should be mostly self explanatory

##### `high_depth`

The depth the control system will target when surfacing

##### `low_depth`

The depth targeted when diving

##### `frequency`

How many times per second the control system updates

> The more it runs the slower the simulation

---

#### `hull`

The hull is a capsule shape (cylinder with hemispheres at the end)

##### `length`

Length of the cylinder part, total length of the glider would be `length + 2 * radius`

##### `radius`

Radius of the cylinder and end caps

##### `drag_multiplier`

This is kind of just to simulate different conditions, drag coefficients are calculated based on `length` and `radius`, and can't be set.

> The hull's position is (0, 0, 0)

---

#### `buoyancy_engine`

This is a tank with a pump. It pumps water in until it's full, or pumps it out until it's empty.

> The buoyancy and mass of the glider must be balanced correctly or the glider will be unable to dive or surface. I made [this calculator](https://www.desmos.com/calculator/2p1i4syeai) for that purpose.

---

#### `hydrofoil`

It's a fancy name for wing. The coefficient of lift is approximated as a line with slope `lift_curve_slope` until `stall_angle`, where it becomes a line to zero lift at 45 degrees.

#### `initial_position / velocity / acceleration`

Positive x is forward

Positive y is right

Positive z is up

#### `initial_orientation / angular_velocity / angular_acceleration`

Positive x is roll right

Positive y is pitch up

Positive z is yaw right

---
---

### `sim`

#### `timestep`

How much time passes between physics calculations, lower is more accurate but slower.

#### `end_time`

How much time is simulated. More time takes longer.

> The simulation runs until the `end_time` is reached, there is no limit to how many iterations there are. The number of iterations is `end_time / timestep`.

---
---
---

**Written by Anthony Cieri - [email](mailto:penguinmillion@gmail.com)**
