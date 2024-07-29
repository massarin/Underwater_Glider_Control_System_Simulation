
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

## TODO

1. ROS 2 integration so parts of the Glider's actual CS may be tested.

> Anthony Cieri [email](mailto:penguinmillion@gmail.com)
