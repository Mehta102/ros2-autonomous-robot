#!/bin/bash
# start the simulation

export LIBGL_ALWAYS_SOFTWARE=1
cd "$(dirname "$0")"
python3 launch/launch_simulation.py
