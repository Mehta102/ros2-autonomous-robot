#!/bin/bash
# start the controller

cd "$(dirname "$0")"
python3 scripts/robot_behaviour_controller.py
