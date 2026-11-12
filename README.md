# Autonomous ROS2 Robot

A simulated mobile robot that explores an unknown arena with no map. Behaviour is built as a **subsumption architecture**: three layers compete every control tick, and the highest-priority layer that is active takes over the velocity command.

Built April 2026 as a university project.

## Architecture

| Priority | Layer | What it does |
|---|---|---|
| 2 (highest) | Obstacle avoidance | Reacts when something is within 0.7 m in the front +/-30 degree arc. Picks a turn direction by comparing left and right clearance, with hysteresis and a committed turn so it does not flip-flop. Backs up under 0.4 m |
| 1 | Wall following | Proportional controller (Kp 1.8) holding about 0.9 m from a wall on the right. Needs at least 5 LIDAR hits, so a lone cylinder is not mistaken for a wall |
| 0 (base) | Wander | Forward motion with random turns, always active as the fallback |

A `SubsumptionArbiter` picks the winning layer each tick (20 Hz) and publishes the active behaviour name on `/active_behaviour`.

## Robot

Four-wheeled robot with the front wheels driven through Gazebo's DiffDrive plugin. Sensors:

- 360 sample GPU LIDAR, 10 Hz, 0.1 to 10 m range
- 640x480 camera at 30 Hz on a pan joint (+/-90 degrees), swept back and forth by the controller

## ROS2 interface

| Topic | Type | Direction |
|---|---|---|
| `/lidar` | `LaserScan` | in |
| `/odom` | `Odometry` | in |
| `/camera` | `Image` | in |
| `/cmd_vel` | `Twist` | out |
| `/camera_pan_cmd` | `Float64` | out |
| `/active_behaviour` | `String` | out |

Gazebo and the ROS bridge are started by `launch/launch_simulation.py`.

## Testing

`formal_testing.py` runs 3 scenarios (open space, obstacle course, wall following) x 3 trials and logs distance, near-collisions (closer than 0.25 m), behaviour switches, minimum clearance and time spent in each layer to CSV. Results are in `src/formal_test_results.csv`.

- 8 of 9 trials had zero near-collisions. The exception is obstacle course trial 1 (13 events, minimum distance 0.163 m)
- Wall-following engagement varied a lot between trials (0 s to 27 s in the layer), which is the first thing I would tune

## Run it

Needs ROS2, Gazebo (`ign gazebo`) and `ros_gz_bridge`. OpenCV is optional, only for the camera viewer.

```bash
cd src
bash start_simulation.sh     # Gazebo + bridge
bash start_controller.sh     # behaviour controller (second terminal)
python3 scripts/formal_testing.py   # optional: run the 9 test trials
python3 scripts/camera_viewer.py    # optional: see the robot camera
```

## Layout

```
src/
  launch/launch_simulation.py   starts Gazebo and the ROS bridge
  models/my_robot/              robot model (SDF)
  worlds/robot_world.sdf        walled arena with obstacles
  scripts/
    robot_behaviour_controller.py   subsumption controller
    formal_testing.py               3 scenarios x 3 trials
    camera_viewer.py                camera debug view
  formal_test_results.csv       recorded results
```
