# ROS2 Occupancy Grid Mapping

Build a 2D **occupancy grid map** from a laser scanner using **log-odds** updates and
**TF2**, publish it as `nav_msgs/OccupancyGrid`, and save it in the same format as
`nav2_map_server` so Nav2 can load it later.

![stack](https://img.shields.io/badge/ROS2-Humble-blue) ![python](https://img.shields.io/badge/python-3.10-green)

## Depends on

This project reuses the simulator from
[ros2-obstacle-avoidance](../ros2-obstacle-avoidance). Clone both into the same workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/<you>/ros2-obstacle-avoidance.git
git clone https://github.com/<you>/ros2-occupancy-mapping.git
cd ~/ros2_ws && rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install && source install/setup.bash
```

## What you learn

- Occupancy grid mapping: inverse sensor model, log-odds, clamping
- Bresenham ray tracing on a grid
- Looking up a transform at the scan's timestamp with a `tf2_ros.Buffer`
- `nav_msgs/OccupancyGrid` layout (row-major, origin, -1 = unknown)
- Latched topics in ROS2 (`TRANSIENT_LOCAL` durability)
- Saving maps as PGM + YAML (map_server format)
- Vectorised array work with NumPy

## Run

```bash
ros2 launch occupancy_mapping mapping.launch.py
```

The robot explores on its own and the map fills in. When it looks complete:

```bash
ros2 service call /save_map std_srvs/srv/Trigger     # -> ~/maps/sim_world.pgm/.yaml
ros2 service call /clear_map std_srvs/srv/Trigger
```

Prefer to drive? `autonomous:=false` and run `teleop_twist_keyboard`.

Load a saved map with Nav2's map server:

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$HOME/maps/sim_world.yaml
ros2 lifecycle set /map_server configure && ros2 lifecycle set /map_server activate
```

## Test

```bash
colcon test --packages-select occupancy_mapping && colcon test-result --verbose
```

## Exercises

1. Add odometry noise to the simulator and watch the map smear, which is why SLAM exists.
2. Publish a separate `map -> odom` transform and map in the `map` frame.
3. Compare your map with `slam_toolbox` running on the same `/scan`.
4. Speed up `integrate_scan` by vectorising the ray tracing with NumPy.
