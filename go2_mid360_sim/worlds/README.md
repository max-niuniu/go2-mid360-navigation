# Go2 Workshop

The default indoor Go2 world for LiDAR mapping, localization, and navigation with obstacle avoidance. Requires Gazebo Classic 11.

## Specifications

| Property | Value |
| --- | --- |
| File | `go2_workshop.world` (SDF 1.6) |
| Dimensions | 18 × 14 m, enclosed by 2.2 m walls; no roof |
| Spawn area | A clear 2 × 2 m area at the world origin `(0, 0)` |
| Obstacles | Shelves, workbenches, storage cabinets, crates, tool carts, and bins |
| Physics | Static obstacles with collision geometry on their main bodies |
| Assets | Geometry and materials embedded in the world file; no external model dependencies |

## Launch

After building the workspace, load the environment from the workspace root:

```bash
source install/setup.bash
```

Run one of the following modes:

```bash
# Simulation
ros2 launch go2_mid360_sim simulation.launch.py gui:=true

# Mapping
ros2 launch go2_mid360_bringup mapping.launch.py gui:=true show_rviz:=true

# Navigation
ros2 launch go2_mid360_bringup navigation.launch.py gui:=true show_rviz:=true
```

Each mode loads this world by default.

## Maps

Default runtime maps are stored in `maps/go2_workshop/` under the workspace root:

- `map.pcd`: 3D point cloud for ICP localization.
- `map.yaml` and `map.pgm`: 2D occupancy map for Nav2.

Mapping saves to `map.pcd` by default, overwriting an existing file. Regenerate the maps after changing the world layout. See [Maps](../../README.md#maps) for saving and conversion commands.
