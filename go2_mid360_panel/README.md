# Go2 Control Panel

Desktop controls for ROS 2 simulation, mapping, localization, and navigation. RViz handles map visualization and goal selection. The dashboard shows joint poses, motion direction, linear speed, and angular speed at up to 10 Hz. Joint poses come from `/joint_states`; velocity comes from Gazebo `/odom/ground_truth`. The display reports a disconnected state when data stops arriving.

![Go2 control panel](../media/control-panel.png)

## Launch

Requires ROS 2 Humble, `python3-pyqt5`, `python3-yaml`, `python3-numpy`, `gnome-terminal`, and `teleop_twist_keyboard`. After building, run from the workspace root:

```bash
bash src/go2-mid360-navigation/scripts/run_control_panel.bash
```

With the workspace environment loaded, you can also run `ros2 run go2_mid360_panel control_panel`.

## Tasks and maps

- Select a mode and choose whether to open Gazebo and RViz. Stop the current task before switching modes.
- Closing the panel stops the tasks it started. Shutdown allows up to 45 seconds for nodes to exit and save point clouds.
- Restarting RViz reopens only the viewer; simulation and navigation continue running.
- A map directory contains `map.pcd`, `map.yaml`, and `map.pgm`. Select a separate directory for a new mapping session. Overwriting existing files requires confirmation in the panel.
- Save the 3D point cloud while mapping is running. Stop the simulation before converting it to a 2D navigation map.
- The default projection height range is 0.35–2.60 m, with a resolution of 0.05 m. Conversion settings can be adjusted or reset in the advanced settings section.

## Keyboard teleoperation

Uses `teleop_twist_keyboard`. Default speeds are **0.5 m/s** and **1.0 rad/s**.

| Key | Action |
| --- | --- |
| W / X | Forward / backward |
| A / D | Turn left / right |
| Q / E / Z / C | Move and turn |
| Shift + movement key | Translate without turning |
| S / Space | Stop |
| R / F | Increase / decrease both speeds |
| T / G | Increase / decrease linear speed |
| Y / H | Increase / decrease angular speed |
| Ctrl+C | Exit teleoperation |

Speed adjustments are 10% per key press. Hold a movement key to keep moving; after release, the controller stops on a 0.5-second command timeout.

Navigation and keyboard teleoperation are mutually exclusive. Teleoperation exits if another velocity publisher is detected and reports the conflicting node. Closing the teleoperation terminal does not stop the simulation.

## Logs

The log terminal opens by default. Press `Ctrl+C` there to stop the task; closing the terminal window does not stop it. After the task ends, press Enter to close the terminal. It can be reopened from the panel.

Full logs are stored in `${ROS_LOG_DIR:-~/.ros/log}/go2_panel/`. The panel retains the most recent 2,500 lines and supports log export.

## Interface assets

`assets/go2-preview.json` contains simplified robot meshes; see the [Go2 model license](../go2_mid360_description/meshes/LICENSE). Regenerate this asset with `tools/build_preview_asset.py`, using NumPy, SciPy, and the workspace environment. `panel.qss` defines the interface styling; SVG files provide the icons.
