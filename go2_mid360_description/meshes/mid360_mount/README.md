# MID-360 Mount

A MID-360 base and protective guard for the Go2 simulation, modeled approximately from photographs of the physical assembly.

## Mesh files

| File | Contents |
| --- | --- |
| `mount_only.dae` | Bent base plate and reinforcing ribs |
| `guard_only.dae` | Four guard strips and an upper ring |

Both meshes use COLLADA format, meters, and a scale of `1 1 1`. They share the origin and axes of the MID-360 LiDAR mesh.

## Assembly and configuration

- [mid360_mount.xacro](../../xacro/mid360_mount.xacro): fixed assembly of the mount, spacers, and LiDAR.
- [mid360_mount.yaml](../../config/mid360_mount.yaml): mounting pose relative to `base_link` and spacer length.
- [robot_mid360.xacro](../../xacro/robot_mid360.xacro): robot assembly entry point.

Default mounting parameters:

| Parameter | Value | Unit |
| --- | --- | --- |
| `xyz` | `[0.163, 0, 0.117]` | m |
| `rpy_degrees` | `[0, -13, 180]` | ° |
| `rear_spacer_length` | `0.022` | m |

The frame chain is `base_link → mid360_mount_link → mid360_link`. The last two frames coincide. In the default pose, the LiDAR connector faces the rear of the robot and the sensor tilts forward by 13°.

The FAST-LIO launch files read the same YAML file to calculate `mapping.extrinsic_T/R`. Currently, `imu_link` coincides with `base_link`. If the IMU mounting pose changes, update the LiDAR–IMU extrinsic calculation accordingly.

## Model properties

The mount contains visual meshes only, with no collision or LiDAR occlusion geometry. Mass and inertia use placeholder values. The LiDAR model and sensor plugin are provided by `ros2_livox_simulation`.
