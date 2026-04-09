# vio_tf

ROS Noetic package: static **imu → base_link** extrinsics and Python nodes that align mocap / OpenVINS frames via TF.

## Build

From your catkin workspace:

```bash
cd /path/to/vio_ws
source /opt/ros/noetic/setup.bash
catkin_make   # or catkin build vio_tf
source devel/setup.bash
```

## Launch files

### `vio_tf.launch`

Starts: static `imu` → `base_link`, `mocap_rot.py`, `tf_odom_rot.py`, `odom_mocap_rot.py`.

**Inputs (from the rest of your stack)**

| Kind | Name | Type / note |
|------|------|-------------|
| Topic | `/natnet_ros/kingfisher/odom` | `nav_msgs/Odometry` — mocap body pose in **`world`** (drives `mocap_rot.py` and `odom_mocap_rot.py`) |
| Topic | `/ov_msckf/odomimu` | `nav_msgs/Odometry` — OpenVINS IMU odometry in **`global`** / **`imu`** (used by `tf_odom_rot.py`) |


**Outputs (from this launch)**

| Kind | Name | Type / note |
|------|------|-------------|
| TF | `imu` → `base_link` | Static extrinsic (fixed in launch args) |
| TF | `world` → `global` | Static, published once from the **first** `/natnet_ros/kingfisher/odom` message (`mocap_rot.py`) |
| Topic | `/ov_msckf/base_link_corrected` | `nav_msgs/Odometry` — base pose/twist in **`global`** / **`base_link`** |
| Topic | `/natnet_ros/mocap_in_global` | `nav_msgs/Odometry` — mocap pose expressed with **`global`** as `frame_id` |

```bash
roslaunch vio_tf vio_tf.launch
```

---

### `vio_tf_static_odom.launch`

Starts: static `imu` → `base_link` and **`tf_odom_rot.py` only** (no mocap nodes).

**Inputs (from the rest of your stack)**

| Kind | Name | Type / note |
|------|------|-------------|
| Topic | `/ov_msckf/odomimu` | `nav_msgs/Odometry` — OpenVINS IMU odometry |
| TF | VIO / your stack | Must provide a connected TF chain so lookups **`global` ← `base_link`** and **`base_link` ← `imu`** succeed (this launch does **not** publish `world` → `global`; add `rot.py` separately or another `global` origin if needed) |

**Outputs (from this launch)**

| Kind | Name | Type / note |
|------|------|-------------|
| TF | `imu` → `base_link` | Static extrinsic (same args as full launch) |
| Topic | `/ov_msckf/base_link_corrected` | `nav_msgs/Odometry` — base pose/twist in **`global`** / **`base_link`** |

```bash
roslaunch vio_tf vio_tf_static_odom.launch
```

## Nodes (scripts)

### Static transform (in launch files)

- **Publishes:** `imu` → `base_link`  
- **Parameters:** `x y z yaw pitch roll` = `0 0.0402 -0.0896 0.0 -1.57 1.57` (radians), implemented with `tf2_ros/static_transform_publisher`.

### `rot.py` (`world_to_global_publisher`)

- **Subscribes:** `/natnet_ros/kingfisher/odom` (`nav_msgs/Odometry`)  
- **Publishes:** One-time static TF `world` → `global` from the first mocap message (anchors OpenVINS `global` to the mocap `world`).

### `tf_odom_rot.py` (`odom_frame_transformer`)

- **Subscribes:** `/ov_msckf/odomimu` (`nav_msgs/Odometry`)  
- **Publishes:** `/ov_msckf/base_link_corrected` (`nav_msgs/Odometry`)  
- **TF:** Looks up `global` ← `base_link` and `base_link` ← `imu` to express pose and twist in `global` / `base_link`. Requires the imu→base_link static TF and a consistent `global` frame (e.g. from `rot.py` or your own publisher).

### `odom_mocap_rot.py` (`global_to_mocap_node`)

- **Subscribes:** `/natnet_ros/kingfisher/odom`  
- **Publishes:** `/natnet_ros/mocap_in_global` (`nav_msgs/Odometry`)  
- **TF:** Transforms mocap pose from `world` into `global` using `global` ← `world`.

## Running nodes without launch

```bash
rosrun vio_tf rot.py
rosrun vio_tf tf_odom_rot.py
rosrun vio_tf odom_mocap_rot.py
```

Equivalent static TF alone:

```bash
rosrun tf2_ros static_transform_publisher 0 0.0402 -0.0896 0.0 -1.57 1.57 imu base_link
```

## Frames overview

Typical chain: **`world`** (mocap) ↔ **`global`** (VIO, from `rot.py`) → **`imu`** → **`base_link`**. Adjust topics and frames in the scripts if your drivers use different names.
