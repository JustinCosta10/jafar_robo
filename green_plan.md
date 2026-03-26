# Green Paper Follower — ROS2 Project Plan

## Overview

A ROS2-based racecar robot that uses an Intel RealSense camera to detect a green piece of
paper and follow it. The vision node localizes the paper as a 3D point in camera space and
publishes it. The control node drives the robot toward the paper using proportional steering
and stops when close enough.

---

## Package Structure

```
ros2_ws/src/
├── green_vision/     # Color detection + 3D back-projection, publishes PointStamped
└── green_control/    # Reads PointStamped, publishes cmd_vel
```

No custom message package needed — uses `geometry_msgs/PointStamped` from standard ROS2.

---

## Topic Graph

```
[realsense2_camera node]
        |
        |  /camera/aligned_depth_to_color/image_raw  (sensor_msgs/Image)
        |  /camera/color/image_raw                   (sensor_msgs/Image)
        |  /camera/color/camera_info                 (sensor_msgs/CameraInfo)
        |
        v
[green_vision node]
        |
        |  /green_target  (geometry_msgs/PointStamped)
        |    point.x = lateral offset in meters (negative=left, positive=right)
        |    point.y = vertical offset (ignored by control)
        |    point.z = forward distance in meters
        |
        v
[green_control node]
        |
        |  /cmd_vel  (geometry_msgs/Twist)
        v
[robot base / motor driver]
```

---

## green_vision Node

### Subscriptions
| Topic | Type |
|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` |

### Publication
| Topic | Type |
|---|---|
| `/green_target` | `geometry_msgs/PointStamped` |

### Algorithm

**Step 1 — Color detection**
1. Convert the RGB image from BGR to HSV
2. Threshold on green (OpenCV HSV scale: H 40–80, S 80–255, V 80–255 — tune these)
3. Erode then dilate the mask to remove noise
4. Find contours in the mask
5. Select the largest contour — reject if area is below `min_contour_area`
6. Compute the centroid `(cx_px, cy_px)` of that contour using image moments

**Step 2 — Depth sampling**
1. Look up the depth value at `(cx_px, cy_px)` in the aligned depth image
2. RealSense depth is in **millimeters** as uint16 — convert: `depth_m = depth_raw / 1000.0`
3. Reject if `depth_m == 0` or `depth_m > max_depth_param` (bad reading)

**Step 3 — Back-projection to 3D**

Use camera intrinsics from `/camera/color/camera_info`:
```python
fx = camera_info.k[0]
fy = camera_info.k[4]
cx = camera_info.k[2]
cy = camera_info.k[5]

x = (cx_px - cx) * depth_m / fx   # lateral offset in meters
y = (cy_px - cy) * depth_m / fy   # vertical offset in meters
z = depth_m                        # forward distance in meters
```

**Step 4 — Publish**
```python
msg = PointStamped()
msg.header.stamp = self.get_clock().now().to_msg()
msg.header.frame_id = 'camera_color_optical_frame'
msg.point.x = x
msg.point.y = y
msg.point.z = z
self.publisher.publish(msg)
```

If no valid detection, **do not publish**. The control node uses message staleness
to detect the lost-target condition.

### Tunable ROS2 Parameters
| Parameter | Default | Description |
|---|---|---|
| `h_min` | 40 | HSV hue lower bound |
| `h_max` | 80 | HSV hue upper bound |
| `s_min` | 80 | HSV saturation lower bound |
| `v_min` | 80 | HSV value lower bound |
| `min_contour_area` | 500 | Minimum blob area in pixels² |
| `max_depth` | 3.0 | Reject detections beyond this distance (m) |

### Debug tip
Publish a second topic `/green_vision/debug_image` with the contour and centroid
drawn on the frame. View with `rqt_image_view` while tuning HSV parameters.
---

## green_control Node

### 3.1 Subscription
| Topic | Type |
|---|---|
| `/green_target` | `geometry_msgs/PointStamped` |

### 3.2 Publication
| Topic | Type |
|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` |

### 3.3 Lost-target detection

Since `green_vision` publishes nothing when the paper is not detected, the control
node detects loss by checking message staleness on a timer:

```python
def timer_callback(self):
    now = self.get_clock().now()
    age = (now - self.last_msg_time).nanoseconds / 1e9
    if age > self.target_timeout:
        self.publish_cmd(linear=0.0, angular=0.0)  # stop
```

### 3.4 State machine

On each received `/green_target` message:

```
if point.z > stop_distance:

    if |point.x| > dead_zone:
        STATE: TURN ONLY
        angular = -Kp_steer * point.x
        linear  = 0.0

    else:
        STATE: FORWARD
        linear  = base_speed
        angular = -Kp_steer * point.x   # small correction while driving

else:
    STATE: STOP
    linear  = 0.0
    angular = 0.0
```

The TURN-only state (no forward when misaligned) prevents the robot from charging
at the paper while pointed the wrong direction.

### Tunable ROS2 Parameters
| Parameter | Default | Description |
|---|---|---|
| `kp_steer` | 1.0 | Proportional steering gain |
| `base_speed` | 0.3 | Forward speed in m/s |
| `stop_distance` | 0.4 | Stop when paper closer than this (m) |
| `dead_zone` | 0.05 | Ignore lateral errors smaller than this (m) |
| `target_timeout` | 0.5 | Seconds without a message before stopping |

---

## Integration & Testing

### Recommended test sequence

**Test 1 — Vision only (robot stationary)**
```bash
ros2 run green_vision green_vision_node
ros2 topic echo /green_target
```
Hold paper at various distances and angles. Confirm `point.z` matches actual
distance and `point.x` is negative when paper is left, positive when right.

**Test 2 — Visualize detection**
```bash
rqt_image_view
# Select /green_vision/debug_image
```
Confirm the contour and centroid overlay land correctly on the paper.

**Test 3 — Control dry run (robot off or wheels elevated)**
```bash
ros2 run green_control green_control_node
ros2 topic echo /cmd_vel
```
Move paper left/right — confirm `angular.z` responds correctly.
Move paper close — confirm `linear.x` goes to 0.
Remove paper — confirm robot stops after `target_timeout`.

**Test 4 — Slow follow test**
Set `base_speed` to 0.1 m/s. Walk slowly with the paper. Tune `kp_steer`
until steering is smooth without oscillation.

**Test 5 — Full speed tuning**
Increase `base_speed` and adjust `stop_distance` to taste.

---

## Common Issues

| Symptom | Likely cause | Fix |
|---|---|---|
| Detection jumps around | HSV range too wide | Narrow hue range, raise `min_contour_area` |
| Robot oscillates left/right | `kp_steer` too high | Reduce gain or widen `dead_zone` |
| Depth reads 0 at centroid | Centroid lands on depth hole at paper edge | Erode the mask before computing centroid |
| Robot doesn't stop | `stop_distance` too small | Increase it |
| Green detected in background | Other green objects in scene | Raise `s_min`/`v_min`, or add an ROI crop |
| Robot charges while misaligned | No TURN-only state | Ensure forward is 0 when `|point.x| > dead_zone` |
