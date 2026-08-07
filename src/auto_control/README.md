# auto_control

## Front-camera lane detector

`front_lane_detector` uses the rectified **front-camera image directly**.  It
does not create a bird's-eye-view image and does not use stereo depth.

The node detects low-saturation bright lane markings in a configurable lower
image ROI, follows each marking with independent sliding windows, fits a
quadratic image-space curve, and applies short temporal confirmation/hold to
avoid background-induced jumps.  If only one marking is visible, it retains
that marking and estimates the centerline from the last reliable lane-width
profile rather than forcing a second detection.

Input and outputs:

```text
/camera/image_rect       sensor_msgs/Image (NV12 from camera_driver)
/front_lane/mask         sensor_msgs/Image (mono8 candidate mask)
/front_lane/overlay      sensor_msgs/Image (bgr8; detected lanes are blue)
/front_lane/model        std_msgs/Float32MultiArray
```

`/front_lane/model.data` is:

```text
[confidence, lateral_error_normalized, lookahead_offset_normalized,
 curvature_px_inverse, left_detected, right_detected]
```

The lookahead offset and curvature are intentionally published for a later
controller.  This first node never publishes a VESC or steering command.

Run the camera and detector together:

```bash
ros2 launch auto_control front_lane.launch.py
```

The default configuration opens an OpenCV window named `Front lane detection`.
It displays the blue lane overlay directly and is capped at 30 FPS. Set
`preview_enabled: false` when operating headlessly, then inspect
`/front_lane/overlay` with `rqt_image_view` if needed.
