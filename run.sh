source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch vehicle_bringup auto_drive.launch.py \
  preview_enabled:=true \
  minimum_speed_mps:=0.8 \
  maximum_speed_mps:=5.0 \
  minimum_duty:=0.11 \
  maximum_duty:=0.18 \
  maximum_lateral_acceleration_mps2:=1.2 \
  speed_pid_kp:=0.012 \
  speed_pid_ki:=0.004 \
  speed_pid_kd:=0.0 \
  electrical_brake_enabled:=false \
  brake_entry_speed_error_mps:=0.10 \
  brake_exit_speed_error_mps:=0.03 \
  brake_minimum_vehicle_speed_mps:=0.20 \
  brake_minimum_current_amps:=1.0 \
  brake_maximum_current_amps:=2.5 \
  brake_current_gain_amps_per_mps:=6.0 \
  brake_current_rise_amps_per_sec:=6.0 \
  brake_current_fall_amps_per_sec:=16.0 \
  stanley_gain:=1.55 \
  stanley_softening_speed_mps:=0.35 \
  stanley_heading_lookahead_m:=0.12 \
  stanley_corner_heading_threshold_deg:=20.0 \
  stanley_corner_opposing_correction_ratio:=0.90 \
  steering_current_weight:=0.58 \
  path_local_smoothing_window_m:=0.08 \
  path_outlier_threshold_m:=0.04 \
  path_geometry_window_m:=0.12 \
lane_seed_roi_height_ratio:=0.25 \
lane_seed_temporal_side_lock_reset_frames:=90 \
lane_seed_pair_minimum_distance_px:=45.0 \
lane_seed_pair_maximum_distance_px:=100.0 \
lane_seed_temporal_side_reacquire_base_distance_px:=38.0 \
lane_seed_temporal_side_reacquire_distance_per_missing_frame_px:=2.0 \
lane_seed_temporal_side_reacquire_maximum_distance_px:=50.0 \
lane_seed_sliding_window_minimum_seed_arc_length_px:=15.0 \
lane_centerline_corner_outward_bias_m:=0.01

