ros2 launch vehicle_bringup auto_drive.launch.py \
  preview_enabled:=true \
  minimum_speed_mps:=0.8 \
  minimum_duty:=0.08 \
  maximum_speed_mps:=2.3 \
  maximum_duty:=0.1 \
  maximum_lateral_acceleration_mps2:=0.8 \
  
  curvature_percentile:=80.0 \
  duty_rise_rate_per_sec:=0.10 \
  duty_fall_rate_per_sec:=0.15 \
  speed_pid_integral_limit:=1.0 \
  speed_filter_time_constant_sec:=0.10 \
  speed_pid_kp:=0.008 \
  speed_pid_ki:=0.001 \
  speed_pid_kd:=0.0 \
  
  electrical_brake_enabled:=true \
  
  brake_entry_speed_error_mps:=0.20 \
  brake_exit_speed_error_mps:=0.07 \
  brake_minimum_current_amps:=0.25 \
  brake_maximum_current_amps:=1.2 \
  brake_current_gain_amps_per_mps:=3.0 \
  brake_current_rise_amps_per_sec:=2.0 \
  brake_current_fall_amps_per_sec:=8.0 \
  
  stanley_gain:=1.50 \
  stanley_softening_speed_mps:=0.35 \
  stanley_heading_lookahead_m:=0.10 \
  stanley_corner_heading_threshold_deg:=15.0 \
  stanley_corner_opposing_correction_ratio:=0.80 \
  steering_current_weight:=0.50 \
  path_local_smoothing_window_m:=0.07 \
  path_outlier_threshold_m:=0.05 \
  path_geometry_window_m:=0.10 \
  lane_seed_roi_height_ratio:=0.25 \
  lane_seed_temporal_side_lock_reset_frames:=95 \
  lane_seed_pair_minimum_distance_px:=50.0 \
  lane_seed_pair_maximum_distance_px:=90.0 \
  lane_seed_temporal_side_reacquire_base_distance_px:=40.0 \
  lane_seed_temporal_side_reacquire_distance_per_missing_frame_px:=2.0 \
  lane_seed_temporal_side_reacquire_maximum_distance_px:=50.0 \
  lane_seed_sliding_window_minimum_seed_arc_length_px:=15.0 \
  lane_centerline_corner_outward_bias_m:=0.1
