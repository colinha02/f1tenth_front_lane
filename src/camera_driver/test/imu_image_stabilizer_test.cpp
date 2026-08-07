#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <thread>

#include <opencv2/core.hpp>

#include "camera_driver/imu_image_stabilizer.hpp"

namespace
{

constexpr double kDegreesToRadians =
  3.141592653589793238462643383279502884 / 180.0;

void require(const bool condition, const char * message)
{
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

camera_driver::ImuImageStabilizerConfig fastConfig()
{
  camera_driver::ImuImageStabilizerConfig config;
  config.startup_discard_duration_sec = 0.0;
  config.reference_calibration_duration_sec = 0.01;
  config.stationary_detection_window_sec = 0.01;
  config.maximum_frame_imu_wait_sec = 0.02;
  config.maximum_frame_imu_age_sec = 0.006;
  config.maximum_frame_imu_prediction_sec = 0.0;
  return config;
}

void calibrate(camera_driver::ImuImageStabilizer & stabilizer)
{
  for (int index = 0; index <= 4; ++index) {
    stabilizer.update(
      cv::Vec3d(0.0, -9.80665, 0.0),
      cv::Vec3d(0.0, 0.0, 0.0),
      0.0025 * static_cast<double>(index));
  }
  require(stabilizer.initialized(), "stationary calibration did not finish");
}

void verifyStartupSamplesAreDiscarded()
{
  auto config = fastConfig();
  config.startup_discard_duration_sec = 0.01;
  camera_driver::ImuImageStabilizer stabilizer(config);

  for (int index = 0; index <= 4; ++index) {
    stabilizer.update(
      cv::Vec3d(0.0, -9.80665, 0.0),
      cv::Vec3d(0.0, 0.0, 1.0),
      0.0025 * static_cast<double>(index));
  }
  const auto discarded = stabilizer.calibrationProgress();
  require(
    !discarded.discarding_startup_samples,
    "startup discard did not finish at its configured boundary");
  require(
    discarded.accepted_samples == 0U,
    "a startup-discard gyro sample leaked into calibration");

  for (int index = 1; index <= 6; ++index) {
    stabilizer.update(
      cv::Vec3d(0.0, -9.80665, 0.0),
      cv::Vec3d(0.0, 0.0, 0.0),
      0.01 + 0.0025 * static_cast<double>(index));
  }
  require(stabilizer.initialized(), "post-discard calibration did not finish");
  require(
    cv::norm(stabilizer.gyroscopeBiasRadps()) < 1.0e-12,
    "discarded startup gyro contaminated the learned bias");
}

void verifyDynamicAccelerationDoesNotMoveTilt()
{
  const auto config = fastConfig();
  camera_driver::ImuImageStabilizer lateral_stabilizer(config);
  calibrate(lateral_stabilizer);

  double timestamp_sec = 0.01;
  for (int index = 1; index <= 800; ++index) {
    timestamp_sec = 0.01 + 0.0025 * static_cast<double>(index);
    lateral_stabilizer.update(
      cv::Vec3d(0.5, -9.80665, 0.0),
      cv::Vec3d(0.0, 0.0, 0.0),
      timestamp_sec);
  }
  const auto lateral_correction = lateral_stabilizer.correctionAt(
    timestamp_sec);
  require(
    lateral_correction.has_value(), "lateral-acceleration lookup failed");
  require(
    std::abs(lateral_correction->roll_error_deg) < 1.0e-9,
    "sub-gate lateral acceleration was accumulated as roll");
  require(
    !lateral_stabilizer.stationaryConfirmed(),
    "lateral acceleration was incorrectly classified as quiet motion");

  camera_driver::ImuImageStabilizer longitudinal_stabilizer(config);
  calibrate(longitudinal_stabilizer);
  for (int index = 1; index <= 800; ++index) {
    timestamp_sec = 0.01 + 0.0025 * static_cast<double>(index);
    longitudinal_stabilizer.update(
      cv::Vec3d(0.0, -9.80665, 0.5),
      cv::Vec3d(0.0, 0.0, 0.0),
      timestamp_sec);
  }
  const auto longitudinal_correction = longitudinal_stabilizer.correctionAt(
    timestamp_sec);
  require(
    longitudinal_correction.has_value(),
    "longitudinal-acceleration lookup failed");
  require(
    std::abs(longitudinal_correction->pitch_error_deg) < 1.0e-9,
    "sub-gate longitudinal acceleration was accumulated as pitch");
  require(
    !longitudinal_stabilizer.stationaryConfirmed(),
    "longitudinal acceleration was incorrectly classified as quiet motion");
}

void verifyFrameWaitsForBracketingImuWithoutPrediction()
{
  const auto config = fastConfig();
  camera_driver::ImuImageStabilizer stabilizer(config);
  calibrate(stabilizer);

  stabilizer.update(
    cv::Vec3d(0.0, -9.80665, 0.0),
    cv::Vec3d(0.0, 0.0, 1.0),
    0.0125);
  std::thread future_imu([&stabilizer]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      stabilizer.update(
        cv::Vec3d(0.0, -9.80665, 0.0),
        cv::Vec3d(0.0, 0.0, 1.0),
        0.0175);
    });

  const auto interpolated = stabilizer.correctionAt(0.0150);
  future_imu.join();
  require(interpolated.has_value(), "bracketing IMU wait failed");
  require(
    !interpolated->predicted,
    "bracketed frame was incorrectly marked as predicted");
  require(
    std::abs(std::abs(interpolated->nearest_imu_delta_sec) - 0.0025) < 1.0e-9,
    "frame did not use the nearest bracketing IMU timestamps");

  require(
    !stabilizer.correctionAt(0.0300).has_value(),
    "future frame was predicted while prediction was disabled");
}

}  // namespace

int main()
{
  verifyStartupSamplesAreDiscarded();
  verifyDynamicAccelerationDoesNotMoveTilt();
  verifyFrameWaitsForBracketingImuWithoutPrediction();

  const auto config = fastConfig();
  camera_driver::ImuImageStabilizer stabilizer(config);
  calibrate(stabilizer);

  stabilizer.update(
    cv::Vec3d(0.0, -9.80665, 0.0),
    cv::Vec3d(0.0, 0.0, 1.0),
    0.0125);
  const auto roll_correction = stabilizer.correctionAt(0.0125);
  require(roll_correction.has_value(), "roll correction lookup failed");
  require(
    std::abs(roll_correction->roll_error_deg) > 0.1,
    "camera-Z rotation did not move the fixed roll reference");
  require(
    !roll_correction->predicted,
    "exact timestamp was incorrectly marked as predicted");

  require(
    !stabilizer.correctionAt(0.0200).has_value(),
    "future frame was predicted while prediction was disabled");

  const auto homography = camera_driver::makeImageStabilizationHomography(
    500.0, 500.0, 640.0, 360.0, *roll_correction);
  require(
    cv::checkRange(cv::Mat(homography)),
    "fixed-reference homography is not finite");

  camera_driver::ImuImageStabilizer pitch_stabilizer(config);
  calibrate(pitch_stabilizer);
  pitch_stabilizer.update(
    cv::Vec3d(0.0, -9.80665, 0.0),
    cv::Vec3d(-1.0, 0.0, 0.0),
    0.0125);
  const auto pitch_correction = pitch_stabilizer.correctionAt(0.0125);
  require(pitch_correction.has_value(), "pitch correction lookup failed");
  require(
    pitch_correction->pitch_error_deg > 0.1,
    "negative camera-X gyro must produce positive downward pitch");

  const auto pitch_homography =
    camera_driver::makeImageStabilizationHomography(
    500.0, 500.0, 640.0, 360.0, *pitch_correction);
  const cv::Vec3d current_horizon_ray(
    0.0,
    -std::sin(pitch_correction->pitch_error_deg * kDegreesToRadians),
    std::cos(pitch_correction->pitch_error_deg * kDegreesToRadians));
  const cv::Vec3d current_horizon_pixel(
    500.0 * current_horizon_ray[0] + 640.0 * current_horizon_ray[2],
    500.0 * current_horizon_ray[1] + 360.0 * current_horizon_ray[2],
    current_horizon_ray[2]);
  const cv::Vec3d stabilized_horizon_pixel =
    pitch_homography * current_horizon_pixel;
  require(
    std::abs(
      stabilized_horizon_pixel[1] / stabilized_horizon_pixel[2] - 360.0) <
    1.0e-5,
    "pitch homography did not restore the startup horizon");

  camera_driver::ImuImageStabilizer yaw_stabilizer(config);
  calibrate(yaw_stabilizer);
  yaw_stabilizer.update(
    cv::Vec3d(0.0, -9.80665, 0.0),
    cv::Vec3d(0.0, 1.0, 0.0),
    0.0125);
  const auto yaw_correction = yaw_stabilizer.correctionAt(0.0125);
  require(yaw_correction.has_value(), "yaw attitude lookup failed");
  require(
    yaw_correction->correction_angle_deg < 1.0e-6,
    "gravity-axis yaw must not be stabilized");

  const cv::Matx33d camera_matrix(
    500.0, 0.0, 640.0,
    0.0, 500.0, 360.0,
    0.0, 0.0, 1.0);
  const auto zoom = camera_driver::makeFixedViewZoomHomography(
    camera_matrix, 1.25);
  require(
    camera_driver::outputIsCoveredBySource(
      zoom, cv::Size(1280, 720), cv::Size(1280, 720), 1.5),
    "fixed 1.25x view is not covered by the source image");

  auto moving_config = fastConfig();
  moving_config.calibration_maximum_angular_speed_degps = 0.5;
  camera_driver::ImuImageStabilizer moving_stabilizer(moving_config);
  for (int index = 0; index <= 8; ++index) {
    moving_stabilizer.update(
      cv::Vec3d(0.0, -9.80665, 0.0),
      cv::Vec3d(0.0, 0.0, 0.1),
      0.0025 * static_cast<double>(index));
  }
  require(
    !moving_stabilizer.initialized(),
    "moving camera was accepted as the fixed startup reference");
  require(
    moving_stabilizer.calibrationProgress().reset_count > 0U,
    "moving calibration did not report a reset");

  std::cout << "imu_image_stabilizer_test passed\n";
  return 0;
}
