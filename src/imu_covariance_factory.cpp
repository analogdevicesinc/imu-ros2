// Copyright 2025 Analog Devices, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "adi_imu/imu_covariance_factory.h"

#include <stdexcept>

#include "adi_imu/ewma_covariance_provider.h"
#include "adi_imu/kalman_covariance_provider.h"
#include "adi_imu/motion_detector.h"
#include "adi_imu/sliding_window_covariance_provider.h"
#include "adi_imu/static_covariance_provider.h"
#include "adi_imu/welford_covariance_provider.h"

namespace adi_imu
{
CovarianceAlgorithm ImuCovarianceFactory::parseAlgorithm(const std::string & algorithm_str)
{
  if (algorithm_str == "static") return CovarianceAlgorithm::STATIC;
  if (algorithm_str == "welford") return CovarianceAlgorithm::WELFORD_ONLINE;
  if (algorithm_str == "sliding_window") return CovarianceAlgorithm::SLIDING_WINDOW;
  if (algorithm_str == "ewma") return CovarianceAlgorithm::EWMA;
  if (algorithm_str == "kalman") return CovarianceAlgorithm::KALMAN;

  throw std::invalid_argument("Unknown covariance algorithm: " + algorithm_str);
}

std::unique_ptr<ImuCovarianceInterface> ImuCovarianceFactory::createFromParameters(
  const std::shared_ptr<rclcpp::Node> & node)
{
  //Declare parameters with defaults
  node->declare_parameter("covariance.enable", false);
  node->declare_parameter("covariance.algorithm", "welford");

  // Welford parameters
  node->declare_parameter(
    "covariance.welford.calibration_samples",
    static_cast<int64_t>(WelfordCovarianceProvider::DEFAULT_CALIBRATION_SAMPLES));
  node->declare_parameter(
    "covariance.welford.min_variance", WelfordCovarianceProvider::DEFAULT_MIN_VARIANCE);

  // Sliding window parameters
  node->declare_parameter(
    "covariance.sliding_window.window_size",
    static_cast<int64_t>(SlidingWindowCovarianceProvider::DEFAULT_WINDOW_SIZE));
  node->declare_parameter(
    "covariance.sliding_window.min_samples",
    static_cast<int64_t>(SlidingWindowCovarianceProvider::DEFAULT_MIN_SAMPLES));
  node->declare_parameter(
    "covariance.sliding_window.min_variance",
    SlidingWindowCovarianceProvider::DEFAULT_MIN_VARIANCE);

  // Static covariance parameters (diagonal values)
  node->declare_parameter(
    "covariance.static.accel_variance_x",
    StaticCovarianceProvider::DEFAULT_STATIC_ACCEL_COVARIANCE);
  node->declare_parameter(
    "covariance.static.accel_variance_y",
    StaticCovarianceProvider::DEFAULT_STATIC_ACCEL_COVARIANCE);
  node->declare_parameter(
    "covariance.static.accel_variance_z",
    StaticCovarianceProvider::DEFAULT_STATIC_ACCEL_COVARIANCE);
  node->declare_parameter(
    "covariance.static.gyro_variance_x", StaticCovarianceProvider::DEFAULT_STATIC_GYRO_COVARIANCE);
  node->declare_parameter(
    "covariance.static.gyro_variance_y", StaticCovarianceProvider::DEFAULT_STATIC_GYRO_COVARIANCE);
  node->declare_parameter(
    "covariance.static.gyro_variance_z", StaticCovarianceProvider::DEFAULT_STATIC_GYRO_COVARIANCE);

  // Exponentially-Weighted moving average (EWMA) covariance parameters
  node->declare_parameter("covariance.ewma.alpha", EwmaCovarianceProvider::DEFAULT_ALPHA);
  node->declare_parameter(
    "covariance.ewma.warmup_samples",
    static_cast<int64_t>(EwmaCovarianceProvider::DEFAULT_WARMUP_SAMPLES));
  node->declare_parameter(
    "covariance.ewma.min_variance", EwmaCovarianceProvider::DEFAULT_MIN_VARIANCE);

  // Kalman filter covariance parameters
  node->declare_parameter(
    "covariance.kalman.process_noise_q", KalmanCovarianceProvider::DEFAULT_PROCESS_NOISE_Q);
  node->declare_parameter(
    "covariance.kalman.measurement_noise_r", KalmanCovarianceProvider::DEFAULT_MEASUREMENT_NOISE_R);
  node->declare_parameter(
    "covariance.kalman.initial_variance", KalmanCovarianceProvider::DEFAULT_INITIAL_VARIANCE);
  node->declare_parameter(
    "covariance.kalman.warmup_samples",
    static_cast<int64_t>(KalmanCovarianceProvider::DEFAULT_WARMUP_SAMPLES));
  node->declare_parameter(
    "covariance.kalman.min_variance", KalmanCovarianceProvider::DEFAULT_MIN_VARIANCE);

  // Motion detection parameters (for adaptive algorithms)
  node->declare_parameter("covariance.motion_detection.enable", true);
  node->declare_parameter(
    "covariance.motion_detection.gyro_threshold", MotionDetector::DEFAULT_GYRO_THRESHOLD);
  node->declare_parameter(
    "covariance.motion_detection.accel_threshold", MotionDetector::DEFAULT_ACCEL_THRESHOLD);

  // Check if covariance is enabled
  bool enable = node->get_parameter("covariance.enable").as_bool();
  if (!enable) {
    RCLCPP_INFO(node->get_logger(), "Covariance computation disabled.");
    return nullptr;
  }

  std::string algorithm_str = node->get_parameter("covariance.algorithm").as_string();
  CovarianceAlgorithm algorithm = parseAlgorithm(algorithm_str);

  RCLCPP_INFO(node->get_logger(), "Creating covariance provider: %s", algorithm_str.c_str());

  return create(algorithm, node);
}

std::unique_ptr<ImuCovarianceInterface> ImuCovarianceFactory::create(
  CovarianceAlgorithm algorithm, const std::shared_ptr<rclcpp::Node> & node)
{
  // Create motion detector from parameters
  bool motion_detection_enable =
    node->get_parameter("covariance.motion_detection.enable").as_bool();
  double gyro_threshold =
    node->get_parameter("covariance.motion_detection.gyro_threshold").as_double();
  double accel_threshold =
    node->get_parameter("covariance.motion_detection.accel_threshold").as_double();

  MotionDetector motion_detector(gyro_threshold, accel_threshold);
  motion_detector.setEnabled(motion_detection_enable);

  if (motion_detection_enable) {
    RCLCPP_INFO(
      node->get_logger(),
      "Motion detection enabled (gyro_threshold=%.4f rad/s, accel_threshold=%.4f m/s^2)",
      gyro_threshold, accel_threshold);
  }

  switch (algorithm) {
    case CovarianceAlgorithm::STATIC: {
      Vec3 accel_var;
      accel_var.x = node->get_parameter("covariance.static.accel_variance_x").as_double();
      accel_var.y = node->get_parameter("covariance.static.accel_variance_y").as_double();
      accel_var.z = node->get_parameter("covariance.static.accel_variance_z").as_double();
      Vec3 gyro_var;
      gyro_var.x = node->get_parameter("covariance.static.gyro_variance_x").as_double();
      gyro_var.y = node->get_parameter("covariance.static.gyro_variance_y").as_double();
      gyro_var.z = node->get_parameter("covariance.static.gyro_variance_z").as_double();
      return std::make_unique<StaticCovarianceProvider>(accel_var, gyro_var);
    }
    case CovarianceAlgorithm::WELFORD_ONLINE: {
      size_t calibration_samples =
        static_cast<size_t>(node->get_parameter("covariance.welford.calibration_samples").as_int());
      double min_variance = node->get_parameter("covariance.welford.min_variance").as_double();
      return std::make_unique<WelfordCovarianceProvider>(calibration_samples, min_variance);
    }
    case CovarianceAlgorithm::SLIDING_WINDOW: {
      size_t window_size =
        static_cast<size_t>(node->get_parameter("covariance.sliding_window.window_size").as_int());
      size_t min_samples =
        static_cast<size_t>(node->get_parameter("covariance.sliding_window.min_samples").as_int());
      double min_variance =
        node->get_parameter("covariance.sliding_window.min_variance").as_double();
      return std::make_unique<SlidingWindowCovarianceProvider>(
        window_size, min_samples, min_variance, motion_detector);
    }
    case CovarianceAlgorithm::EWMA: {
      double alpha = node->get_parameter("covariance.ewma.alpha").as_double();
      size_t warmup_samples =
        static_cast<size_t>(node->get_parameter("covariance.ewma.warmup_samples").as_int());
      double min_variance = node->get_parameter("covariance.ewma.min_variance").as_double();
      return std::make_unique<EwmaCovarianceProvider>(
        alpha, warmup_samples, min_variance, motion_detector);
    }
    case CovarianceAlgorithm::KALMAN: {
      double process_noise_q = node->get_parameter("covariance.kalman.process_noise_q").as_double();
      double measurement_noise_r =
        node->get_parameter("covariance.kalman.measurement_noise_r").as_double();
      double initial_variance =
        node->get_parameter("covariance.kalman.initial_variance").as_double();
      size_t warmup_samples =
        static_cast<size_t>(node->get_parameter("covariance.kalman.warmup_samples").as_int());
      double min_variance = node->get_parameter("covariance.kalman.min_variance").as_double();
      return std::make_unique<KalmanCovarianceProvider>(
        process_noise_q, measurement_noise_r, initial_variance, warmup_samples, min_variance,
        motion_detector);
    }
  }

  return nullptr;
}

}  // namespace adi_imu