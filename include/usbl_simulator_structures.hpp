/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#pragma once

#include <Eigen/Dense>

namespace usbl_simulator {

struct UsblSimParams {
  double speed_of_sound;   // underwater speed of sound (m/s)
  double max_range;        // max. operating range of USBL (m)
  double rtt_std_dev;      // RTT standard deviation (s)
  double tdoa_std_dev;     // TDOA standard deviation (s)
  double rtt_resolution;   // RTT resolution (s)
  double tdoa_resolution;  // TDOA resolution (s)
  Eigen::Vector3d
      usblAhrsNoiseStdDev;   // RPY STD of int. AHRS of USBL transceiver (rad)
  Eigen::Vector3d p_bsTP_b;  // lever arm between transponder and body frame (m)
  Eigen::Matrix3d C_sTP_b;   // rotation from transponder to body frame (-)
  Eigen::Vector3d p_bu_b;  // lever arm between body and USBL centroid frame (m)
  Eigen::Matrix3d C_u_b;   // rotation from body to USBL centroid frame (-)
  Eigen::Vector3d
      p_usTC_u;  // Pos. of transceiver transducer in USBL centroid frame (m)
  Eigen::Matrix<double, 5, 3>
      p_usH_u;  // Relative positions of hydrophone in USBL centroid frame (m)
};

struct UsblFixLossModelSimParams {
  double a;
  double b;
  double c;
  double d;
};
struct UsblModelEnableSettings {
  bool enable_rtt_noise_model;
  bool enable_tdoa_noise_model;
  bool enable_rtt_quantization_model;
  bool enable_tdoa_quantization_model;
  bool enable_acoustic_path_delay;
  bool enable_fix_loss_rate_model;  // NOLINTNEXTLINE(readability/todo)
  bool enable_fix_outlier_model;    // TODO: implement
  bool enable_ahrs_noise_model;
};

struct UsblMeasurement {
  double round_trip_time;                              // (s)
  Eigen::Vector<double, 10> timeDifferencesOfArrival;  // (s)
  Eigen::Vector3d ahrsEulRpyAngles;                    // (rad)
  Eigen::Vector3d directionVector;                     // (-)
  Eigen::Vector3d p_nu_u;                              // (m)
  Eigen::Vector3d p_nu_n;                              // (m)
  Eigen::Vector3d p_nu_u_spherical;  // range, elevation, azimuth (m, rad, rad)
  Eigen::Vector3d p_nu_n_spherical;  // range, elevation, azimuth (m, rad, rad)
  double accuracy;                   // (m)
  double accuracy_spherical;         // (rad)
  bool is_valid;                     //  measurement valid flag (-)
};

}  // namespace usbl_simulator
