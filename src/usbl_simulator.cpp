/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#include <algorithm>
#include <iomanip>
#include <iostream>

#include "usbl_simulator.hpp"

namespace usbl_simulator {

/**
 * @brief Default constructor for the UsblSimulator class.
 * 
 * This constructor initializes the USBL simulator with default parameters.
 * The USBL transceiver geometry matrix Tplus is calculated with given
 * parameters.
 * 
 * @return An instance of the UsblSimulator class initialized with default
 *         parameters.
*/
UsblSimulator::UsblSimulator()
    : randomNumberGenerator_(42), seed_(42), use_fixed_random_numbers_(false) {
  // Set default USBL simulation parameters
  usblSimParams_.speed_of_sound = 1500.0;
  usblSimParams_.max_range = 500.0;
  usblSimParams_.rtt_std_dev = 0.01 / 1500;
  usblSimParams_.tdoa_std_dev = 2.60296781192103e-07;
  usblSimParams_.rtt_resolution = 1.0e-06;
  usblSimParams_.tdoa_resolution = 1.0e-06;
  usblSimParams_.usblAhrsNoiseStdDev =
      Eigen::Vector3d(0.2, 0.2, 1.0) * M_PI / 180;
  usblSimParams_.p_bu_b = Eigen::Vector3d(0.0, 0.0, 0.0);
  usblSimParams_.C_u_b = Eigen::Matrix3d::Identity();
  usblSimParams_.p_usTC_u = Eigen::Vector3d(-80e-3, 0, 39.904e-3);
  usblSimParams_.p_usH_u << -0.022275, -0.022275, -0.014696, -0.022275,
      0.022275, -0.014696, 0.022275, -0.022275, -0.014696, 0.022275, 0.022275,
      -0.014696, 0.0, 0.0, 0.058784;

  // Set default USBL model enable settings
  usblModelEnableSettings_.enable_rtt_noise_model = true;
  usblModelEnableSettings_.enable_tdoa_noise_model = true;
  usblModelEnableSettings_.enable_rtt_quantization_model = true;
  usblModelEnableSettings_.enable_tdoa_quantization_model = true;
  usblModelEnableSettings_.enable_acoustic_path_delay = true;
  usblModelEnableSettings_.enable_fix_loss_rate_model = true;
  usblModelEnableSettings_.enable_fix_outlier_model = false;
  usblModelEnableSettings_.enable_ahrs_noise_model = true;

  // Set default USBL fix loss model simulation parameters
  usblFixLossModelSimParams_.a = -6.07039718341056;
  usblFixLossModelSimParams_.b = 0.00212458825600739;
  usblFixLossModelSimParams_.c = 5.98749360969012;
  usblFixLossModelSimParams_.d = 0.00225378907075506;

  // Calculate USBL transceiver geometry matrix Tplus
  calcTransceiverGeometryMatrix();
}

/**
 * @brief Constructor for the UsblSimulator class with given parameters.
 * 
 * This constructor initializes the USBL simulator with given parameters.
 * The USBL transceiver geometry matrix Tplus is calculated with given
 * parameters.
 * 
 * @param[in] usblSimParams              USBL simulation parameters.
 * @param[in] usblFixLossModelSimParams  USBL fix loss model simulation
 *                                       parameters.
 * @param[in] usblModelEnableSettings    USBL model enable settings.
 * @param[in] seed                       Random number generator seed.
 * 
 * @return An instance of the UsblSimulator class initialized with given
 *         parameters.
*/
UsblSimulator::UsblSimulator(
    UsblSimParams usblSimParams,
    UsblFixLossModelSimParams usblFixLossModelSimParams,
    UsblModelEnableSettings usblModelEnableSettings, unsigned int seed)
    : usblSimParams_(usblSimParams),
      usblFixLossModelSimParams_(usblFixLossModelSimParams),
      usblModelEnableSettings_(usblModelEnableSettings),
      randomNumberGenerator_(seed),
      seed_(seed),
      use_fixed_random_numbers_(false) {
  // Calculate geometry matrix
  calcTransceiverGeometryMatrix();
}

/**
 * @brief Destructor for the UsblSimulator class.
 *
 * This destructor is responsible for cleaning up the resources used by the
 * DVL simulator.
 *
 * @return None
 */
UsblSimulator::~UsblSimulator() {}

/**
 * @brief Generates a USBL measurement based on the given transceiver position,
 *        transceiver attitude, and transponder position.
 * 
 * This function generates a USBL measurement based on the given transceiver
 * position, transceiver attitude, and transponder position. For the standard
 * USBL configuration, the transponder is on the vehicle. The transceiver is
 * fixed on the air/water or ice/water interface. For the inverse USBL
 * configuration (TODO: implement), the transceiver is on the vehicle and the
 * transponder is fixed on the air/water or ice/water interface. The function
 * calls multiple subfunctions to calculate the round trip time (RTT)
 * measurement, the time differences of arrival (TDOA) measurement, the
 * direction vector measurement, and the AHRS measurement. All these
 * measurements are used to generate the position fix represented in the USBL
 * centroid frame (USBLLONG message). The AHRS measurement is used to
 * transform the position fix to the NED frame. Additionally, the position fix
 * is converted to spherical coordinates to mimic USBLANGLES message. The
 * accuracy (maximum standard deviation of all three coordinates) is provided
 * (see USBLLONG message) as a single scalar value. Depending on the Euclidean
 * distance between the transceiver and transponder, the measurement is marked
 * as valid or invalid (see USBLLONG message). This mimics the exponential
 * loss of the USBL fix due to the maximum operating range of the USBL system.
 * 
 * TODO: implement lever arm and rotation transformation for transponder.
 * 
 * @param[in] p_nb_n     Position of the body frame that the transceiver is
 *                       attached to w.r.t. the NED frame, represented in the
 *                       NED frame.
 * @param[in] q_b_n      Attitude of the body frame that the transceiver is
 *                       attached to w.r.t. the NED frame.
 * @param[in] p_nsTP_n   Position of the transponder modem transducer sensor
 *                       frame (sTP) w.r.t. the NED frame, represented in the
 *                       NED frame.
 * 
 * @return Generated USBL measurement structure.
*/
UsblMeasurement UsblSimulator::generateUsblMeasurement(
    const Eigen::Vector3d& p_nb_n, const Eigen::Quaterniond& q_b_n,
    const Eigen::Vector3d& p_nsTP_n) {
  UsblMeasurement usblMeasurement;

  // Position of USBL centroid w.r.t. body, represented in body
  Eigen::Vector3d p_bu_b = usblSimParams_.p_bu_b;

  // Position of TC modem w.r.t. USBL centroid, represented in USBL centroid
  Eigen::Vector3d p_usTC_u = usblSimParams_.p_usTC_u;

  // Rotation matrix from body to NED and vice versa
  Eigen::Matrix3d C_b_n = q_b_n.toRotationMatrix();
  Eigen::Matrix3d C_n_b = C_b_n.transpose();

  // Rotation matrix from USBL centroid to body and vice versa
  Eigen::Matrix3d C_u_b = usblSimParams_.C_u_b;
  Eigen::Matrix3d C_b_u = C_u_b.transpose();

  // Position of USBL centroid w.r.t. NED, represented in NED
  Eigen::Vector3d p_nu_n = p_nb_n + C_b_n * p_bu_b;

  // Position of USBL centroid w.r.t. NED, represented in USBL centroid
  Eigen::Vector3d p_nu_u = C_u_b * C_n_b * p_nu_n;

  // Position of transceiver modem w.r.t. NED, represented in NED
  Eigen::Vector3d p_nsTC_n = p_nu_n + C_b_n * C_u_b * p_usTC_u;

  // Position of transceiver modem w.r.t. NED, represented in USBL centroid
  Eigen::Vector3d p_nsTC_u = C_b_u * C_n_b * p_nsTC_n;

  // Position of transponder modem w.r.t. NED, represented in USBL centroid
  Eigen::Vector3d p_nsTP_u = C_b_u * C_n_b * p_nsTP_n;

  // Calculate round trip time (RTT) measurement
  usblMeasurement.round_trip_time = calcRttMeasurement(p_nsTC_u, p_nsTP_u);

  // Calculate time differences of arrival (TDOA) measurement
  usblMeasurement.timeDifferencesOfArrival =
      calcTdoaMeasurement(p_nu_u, p_nsTP_u);

  // Calculate direction vector measurement
  usblMeasurement.directionVector =
      calcDirectionVector(usblMeasurement.timeDifferencesOfArrival);

  // Calculate AHRS measurement
  Eigen::Matrix3d C_b_n_ahrs;
  if (usblModelEnableSettings_.enable_ahrs_noise_model) {
    usblMeasurement.ahrsEulRpyAngles = calcAhrsMeasurement();
    C_b_n_ahrs =
        transformEulerAnglesToRotationMatrix(usblMeasurement.ahrsEulRpyAngles);
  } else {
    usblMeasurement.ahrsEulRpyAngles = Eigen::Vector3d::Zero();
    C_b_n_ahrs = Eigen::Matrix3d::Identity();
  }

  // Calculate position fix represented in USBL centroid frame
  usblMeasurement.p_nu_u = calcPositionFix(usblMeasurement.round_trip_time,
                                           usblMeasurement.directionVector);

  // Calculate position fix represented in NED frame
  usblMeasurement.p_nu_n = C_b_n_ahrs * C_u_b * usblMeasurement.p_nu_u;

  // Convert position fixes to spherical coordinates
  usblMeasurement.p_nu_u_spherical =
      transformCartesianToSpherical(usblMeasurement.p_nu_u);
  usblMeasurement.p_nu_n_spherical =
      transformCartesianToSpherical(usblMeasurement.p_nu_n);

  // Calculate position fix accuracy in Cartesian coordinates
  usblMeasurement.accuracy = calcAccuracyPositionFixCartesian(
      usblMeasurement.round_trip_time, usblMeasurement.p_nu_u,
      usblMeasurement.ahrsEulRpyAngles);

  // Calculate position fix accuracy in spherical coordinates
  usblMeasurement.accuracy_spherical = calcAccuracyPositionFixSpherical(
      usblMeasurement.p_nu_u_spherical, usblMeasurement.accuracy);

  // Check if USBL measurement is valid
  usblMeasurement.is_valid = true;
  if (usblModelEnableSettings_.enable_fix_loss_rate_model) {
    usblMeasurement.is_valid = !isUsblOutage(p_nsTC_n, p_nsTP_n);
  }

  // Set last generated USBL measurement
  usblMeasurement_ = usblMeasurement;

  return usblMeasurement;
}

/**
 * @brief Getter function for the last generated USBL measurement.
 * 
 * @return Last generated USBL measurement.
 */
UsblMeasurement UsblSimulator::getUsblMeasurement() const {
  return usblMeasurement_;
}

/** 
 * @brief Getter function for the USBL simulation parameters.
 * 
 * @return USBL simulation parameters.
*/
UsblSimParams UsblSimulator::getUsblSimParams() const {
  return usblSimParams_;
}

/**
 * @brief Getter function for the USBL model enable settings.
 * 
 * @return USBL model enable settings.
*/
UsblModelEnableSettings UsblSimulator::getUsblModelEnableSettings() const {
  return usblModelEnableSettings_;
}

/**
 * @brief Getter function for the transceiver modem position.
 *  
 * @return Transceiver modem position.
*/
Eigen::Vector3d UsblSimulator::getTransceiverModemPosition() const {
  return usblSimParams_.p_usTC_u;
}

/**
 * @brief Getter function for the transceiver hydrophone positions.
 * 
 * @return Transceiver hydrophone positions.
*/
Eigen::Matrix<double, 5, 3> UsblSimulator::getTransceiverHydroPositions()
    const {
  return usblSimParams_.p_usH_u;
}

/**
 * @brief Getter function for the Moore-Penrose pseudoinverse of the geometry matrix.
 * 
 * @return Moore-Penrose pseudoinverse of the geometry matrix.
*/
Eigen::Matrix<double, 3, 10> UsblSimulator::getTplus() const {
  return Tplus_;
}

/**
 * @brief Setter function for the USBL simulation parameters.
 * 
 * @param[in] usblSimParams USBL simulation parameters.
*/
void UsblSimulator::setUsblSimParams(const UsblSimParams& usblSimParams) {
  usblSimParams_ = usblSimParams;

  // Recalculate geometry matrix
  calcTransceiverGeometryMatrix();
}

/**
 * @brief Setter function for the USBL fix loss model simulation parameters.
 * 
 * @param[in] usblFixLossModelSimParams USBL fix loss model simulation parameters.
*/
void UsblSimulator::setUsblFixLossModelSimParams(
    const UsblFixLossModelSimParams& usblFixLossModelSimParams) {
  usblFixLossModelSimParams_ = usblFixLossModelSimParams;
}

/**
 * @brief Setter function for the USBL model enable settings.
 * 
 * @param[in] usblModelEnableSettings USBL model enable settings.
*/
void UsblSimulator::setUsblModelEnableSettings(
    const UsblModelEnableSettings& usblModelEnableSettings) {
  usblModelEnableSettings_ = usblModelEnableSettings;
}

/**
 * @brief Setter function for the enable RTT noise model.
 * 
 * @param[in] enable_rtt_noise_model enable flag for RTT noise model.
*/
void UsblSimulator::setEnableRttNoiseModel(const bool enable_rtt_noise_model) {
  usblModelEnableSettings_.enable_rtt_noise_model = enable_rtt_noise_model;
}

/**
 * @brief Setter function for the enable TDOA noise model.
 * 
 * @param[in] enable_tdoa_noise_model enable flag for TDOA noise model.
*/
void UsblSimulator::setEnableTdoaNoiseModel(
    const bool enable_tdoa_noise_model) {
  usblModelEnableSettings_.enable_tdoa_noise_model = enable_tdoa_noise_model;
}

/**
 * @brief Setter function for the enable RTT quantization model.
 * 
 * @param[in] enable_rtt_quantization_model enable flag for RTT quantization model.
*/
void UsblSimulator::setEnableRttQuantizationModel(
    const bool enable_rtt_quantization_model) {
  usblModelEnableSettings_.enable_rtt_quantization_model =
      enable_rtt_quantization_model;
}

/**
 * @brief Setter function for the enable TDOA quantization model.
 * 
 * @param[in] enable_tdoa_quantization_model enable flag for TDOA quantization model.
*/
void UsblSimulator::setEnableTdoaQuantizationModel(
    const bool enable_tdoa_quantization_model) {
  usblModelEnableSettings_.enable_tdoa_quantization_model =
      enable_tdoa_quantization_model;
}

/**
 * @brief Setter function for the enable acoustic path delay.
 * 
 * @param[in] enable_acoustic_path_delay enable flag for acoustic path delay.
*/
void UsblSimulator::setEnableAcousticPathDelay(
    const bool enable_acoustic_path_delay) {
  usblModelEnableSettings_.enable_acoustic_path_delay =
      enable_acoustic_path_delay;
}

/**
 * @brief Setter function for the enable fix loss rate model.
 * 
 * @param[in] enable_fix_loss_rate_model enable flag for fix loss rate model.
*/
void UsblSimulator::setEnableFixLossRateModel(
    const bool enable_fix_loss_rate_model) {
  usblModelEnableSettings_.enable_fix_loss_rate_model =
      enable_fix_loss_rate_model;
}

/**
 * @brief Setter function for the enable fix outlier model.
 * 
 * @param[in] enable_fix_outlier_model enable flag for fix outlier model.
*/
void UsblSimulator::setEnableFixOutlierModel(
    const bool enable_fix_outlier_model) {
  usblModelEnableSettings_.enable_fix_outlier_model = enable_fix_outlier_model;
}

/**
 * @brief Setter function for the enable AHRS noise model.
 * 
 * @param[in] enable_ahrs_noise_model enable flag for AHRS noise model.
*/
void UsblSimulator::setEnableAhrsNoiseModel(
    const bool enable_ahrs_noise_model) {
  usblModelEnableSettings_.enable_ahrs_noise_model = enable_ahrs_noise_model;
}

/**
 * @brief Setter function for the use fixed random numbers flag.
 * 
 * @param[in] useFixedRandomNumbers use fixed random numbers flag.
*/
void UsblSimulator::setUseFixedRandomNumbers(const bool useFixedRandomNumbers) {
  use_fixed_random_numbers_ = useFixedRandomNumbers;
}

/**
 * @brief Setter function for the random number generator seed.
 * 
 * @param[in] seed random number generator seed.
*/
void UsblSimulator::setSeed(const unsigned int seed) {
  randomNumberGenerator_.seed(seed);
  seed_ = seed;
}

/**
 * @brief Generates a string stream with the USBL simulator parameters.
 * 
 * This function generates a string stream with the USBL simulator parameters.
 * 
 * @return String stream with the USBL simulator parameters.
*/
std::stringstream UsblSimulator::printSimulatorParams() const {
  // Create stringstream to store the output
  std::stringstream ss;

  ss << "***************************************************************"
        "********************************************************************"
        "**"
        "*"
     << "\n";
  ss << std::left << std::setw(50) << "Starting USBL Simulator"
     << "\n";
  ss << "***************************************************************"
        "********************************************************************"
        "**"
        "*"
     << "\n";

  // Simulation seed
  ss << std::left << "General Settings:\n";

  ss << std::left << std::setw(50) << "Seed:" << std::fixed
     << std::setprecision(6) << seed_ << "\n";

  ss << "***************************************************************"
        "********************************************************************"
        "**"
        "*"
     << "\n";

  // USBL simulation parameters
  ss << std::left << "Simulation Parameters:\n";

  ss << std::fixed << std::setprecision(6);

  ss << std::left << std::setw(50)
     << "speed_of_sound:" << usblSimParams_.speed_of_sound << " m/s\n";

  ss << std::left << std::setw(50) << "max_range:" << usblSimParams_.max_range
     << " m\n";

  ss << std::left << std::setw(50)
     << "rtt_std_dev:" << usblSimParams_.rtt_std_dev * 1e06 << " µs\n";

  ss << std::left << std::setw(50)
     << "tdoa_std_dev:" << usblSimParams_.tdoa_std_dev * 1e06 << " µs\n";

  ss << std::left << std::setw(50)
     << "rtt_resolution:" << usblSimParams_.rtt_resolution * 1e06 << " µs\n";

  ss << std::left << std::setw(50)
     << "tdoa_resolution:" << usblSimParams_.tdoa_resolution * 1e06 << " µs\n";

  ss << std::left << std::setw(50) << "usblAhrsNoiseStdDev:"
     << usblSimParams_.usblAhrsNoiseStdDev.transpose() * 180.0 / M_PI
     << " deg\n";

  ss << std::left << std::setw(50)
     << "p_bu_b:" << usblSimParams_.p_bu_b.transpose() << " m\n";

  ss << std::left << std::setw(50) << "C_u_b as RPY angle:"
     << usblSimParams_.C_u_b.eulerAngles(2, 1, 0).transpose() * 180 / M_PI
     << " deg\n";

  ss << "***************************************************************"
        "********************************************************************"
        "**"
        "*"
     << "\n";

  // USBL fix loss model settings
  ss << std::left << "Fix Loss Model Settings:\n";

  ss << std::left << std::setw(50) << "a:" << usblFixLossModelSimParams_.a
     << "\n";

  ss << std::left << std::setw(50) << "b:" << usblFixLossModelSimParams_.b
     << "\n";

  ss << std::left << std::setw(50) << "c:" << usblFixLossModelSimParams_.c
     << "\n";

  ss << std::left << std::setw(50) << "d:" << usblFixLossModelSimParams_.d
     << "\n";

  ss << "***************************************************************"
        "********************************************************************"
        "**"
        "*"
     << "\n";

  // USBL model enable settings
  ss << std::left << "Model Enable Settings:\n";

  ss << std::left << std::setw(50) << "enable_rtt_noise_model:"
     << usblModelEnableSettings_.enable_rtt_noise_model << "\n";

  ss << std::left << std::setw(50) << "enable_tdoa_noise_model:"
     << usblModelEnableSettings_.enable_tdoa_noise_model << "\n";

  ss << std::left << std::setw(50) << "enable_rtt_quantization_model:"
     << usblModelEnableSettings_.enable_rtt_quantization_model << "\n";

  ss << std::left << std::setw(50) << "enable_tdoa_quantization_model:"
     << usblModelEnableSettings_.enable_tdoa_quantization_model << "\n";

  ss << std::left << std::setw(50) << "enable_acoustic_path_delay:"
     << usblModelEnableSettings_.enable_acoustic_path_delay << "\n";

  ss << std::left << std::setw(50) << "enable_fix_loss_rate_model:"
     << usblModelEnableSettings_.enable_fix_loss_rate_model << "\n";

  ss << std::left << std::setw(50) << "enable_fix_outlier_model:"
     << usblModelEnableSettings_.enable_fix_outlier_model << "\n";

  ss << std::left << std::setw(50) << "enable_ahrs_noise_model:"
     << usblModelEnableSettings_.enable_ahrs_noise_model << "\n";

  ss << "***************************************************************"
        "********************************************************************"
        "**"
        "*"
     << "\n";

  return ss;
}

/**
 * @brief Calculate the geometry matrix for the USBL transceiver.
 * 
 * This function calculates the Moore-Penrose pseudoinverse of the geometry
 * matrix for the USBL transceiver. The geometry matrix is composed of the 
 * difference matrix D and the matrix that contains the positions of the 
 * hydrophone transducers in the USBL centroid frame. The difference matrix 
 * is used to construct the TDOA measurement.
 * 
 * @return Moore-Penrose pseudoinverse of the geometry matrix.
*/
void UsblSimulator::calcTransceiverGeometryMatrix() {
  // Define difference matrix
  Eigen::Matrix<double, 10, 5> D;
  D.row(0) << 1, -1, 0, 0, 0;
  D.row(1) << 1, 0, -1, 0, 0;
  D.row(2) << 1, 0, 0, -1, 0;
  D.row(3) << 1, 0, 0, 0, -1;
  D.row(4) << 0, 1, -1, 0, 0;
  D.row(5) << 0, 1, 0, -1, 0;
  D.row(6) << 0, 1, 0, 0, -1;
  D.row(7) << 0, 0, 1, -1, 0;
  D.row(8) << 0, 0, 1, 0, -1;
  D.row(9) << 0, 0, 0, 1, -1;

  // Calculate geometry matrix
  Eigen::Matrix<double, 10, 3> T = D * usblSimParams_.p_usH_u;

  // Calculate Moore-Penrose pseudo-inverse for least squares solution
  Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix<double, 10, 3>>
      completeOrthogonalDecomposition(T);

  Tplus_ = completeOrthogonalDecomposition.pseudoInverse();
}

/**
 * @brief Calculates the round trip time (RTT) measurement.
 * 
 * This function calculates the round trip time (RTT) measurement based on the
 * position of the tranceiver modem transducer and the transponder modem
 * transducer position within the USBL centroid frame as representation frame.
 * The transducer transducer refers to the (spherical) piezo element which
 * emits/receives sound for the transit time measurement. The representation
 * frame is arbitrary as long as the same frame is used for both positions.
 * The RTT is twice the Euclidean distance between the two positions divided by
 * the speed of sound. The RTT is then corrupted by a normal distributed noise
 * with zero mean with given RTT standard deviation. The RTT is then quantized
 * to its resolution.
 * 
 * @param[in] p_nsTC_u Position of the transceiver modem transducer sensor
 *                     frame (sTC) w.r.t. the NED (n) frame, represented in the
 *                     USBL centroid (u) frame.
 * @param[in] p_nsTP_u Position of the transponder modem transducer sensor
 *                     frame (sTP) w.r.t. the NED (n) frame, represented in the
 *                     USBL centroid (u) frame.
 *
 * @return Round trip time (RTT) measurement.
 */
double UsblSimulator::calcRttMeasurement(const Eigen::Vector3d& p_nsTC_u,
                                         const Eigen::Vector3d& p_nsTP_u) {
  // Extract speed of sound from USBL parameters
  double c = usblSimParams_.speed_of_sound;

  // Calculate true round trip time (RTT)
  double rtt_true = 2 * (p_nsTC_u - p_nsTP_u).norm() / c;

  // Declare random number
  double random_number;

  if (use_fixed_random_numbers_) {
    // Draw fixed random numbers for testing/debugging
    random_number = 0.5;
  } else {
    // Draw random number for normal distributed noise with zero mean
    random_number = drawRandNormalDistNum();
  }

  // Set random number to zero if noise model is disabled
  if (!usblModelEnableSettings_.enable_rtt_noise_model) {
    random_number = 0;
  }

  // Calculate normal distributed RTT noise
  double rtt_noise = usblSimParams_.rtt_std_dev * random_number;

  // Add noise to true RTT to obtain measured RTT
  double rtt_measurement = rtt_true + rtt_noise;

  // Quantize RTT measurement to its resolution
  if (usblModelEnableSettings_.enable_rtt_quantization_model) {
    rtt_measurement =
        std::round(rtt_measurement / usblSimParams_.rtt_resolution) *
        usblSimParams_.rtt_resolution;
  }

  return rtt_measurement;
}

/**
 * @brief Calculates the time differences of arrival (TDOA) measurement.
 * 
 * This function calculates the time differences of arrival (TDOA) measurement
 * based on the position of the five tranceiver hydrophone transducer positions
 * and the transponder modem transducer position within the USBL centroid frame
 * as representation frame. A transponder modem transducer refers to the
 * (spherical) piezo element which emits the sound for the TDOA measurement. The
 * transceiver hydrophone transducers just receive the sound to measure the TDOA.
 * The representation frame is arbitrary as long as the same frame is used for
 * both positions. The TDOA measurement is composed of the differences of the
 * propagation times between the transponder modem transducer and the five
 * transceiver hydrophone transducers. Hence ten combinations between the five
 * hydrophone transducers are calculated. Each TDOA combination is then corrupted
 * by a normal distributed noise with zero mean with given TDOA standard deviation.
 * The TDOA is then quantized to its resolution.
 * 
 * @param[in] p_nu_u Position of the USBL centroid (u) w.r.t. the NED (n) frame,
 *                   represented in the USBL centroid (u) frame.
 * @param[in] p_nsTP_u Position of the transponder modem transducer sensor frame
 *                     (sTP) w.r.t. the NED (n) frame, represented in the USBL
 *                     centroid (u) frame. 
 *
 * @return Time differences of arrival (TDOA) measurement.
*/
Eigen::Vector<double, 10> UsblSimulator::calcTdoaMeasurement(
    const Eigen::Vector3d& p_nu_u, const Eigen::Vector3d& p_nsTP_u) {
  // Extract speed of sound from USBL parameters
  double c = usblSimParams_.speed_of_sound;

  // Transform five transceiver hydrophone positions w.r.t. NED frame
  Eigen::Vector3d p_nsH1_u = p_nu_u + usblSimParams_.p_usH_u.row(0).transpose();
  Eigen::Vector3d p_nsH2_u = p_nu_u + usblSimParams_.p_usH_u.row(1).transpose();
  Eigen::Vector3d p_nsH3_u = p_nu_u + usblSimParams_.p_usH_u.row(2).transpose();
  Eigen::Vector3d p_nsH4_u = p_nu_u + usblSimParams_.p_usH_u.row(3).transpose();
  Eigen::Vector3d p_nsH5_u = p_nu_u + usblSimParams_.p_usH_u.row(4).transpose();

  // Calculate propagation times between TC hydrophones and TP modem
  double t1 = (p_nsH1_u - p_nsTP_u).norm() / c;
  double t2 = (p_nsH2_u - p_nsTP_u).norm() / c;
  double t3 = (p_nsH3_u - p_nsTP_u).norm() / c;
  double t4 = (p_nsH4_u - p_nsTP_u).norm() / c;
  double t5 = (p_nsH5_u - p_nsTP_u).norm() / c;

  // Calculate time differences between H1 and Hj
  double d12 = t1 - t2;
  double d13 = t1 - t3;
  double d14 = t1 - t4;
  double d15 = t1 - t5;

  // Calculate time differences between H2 and Hj
  double d23 = t2 - t3;
  double d24 = t2 - t4;
  double d25 = t2 - t5;

  // Calculate time differences between H3 and Hj
  double d34 = t3 - t4;
  double d35 = t3 - t5;

  // Calculate time differences between H4 and Hj
  double d45 = t4 - t5;

  // Populate vector with true time differences of arrival
  Eigen::Vector<double, 10> tdoaTrue;
  tdoaTrue << d12, d13, d14, d15, d23, d24, d25, d34, d35, d45;

  // Declare vector of random numbers
  Eigen::Vector<double, 10> randomNumbers;

  if (use_fixed_random_numbers_) {
    // Draw fixed random numbers for testing/debugging
    randomNumbers << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0;
  } else {
    // Draw random number for normal distributed noise with zero mean
    for (int i = 0; i < 10; i++) {
      randomNumbers(i) = drawRandNormalDistNum();
    }
  }

  // Set random numbers to zero if noise model is disabled
  if (!usblModelEnableSettings_.enable_tdoa_noise_model) {
    randomNumbers.setZero();
  }

  // Calculate normal distributed TDOA noise
  Eigen::Vector<double, 10> tdoaNoise =
      usblSimParams_.tdoa_std_dev * randomNumbers;

  // Add noise to true TDOA to obtain measured TDOA
  Eigen::Vector<double, 10> tdoaMeasurement;
  tdoaMeasurement = tdoaTrue + tdoaNoise;

  // Quantize TDOA measurements to their resolution
  if (usblModelEnableSettings_.enable_tdoa_quantization_model) {
    tdoaMeasurement =
        calcQuantizationModel(tdoaMeasurement, usblSimParams_.tdoa_resolution);
  }

  return tdoaMeasurement;
}

/**
 * @brief Calculates the AHRS for standard USBL (non-inverse configuration).
 * 
 * This function calculates the AHRS measurement for the standard USBL
 * configuration. The transceiver can be equipped with an internal IMU and an
 * AHRS to measure its own attitude. This function creates a simple normal
 * distributed AHRS measurement with zero mean and given standard deviation. The
 * model can be enabled or disabled, depending on the USBL model enable settings.
 * 
 * @return AHRS measurement as RPY Euler angles (ZYX convention).
*/
Eigen::Vector3d UsblSimulator::calcAhrsMeasurement() {
  Eigen::Vector3d ahrsMeasurement;

  // Declare vector of random numbers
  Eigen::Vector3d randomNumbers;

  if (use_fixed_random_numbers_) {
    // Draw fixed random numbers for testing/debugging
    randomNumbers << 1.0, 2.0, 3.0;
  } else {
    // Draw random number for normal distributed noise with zero mean
    for (int i = 0; i < 3; i++) {
      randomNumbers(i) = drawRandNormalDistNum();
    }
  }

  if (usblModelEnableSettings_.enable_ahrs_noise_model) {
    // Calculate AHRS measurement with normal distributed noise
    for (int i = 0; i < 3; i++) {
      ahrsMeasurement(i) =
          usblSimParams_.usblAhrsNoiseStdDev(i) * randomNumbers(i);
    }
  } else {
    // Set random numbers to zero if noise model is disabled
    ahrsMeasurement = Eigen::Vector3d::Zero();
  }

  return ahrsMeasurement;
}

/**
 * @brief Calculates the direction vector.
 * 
 * This function calculates the direction vector based on the time differences
 * of arrival (TDOA) measurement. The direction vector is calculated using the
 * planar wave approximation. The planar wave approximation assumes that the
 * wavefront is planar and the wavefront is perpendicular to the direction
 * vector.
 * 
 * @param[in] tdoaMeasurement Time differences of arrival (TDOA) measurement.
 * 
 * @return Direction vector measurement.
*/
Eigen::Vector3d UsblSimulator::calcDirectionVector(
    const Eigen::Vector<double, 10>& tdoaMeasurement) const {
  // Declare direction vector
  Eigen::Vector3d directionVectorMeasurement;

  // Extract speed of sound from USBL parameters
  double c = usblSimParams_.speed_of_sound;

  // Calculate direction vector using planar wave approximation
  directionVectorMeasurement = -c * Tplus_ * tdoaMeasurement;

  // Normalize direction vector
  directionVectorMeasurement.normalize();

  return directionVectorMeasurement;
}

/**
 * @brief Calculates the position fix.
 * 
 * This function calculates the position fix, represented in the USBL centroid
 * frame. The calculation uses the round trip time (RTT) measurement and the
 * direction vector measurement. The direction vector is calculated from the
 * time differences of arrival (TDOA) measurement.
 * 
 * @param[in] rtt_measurement Round trip time (RTT) measurement.
 * @param[in] directionVectorMeasurement Direction vector measurement.
 * 
 * @return Position fix represented in the USBL centroid frame. 
*/
Eigen::Vector3d UsblSimulator::calcPositionFix(
    const double rtt_measurement,
    const Eigen::Vector3d& directionVectorMeasurement) const {
  // Extract speed of sound from USBL parameters
  double c = usblSimParams_.speed_of_sound;

  // Calculate position fix using planar wave approximation
  Eigen::Vector3d p_nu_u_meas =
      c / 2 * rtt_measurement * directionVectorMeasurement;

  return p_nu_u_meas;
}

/**
 * @brief Calculates the accuracy (standard deviation) of the position fix in
 *        Cartesian coordinates.
 * 
 * This function calculates the accuracy (standard deviation) of the position
 * fix in Cartesian coordinates. The accuracy is calculated based on the round
 * trip time (RTT) measurement, the position fix, and the Euler angles
 * measurement of the internal USBL AHRS. The accuracy is calculated using the
 * covariance matrix of the direction vector measurement, the covariance matrix
 * of the position measurement, and the covariance matrix of the AHRS
 * measurement.
 * 
 * @param[in] rtt_measurement Round trip time (RTT) measurement.
 * @param[in] p_nu_u_meas    Position fix represented in the USBL centroid frame.
 * @param[in] eulerAngleRpyMeasurement Euler angles measurement of internal USBL
 *                                       AHRS.
 *
 * @return Accuracy of the position fix.
*/
double UsblSimulator::calcAccuracyPositionFixCartesian(
    const double rtt_measurement, const Eigen::Vector3d& p_nu_u_meas,
    const Eigen::Vector3d& eulerAngleRpyMeasurement) const {
  // Extract speed of sound from USBL parameters
  double c = usblSimParams_.speed_of_sound;

  // Define 10x10 identity matrix
  Eigen::Matrix<double, 10, 10> I = Eigen::Matrix<double, 10, 10>::Identity();

  // Calculate covariance of direction vector measurement
  Eigen::Matrix3d covMatrixDirVecMeas =
      Tplus_ *
      (c * c * I * usblSimParams_.tdoa_std_dev * usblSimParams_.tdoa_std_dev) *
      Tplus_.transpose();

  // Calculate covariance of position measurement (position fix)
  Eigen::Matrix3d covMatrixPosMeas = (c / 2) * (c / 2) * rtt_measurement *
                                     rtt_measurement * covMatrixDirVecMeas;

  // Get maximum square root of the covariance matrix which mimics USBL accuracy
  double pos_meas_accuracy = std::sqrt(covMatrixPosMeas.diagonal().maxCoeff());

  // Override covariance matrix with given accuracy to mimic USBL accuracy
  covMatrixPosMeas =
      pos_meas_accuracy * pos_meas_accuracy * Eigen::Matrix3d::Identity();

  Eigen::Matrix3d covMatrixPosMeasFinal;

  if (usblModelEnableSettings_.enable_ahrs_noise_model) {
    // Set covariance matrix for AHRS measurement
    Eigen::Matrix3d covMatrixAhrsUsbl;
    covMatrixAhrsUsbl(0, 0) = usblSimParams_.usblAhrsNoiseStdDev(0) *
                              usblSimParams_.usblAhrsNoiseStdDev(0);
    covMatrixAhrsUsbl(1, 1) = usblSimParams_.usblAhrsNoiseStdDev(1) *
                              usblSimParams_.usblAhrsNoiseStdDev(1);
    covMatrixAhrsUsbl(2, 2) = usblSimParams_.usblAhrsNoiseStdDev(2) *
                              usblSimParams_.usblAhrsNoiseStdDev(2);

    // Create covariance matrix for position and AHRS covariance
    Eigen::Matrix<double, 6, 6> covMatrixPosAhrs =
        Eigen::Matrix<double, 6, 6>::Zero();

    // Fill covariance matrix with position and AHRS covariance
    covMatrixPosAhrs.block<3, 3>(0, 0) = covMatrixPosMeas;
    covMatrixPosAhrs.block<3, 3>(3, 3) = covMatrixAhrsUsbl;

    // Calculate Jacobian for additional AHRS noise
    Eigen::Matrix<double, 3, 6> J_ahrs =
        calcJacobianAhrsNoise(p_nu_u_meas, eulerAngleRpyMeasurement);

    // Calculate final position covariance matrix with AHRS noise
    covMatrixPosMeasFinal = J_ahrs * covMatrixPosAhrs * J_ahrs.transpose();
  } else {
    // No AHRs noise added
    covMatrixPosMeasFinal = covMatrixPosMeas;
  }

  // Get maximum value of the diagonal of the covariance matrix
  double accuracy = std::sqrt(covMatrixPosMeasFinal.diagonal().maxCoeff());

  return accuracy;
}

/**
 * @brief Calculates the accuracy (standard deviation) of the position fix in
 *        spherical coordinates.
 * 
 * This function calculates the accuracy (standard deviation) of the position
 * fix in spherical coordinates. The accuracy is calculated based on the round
 * trip time (RTT) measurement, the position fix, and accuracy of the position
 * fix in Cartesian coordinates. 
 * 
 * @param[in] rtt_measurement Round trip time (RTT) measurement.
 * @param[in] p_nu_u_meas    Position fix represented in the USBL centroid frame.
 * @param[in] accuracy_position_fix_cartesian Accuracy of the position fix in
 *                                              Cartesian coordinates.
 *
 * @return Accuracy of the position fix in spherical coordinates.
*/
double UsblSimulator::calcAccuracyPositionFixSpherical(
    const Eigen::Vector3d& p_nu_u_meas,
    const double accuracy_position_fix_cartesian) const {
  // Elevation accuracy
  double accuracy_elevation =
      std::acos(p_nu_u_meas[2] / p_nu_u_meas.norm()) *
      (sqrt(accuracy_position_fix_cartesian * accuracy_position_fix_cartesian) /
       p_nu_u_meas.norm());

  // Bearing accuracy
  double accuracy_bearing =
      std::atan2(p_nu_u_meas[1], p_nu_u_meas[0]) *
      (sqrt(accuracy_position_fix_cartesian * accuracy_position_fix_cartesian) /
       p_nu_u_meas.norm());

  // Get the maximum value of the individual spherical accuracies
  double accuracy_spherical = std::max(accuracy_elevation, accuracy_bearing);

  return accuracy_spherical;
}

/**
 * @brief Calculates the Jacobian for additional AHRS noise.
 * 
 * This function calculates the Jacobian for additional AHRS noise. The Jacobian 
 * is calculated based on the position fix and the Euler angles measurement.
 * 
 * @param[in] p_nu_u Position fix represented in the USBL centroid frame.
 * @param[in] eulerAngleRpy Euler angles measurement of internal USBL AHRS.
 * 
 * @return Jacobian for additional AHRS noise.
*/
Eigen::Matrix<double, 3, 6> UsblSimulator::calcJacobianAhrsNoise(
    const Eigen::Vector3d& p_nu_u, const Eigen::Vector3d& eulerAngleRpy) const {
  // Calculate Jacobian for additional AHRS noise
  Eigen::Matrix<double, 3, 6> J;

  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

  J.block<3, 3>(0, 0) = I + calcSkewMatrix3(eulerAngleRpy);
  J.block<3, 3>(0, 3) = -calcSkewMatrix3(p_nu_u);

  return J;
}

/**
 * @brief Transforms Euler angles to rotation matrix.
 * 
 * This function transforms Euler angles to a rotation matrix. The Euler angles
 * are represented in the ZYX convention.
 * 
 * @param[in] eulerAngleRpy RPY Euler angles (ZYX convention).
 * 
 * @return Rotation matrix.
*/
Eigen::Matrix3d UsblSimulator::transformEulerAnglesToRotationMatrix(
    const Eigen::Vector3d& eulerAngleRpy) const {
  // Calculate rotation matrix from Euler angles
  Eigen::Matrix3d C;

  C = Eigen::AngleAxisd(eulerAngleRpy.z(), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(eulerAngleRpy.y(), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(eulerAngleRpy.x(), Eigen::Vector3d::UnitX());

  return C;
}

/**
 * @brief Quantization model for measurements.
 *
 * This function template quantizes the measurement vector to the resolution of the
 * resolution vector. This replicates that an USBL can only measure values with
 * a certain resolution (analog/digital conversion).
 *
 * @tparam Derived The Eigen expression type of the measurement vector.
 *                 It can be Eigen::Vector2d, Eigen::Vector3d, etc.
 * @param[in] measurement Vector of measurements.
 *
 * @return Quantized measurement vector.
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, 1>
UsblSimulator::calcQuantizationModel(
    const Eigen::MatrixBase<Derived>& measurement, double resolution) {
  // Define type of scalar
  using Scalar = typename Derived::Scalar;

  // Declare quantized measurement
  Eigen::Matrix<Scalar, Derived::RowsAtCompileTime, 1> quantizedMeasurement;

  // Quantize elements of measurement to resolution
  for (int i = 0; i < measurement.rows(); i++) {
    quantizedMeasurement(i) =
        std::round(measurement(i) / resolution) * resolution;
  }

  return quantizedMeasurement;
}

/**
 * @brief Simulates a USBL outage based on the range between transceiver and
 *        transponder.
 * 
 * This function simulates a USBL outage based on the range between the
 * transceiver and the transponder. The outage is simulated using a fix loss
 * model. The fix loss model is based on a loss probability model which is an
 * exponential function of the range between the transceiver and the
 * transponder. Four parameters [a,b,c,d] shape the exponential function. They
 * are fitted for different combinations of range and loss probability.
 * Given the loss probability for the range, a random number is drawn from a
 * uniform distribution. If the loss probability is greater than the random
 * number, a USBL outage is simulated and indicated by a boolean flag.
 * 
 * @param[in] p_nsTC_n Position of the transceiver modem transducer sensor
 *                     frame (sTC) w.r.t. the NED (n) frame.
 * @param[in] p_nsTP_n Position of the transponder modem transducer sensor
 *                     frame (sTP) w.r.t. the NED (n) frame.
 * 
 * @return Boolean flag indicating a USBL outage.
*/
bool UsblSimulator::isUsblOutage(const Eigen::Vector3d& p_nsTC_n,
                                 const Eigen::Vector3d& p_nsTP_n) {
  // Flag for USBL outage
  bool is_usbl_outage = false;

  // Calculate range between transceiver and transponder
  double range = (p_nsTC_n - p_nsTP_n).norm();

  // Get parameters for loss probability model
  double a = usblFixLossModelSimParams_.a;
  double b = usblFixLossModelSimParams_.b;
  double c = usblFixLossModelSimParams_.c;
  double d = usblFixLossModelSimParams_.d;

  // Calculate loss probability for a given range
  double loss_probability =
      std::min(a * std::exp(b * std::min(range, 800.0)) +
                   c * std::exp(d * std::min(range, 800.0)),
               0.999);

  // Simulate random USBL outage
  if (loss_probability > drawRandUniformDistNum()) {
    is_usbl_outage = true;
  }

  // Check if maximum range is exceeded
  if (range > usblSimParams_.max_range) {
    is_usbl_outage = true;
  }

  return is_usbl_outage;
}

/**
 * @brief Transform Cartesian to spherical position coordinates.
 * 
 * This function transforms a Cartesian position vector to spherical position
 * coordinates.
 * 
 * @param[in] posCart Cartesian position vector.
 * 
 * @return Spherical position coordinates.
*/
Eigen::Vector3d UsblSimulator::transformCartesianToSpherical(
    Eigen::Vector3d posCart) const {
  // Calculate spherical position vector
  Eigen::Vector3d posSph;

  // Calculate radius
  posSph(0) = posCart.norm();

  // Calculate elevation angle
  double hypotxy = posCart.head<2>().norm();
  posSph(1) = std::atan2(posCart.z(), hypotxy);

  // Calculate azimuth angle
  posSph(2) = std::atan2(posCart.y(), posCart.x());

  return posSph;
}

/**
 * @brief Calculate the skew matrix of a given vector.
 * 
 * This function calculates the skew matrix of a given vector.
 * 
 * @param v Vector.
 * 
 * @return 3x3 skew matrix of the given vector.
*/
Eigen::Matrix3d UsblSimulator::calcSkewMatrix3(const Eigen::Vector3d& v) const {
  Eigen::Matrix3d V;
  V << 0.0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0.0;
  return V;
}

/**
 * @brief Draws a random number from a normal distribution.
 *
 * This function draws a random number from a normal distribution with the given
 * mean and standard deviation.
 *
 * @return Single random number from the normal distribution.
 */
double UsblSimulator::drawRandNormalDistNum() {
  // Generate normal distributed random number
  return normalDistribution_(randomNumberGenerator_);
}

/**
 * @brief Draws a random number from a uniform distribution.
 * 
 * This function draws a random number from a uniform distribution in the
 * interval [0, 1).
 * 
 * @return Single random number from the uniform distribution.
*/
double UsblSimulator::drawRandUniformDistNum() {
  // Generate uniform distributed random number
  return uniformDistribution_(randomNumberGenerator_);
}

}  // namespace usbl_simulator
