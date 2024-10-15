/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#pragma once

#include "usbl_simulator_structures.hpp"

#include <random>

namespace usbl_simulator {

class UsblSimulator {
 public:
  /* ***************************************************************************
   * Public member functions
   ****************************************************************************/

  // Default constructor
  UsblSimulator();

  // Constructor with parameters
  UsblSimulator(UsblSimParams usblSimParams,
                UsblFixLossModelSimParams usblFixLossModelSimParams,
                UsblModelEnableSettings usblModelEnableSettings,
                unsigned int seed);

  // Destructor
  ~UsblSimulator();

  // Function to generate USBL measurement (main interface function)
  UsblMeasurement generateUsblMeasurement(const Eigen::Vector3d& p_nb_n,
                                          const Eigen::Quaterniond& q_b_n,
                                          const Eigen::Vector3d& p_nsTP_n);

  // Getter functions
  UsblMeasurement getUsblMeasurement() const;
  UsblSimParams getUsblSimParams() const;
  UsblModelEnableSettings getUsblModelEnableSettings() const;
  Eigen::Vector3d getTransceiverModemPosition() const;
  Eigen::Matrix<double, 5, 3> getTransceiverHydroPositions() const;
  Eigen::Matrix<double, 3, 10> getTplus() const;

  // Setter functions
  void setUsblSimParams(const UsblSimParams& usblSimParams);
  void setUsblFixLossModelSimParams(
      const UsblFixLossModelSimParams& usblFixLossModelSimParams);
  void setUsblModelEnableSettings(
      const UsblModelEnableSettings& usblModelEnableSettings);
  void setEnableRttNoiseModel(const bool enableRttNoiseModel);
  void setEnableTdoaNoiseModel(const bool enableTdoaNoiseModel);
  void setEnableRttQuantizationModel(const bool enableRttQuantizationModel);
  void setEnableTdoaQuantizationModel(const bool enableTdoaQuantizationModel);
  void setEnableAcousticPathDelay(const bool enableAcousticPathDelay);
  void setEnableFixLossRateModel(const bool enableFixLossRateModel);
  void setEnableFixOutlierModel(const bool enableFixOutlierModel);
  void setEnableAhrsNoiseModel(const bool enableAhrsNoiseModel);
  void setEnableInverseUsblModel(const bool enableInverseUsblModel);
  void setUseFixedRandomNumbers(const bool useFixedRandomNumbers);
  void setSeed(const unsigned int seed);

  // Print functions
  std::stringstream printSimulatorParams() const;

 private:
  /* ***************************************************************************
   * Private member functions
   ****************************************************************************/
  // Function to calculate USBL transceiver geometry
  void calcTransceiverGeometryMatrix();

  // Function to calculate USBL round trip time (RTT) measurement
  double calcRttMeasurement(const Eigen::Vector3d& p_nsTD_u,
                            const Eigen::Vector3d& p_nsTP_u);

  // Function to calculate USBL time differences of arrival (TDOA) measurement
  Eigen::Vector<double, 10> calcTdoaMeasurement(
      const Eigen::Vector3d& p_nu_u, const Eigen::Vector3d& p_nsTP_u);

  // Function to generate USBL internal AHRS measurement
  Eigen::Vector3d calcAhrsMeasurement();

  // Function to calculate USBL direction vector measurement
  Eigen::Vector3d calcDirectionVector(
      const Eigen::Vector<double, 10>& tdoaMeasurement) const;

  // Function to calculate USBL position fix (USBLLONG)
  Eigen::Vector3d calcPositionFix(
      const double rtt_measurement,
      const Eigen::Vector3d& directionVectorMeasurement) const;

  // Function to calculate USBL position fix accuracy in Cartesian coordinates
  double calcAccuracyPositionFixCartesian(
      const double rtt_measurement, const Eigen::Vector3d& p_nu_u_meas,
      const Eigen::Vector3d& eulerAngleRpyMeasurement) const;

  // Function to calculate USBL position fix accuracy in spherical coordinates
  double calcAccuracyPositionFixSpherical(
      const Eigen::Vector3d& p_nu_u_meas,
      const double accuracy_position_fix_cartesian) const;

  // Function to calculate incorporate AHRS noise into accuracy
  Eigen::Matrix<double, 3, 6> calcJacobianAhrsNoise(
      const Eigen::Vector3d& p_nu_u,
      const Eigen::Vector3d& eulerAngleRpy) const;

  // Template to calculate quantization of measurements
  template <typename Derived>
  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, 1>
  calcQuantizationModel(const Eigen::MatrixBase<Derived>& measurement,
                        double resolution);

  // Function to model USBL fix loss rate
  bool isUsblOutage(const Eigen::Vector3d& p_nsTC_n,
                    const Eigen::Vector3d& p_nsTP_n);

  // Auxiliary function to transform Euler angles to rotation matrix
  Eigen::Matrix3d transformEulerAnglesToRotationMatrix(
      const Eigen::Vector3d& eulerAngleRpy) const;

  // Auxiliary function to transform Cartesian to spherical coordinates
  Eigen::Vector3d transformCartesianToSpherical(Eigen::Vector3d posCart) const;

  // Auxiliary function to calculate skew-symmetric matrix
  Eigen::Matrix3d calcSkewMatrix3(const Eigen::Vector3d& v) const;

  // Functions to draw random numbers (normal and uniform distribution)
  double drawRandNormalDistNum();
  double drawRandUniformDistNum();

  /* ***************************************************************************
   * Private member variables
   ****************************************************************************/

  // Last generated USBL measurement
  UsblMeasurement usblMeasurement_;

  // USBL simulation parameters
  UsblSimParams usblSimParams_;

  // USBL fix loss model simulation parameters
  UsblFixLossModelSimParams usblFixLossModelSimParams_;

  // USBL model enable settings
  UsblModelEnableSettings usblModelEnableSettings_;

  // Moore-Penrose pseudoinverse of the geometry matrix
  Eigen::Matrix<double, 3, 10> Tplus_;

  // Random number generator for normal distributiona and uniform distribution
  std::mt19937 randomNumberGenerator_;
  unsigned int seed_;
  std::normal_distribution<> normalDistribution_;
  std::uniform_real_distribution<> uniformDistribution_;

  // Flag to use fixed random numbers for debugging/testing
  bool use_fixed_random_numbers_;
};

}  // namespace usbl_simulator
