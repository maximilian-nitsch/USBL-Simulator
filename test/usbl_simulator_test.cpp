/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#include "usbl_simulator.hpp"

#include "gtest/gtest.h"

/**
 * @brief Test fixture for the USBL simulator class.
*/
class UsblSimulatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Set the USBL simulation parameters
    usblSimParams.speed_of_sound = 1500.0;
    usblSimParams.max_range = 500.0;
    usblSimParams.rtt_std_dev = 0.01 / 1500;
    usblSimParams.tdoa_std_dev = 2.60296781192103e-07;
    usblSimParams.rtt_resolution = 1.0e-06;
    usblSimParams.tdoa_resolution = 1.0e-06;
    usblSimParams.usblAhrsNoiseStdDev =
        Eigen::Vector3d(0.2, 0.2, 1.0) * M_PI / 180;
    usblSimParams.p_bu_b = Eigen::Vector3d(0.0, 0.0, 0.0);
    usblSimParams.C_u_b = Eigen::Matrix3d::Identity();
    usblSimParams.p_usTC_u = Eigen::Vector3d(-80e-3, 0, 39.904e-3);
    usblSimParams.p_usH_u << -0.022275, -0.022275, -0.014696, -0.022275,
        0.022275, -0.014696, 0.022275, -0.022275, -0.014696, 0.022275, 0.022275,
        -0.014696, 0.0, 0.0, 0.058784;

    // Set the USBL enable settings
    usblModelEnableSettings.enable_rtt_noise_model = false;
    usblModelEnableSettings.enable_tdoa_noise_model = false;
    usblModelEnableSettings.enable_rtt_quantization_model = false;
    usblModelEnableSettings.enable_tdoa_quantization_model = false;
    usblModelEnableSettings.enable_acoustic_path_delay = false;
    usblModelEnableSettings.enable_fix_loss_rate_model = false;
    usblModelEnableSettings.enable_fix_outlier_model = false;
    usblModelEnableSettings.enable_ahrs_noise_model = false;

    // Set the USBL fix loss model simulation parameters
    usblFixLossModelSimParams.a = -6.07039718341056;
    usblFixLossModelSimParams.b = 0.00212458825600739;
    usblFixLossModelSimParams.c = 5.98749360969012;
    usblFixLossModelSimParams.d = 0.00225378907075506;

    // Random number generator seed
    unsigned int seed = 42;

    // Initialize the USBL simulator class
    pUsblSimulator = std::make_unique<usbl_simulator::UsblSimulator>(
        usblSimParams, usblFixLossModelSimParams, usblModelEnableSettings,
        seed);
  }

  // Simulation parameters for EvoLogics OEM USBL
  usbl_simulator::UsblSimParams usblSimParams;

  // USBL model enable settings
  usbl_simulator::UsblModelEnableSettings usblModelEnableSettings;

  // USBL fix loss model simulation parameters
  usbl_simulator::UsblFixLossModelSimParams usblFixLossModelSimParams;

  // Declare the class under test
  std::unique_ptr<usbl_simulator::UsblSimulator> pUsblSimulator;
};

/**
 * @brief Tests the transceiver hydrophone array geometry and the Moore-Penrose
 *        pseudoinverse of the geometry matrix.
 */
TEST_F(UsblSimulatorTest, TrannsceiverHydrophoneArrayGeometryTest) {
  // Get transceiver hydrophone positions
  Eigen::Matrix<double, 5, 3> p_usH_u =
      pUsblSimulator->getTransceiverHydroPositions();

  // Get Moore-Penrose pseudoinverse of the geometry matrix
  Eigen::Matrix<double, 3, 10> Tplus = pUsblSimulator->getTplus();

  // Expected transceiver hydrophone positions
  Eigen::Matrix<double, 5, 3> p_usH_u_expected;
  p_usH_u_expected << -0.022275, -0.022275, -0.014696, -0.022275, 0.022275,
      -0.014696, 0.022275, -0.022275, -0.014696, 0.022275, 0.022275, -0.014696,
      0, 0, 0.058784;

  // Expected Moore-Penrose pseudoinverse of the geometry matrix
  Eigen::Matrix<double, 3, 10> Tplus_expected;
  Tplus_expected << 0, -4.48933782267116, -4.48933782267116, -2.24466891133558,
      -4.48933782267116, -4.48933782267116, -2.24466891133558, 0.0,
      2.24466891133558, 2.24466891133558, -4.48933782267115,
      -2.78623361756147e-16, -4.48933782267116, -2.24466891133558,
      4.48933782267116, -2.78623361756147e-16, 2.24466891133558,
      -4.48933782267116, -2.24466891133558, 2.24466891133558, 0.0, 0.0, 0.0,
      -3.40228633641807, 0.0, 0.0, -3.40228633641807, 0.0, -3.40228633641807,
      -3.40228633641807;

  // Compare the hydrophone positions
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 3; j++) {
      EXPECT_NEAR(p_usH_u(i, j), p_usH_u_expected(i, j), 1e-10);
    }
  }
  // Compare the Moore-Penrose pseudoinverse of the geometry matrices
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 10; j++) {
      EXPECT_NEAR(Tplus(i, j), Tplus_expected(i, j), 1e-10);
    }
  }
}
/**
 * @brief Tests the round trip time measurement model.
 */
TEST_F(UsblSimulatorTest, RttMeasurementModelTest) {
  // Enable the RTT noise model
  pUsblSimulator->setEnableRttNoiseModel(true);
  pUsblSimulator->setUseFixedRandomNumbers(true);

  Eigen::Vector3d p_nb_n = Eigen::Vector3d(50.0, -10.0, 75.0);
  Eigen::Matrix3d C_b_n;
  C_b_n << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0;
  Eigen::Quaterniond q_b_n = Eigen::Quaterniond(C_b_n);
  Eigen::Vector3d p_nsTP_n = Eigen::Vector3d(0.0, 0.0, 0.0);

  // Generate a new USBL measurement
  usbl_simulator::UsblMeasurement usblMeasurement =
      pUsblSimulator->generateUsblMeasurement(p_nb_n, q_b_n, p_nsTP_n);

  // Expected round trip time
  double round_trip_time_expected = 0.120866880947652;

  // Compare the round trip times
  EXPECT_NEAR(usblMeasurement.round_trip_time, round_trip_time_expected, 1e-10);

  // Enable quantization model
  pUsblSimulator->setEnableRttQuantizationModel(true);

  // Generate a new USBL measurement
  usblMeasurement =
      pUsblSimulator->generateUsblMeasurement(p_nb_n, q_b_n, p_nsTP_n);

  // Expected round trip time
  round_trip_time_expected = 0.120867;

  // Compare the round trip times
  EXPECT_NEAR(usblMeasurement.round_trip_time, round_trip_time_expected, 1e-10);
}

/**
 * @brief Tests the time difference of arrival measurement model.
 */
TEST_F(UsblSimulatorTest, TdoaMeasurementModelTest) {
  // Enable the TDOA noise model
  pUsblSimulator->setEnableTdoaNoiseModel(true);
  pUsblSimulator->setUseFixedRandomNumbers(true);

  Eigen::Vector3d p_nb_n = Eigen::Vector3d(50.0, -10.0, 75.0);
  Eigen::Matrix3d C_b_n;
  C_b_n << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0;
  Eigen::Quaterniond q_b_n = Eigen::Quaterniond(C_b_n);
  Eigen::Vector3d p_nsTP_n = Eigen::Vector3d(0.0, 0.0, 0.0);

  // Generate a new USBL measurement
  usbl_simulator::UsblMeasurement usblMeasurement =
      pUsblSimulator->generateUsblMeasurement(p_nb_n, q_b_n, p_nsTP_n);

  // Expected time differences of arrival
  Eigen::Vector<double, 10> timeDifferencesOfArrivalExpected;

  timeDifferencesOfArrivalExpected(0) = -3.24975624330298e-06;
  timeDifferencesOfArrivalExpected(1) = -2.45120093060158e-05;
  timeDifferencesOfArrivalExpected(2) = -2.77604348561797e-05;
  timeDifferencesOfArrivalExpected(3) = -4.08286061182052e-05;
  timeDifferencesOfArrivalExpected(4) = -2.11581343502359e-05;
  timeDifferencesOfArrivalExpected(5) = -2.44065599003999e-05;
  timeDifferencesOfArrivalExpected(6) = -3.74747311624254e-05;
  timeDifferencesOfArrivalExpected(7) = -3.0662178033295e-06;
  timeDifferencesOfArrivalExpected(8) = -1.6134389065355e-05;
  timeDifferencesOfArrivalExpected(9) = -1.28339041589526e-05;

  // Compare the round trip times
  for (int i = 0; i < 10; i++) {
    EXPECT_NEAR(usblMeasurement.timeDifferencesOfArrival(i),
                timeDifferencesOfArrivalExpected(i), 1e-10);
  }

  // Enable quantization model
  pUsblSimulator->setEnableTdoaQuantizationModel(true);

  // Generate a new USBL measurement
  usblMeasurement =
      pUsblSimulator->generateUsblMeasurement(p_nb_n, q_b_n, p_nsTP_n);

  // Expected time differences of arrival
  timeDifferencesOfArrivalExpected(0) = -3.0e-06;
  timeDifferencesOfArrivalExpected(1) = -2.5e-05;
  timeDifferencesOfArrivalExpected(2) = -2.8e-05;
  timeDifferencesOfArrivalExpected(3) = -4.1e-05;
  timeDifferencesOfArrivalExpected(4) = -2.1e-05;
  timeDifferencesOfArrivalExpected(5) = -2.4e-05;
  timeDifferencesOfArrivalExpected(6) = -3.7e-05;
  timeDifferencesOfArrivalExpected(7) = -3.0e-06;
  timeDifferencesOfArrivalExpected(8) = -1.6e-05;
  timeDifferencesOfArrivalExpected(9) = -1.3e-05;

  // Compare the round trip times
  for (int i = 0; i < 10; i++) {
    EXPECT_NEAR(usblMeasurement.timeDifferencesOfArrival(i),
                timeDifferencesOfArrivalExpected(i), 1e-10);
  }
}

/**
 * @brief Tests the direction vector measurement model.
 */
TEST_F(UsblSimulatorTest, DirectionVectorMeasurementModelTest) {
  // Enable the RTT and TDOA noise model
  pUsblSimulator->setEnableRttNoiseModel(true);
  pUsblSimulator->setEnableRttQuantizationModel(true);
  pUsblSimulator->setEnableTdoaNoiseModel(true);
  pUsblSimulator->setEnableTdoaQuantizationModel(true);
  pUsblSimulator->setUseFixedRandomNumbers(true);

  Eigen::Vector3d p_nb_n = Eigen::Vector3d(50.0, -10.0, 75.0);
  Eigen::Matrix3d C_b_n;
  C_b_n << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0;
  Eigen::Quaterniond q_b_n = Eigen::Quaterniond(C_b_n);
  Eigen::Vector3d p_nsTP_n = Eigen::Vector3d(0.0, 0.0, 0.0);

  // Generate a new USBL measurement
  usbl_simulator::UsblMeasurement usblMeasurement =
      pUsblSimulator->generateUsblMeasurement(p_nb_n, q_b_n, p_nsTP_n);

  // Expected direction vector
  Eigen::Vector3d directionVectorExpected;
  directionVectorExpected << -0.828644446603107, -0.111613333624092,
      -0.548535181069681;

  // Compare the direction vectors
  EXPECT_NEAR(usblMeasurement.directionVector(0), directionVectorExpected(0),
              1e-10);
  EXPECT_NEAR(usblMeasurement.directionVector(1), directionVectorExpected(1),
              1e-10);
  EXPECT_NEAR(usblMeasurement.directionVector(2), directionVectorExpected(2),
              1e-10);
}

/**
 * @brief Tests the position fix model (least squares).
 */
TEST_F(UsblSimulatorTest, PositionFixTest) {
  // Enable the RTT and TDOA noise model
  pUsblSimulator->setEnableRttNoiseModel(true);
  pUsblSimulator->setEnableRttQuantizationModel(true);
  pUsblSimulator->setEnableTdoaNoiseModel(true);
  pUsblSimulator->setEnableTdoaQuantizationModel(true);
  pUsblSimulator->setEnableAhrsNoiseModel(true);
  pUsblSimulator->setUseFixedRandomNumbers(true);

  Eigen::Vector3d p_nb_n = Eigen::Vector3d(50.0, -10.0, 75.0);
  Eigen::Matrix3d C_b_n;
  C_b_n << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0;
  Eigen::Quaterniond q_b_n = Eigen::Quaterniond(C_b_n);
  Eigen::Vector3d p_nsTP_n = Eigen::Vector3d(0.0, 0.0, 0.0);

  // Generate a new USBL measurement
  usbl_simulator::UsblMeasurement usblMeasurement =
      pUsblSimulator->generateUsblMeasurement(p_nb_n, q_b_n, p_nsTP_n);

  // Expected position fix in USB centroid frame
  Eigen::Vector3d p_nu_u_expected;
  p_nu_u_expected << -75.1168262456833, -10.1177765963573, -49.7248512977619;

  // Compare the position fix in USB centroid frame
  EXPECT_NEAR(usblMeasurement.p_nu_u(0), p_nu_u_expected(0), 1e-10);
  EXPECT_NEAR(usblMeasurement.p_nu_u(1), p_nu_u_expected(1), 1e-10);
  EXPECT_NEAR(usblMeasurement.p_nu_u(2), p_nu_u_expected(2), 1e-10);

  // Expected position fix in navigation frame
  Eigen::Vector3d p_nu_n_expected;
  p_nu_n_expected << -74.8385274820465, -13.8799106676119, -49.2342432585348;

  // Compare the position fix in navigation frame
  EXPECT_NEAR(usblMeasurement.p_nu_n(0), p_nu_n_expected(0), 1e-10);
  EXPECT_NEAR(usblMeasurement.p_nu_n(1), p_nu_n_expected(1), 1e-10);
  EXPECT_NEAR(usblMeasurement.p_nu_n(2), p_nu_n_expected(2), 1e-10);

  // Expected position fix in USB centroid frame in spherical coordinates
  Eigen::Vector3d p_nu_u_spherical_expected;
  p_nu_u_spherical_expected << 90.65025, -0.580611320435469, -3.00770458105715;

  // Compare the position fix in USB centroid frame in spherical coordinates
  EXPECT_NEAR(usblMeasurement.p_nu_u_spherical(0), p_nu_u_spherical_expected(0),
              1e-10);
  EXPECT_NEAR(usblMeasurement.p_nu_u_spherical(1), p_nu_u_spherical_expected(1),
              1e-10);
  EXPECT_NEAR(usblMeasurement.p_nu_u_spherical(2), p_nu_u_spherical_expected(2),
              1e-10);

  // Expected position fix in NED frame in spherical coordinates
  Eigen::Vector3d p_nu_n_spherical_expected;
  p_nu_n_spherical_expected << 90.65025, -0.574152146349087, -2.95821153018157;

  // Compare the position fix in NED frame in spherical coordinates
  EXPECT_NEAR(usblMeasurement.p_nu_n_spherical(0), p_nu_n_spherical_expected(0),
              1e-10);
  EXPECT_NEAR(usblMeasurement.p_nu_n_spherical(1), p_nu_n_spherical_expected(1),
              1e-10);
  EXPECT_NEAR(usblMeasurement.p_nu_n_spherical(2), p_nu_n_spherical_expected(2),
              1e-10);
}

/**
 * @brief Tests the accuracy of the position fix.
 */
TEST_F(UsblSimulatorTest, AccuracyPositionFixTest) {
  // Enable the RTT and TDOA noise model
  pUsblSimulator->setEnableRttNoiseModel(true);
  pUsblSimulator->setEnableRttQuantizationModel(true);
  pUsblSimulator->setEnableTdoaNoiseModel(true);
  pUsblSimulator->setEnableTdoaQuantizationModel(true);
  pUsblSimulator->setUseFixedRandomNumbers(true);

  Eigen::Vector3d p_nb_n = Eigen::Vector3d(50.0, -10.0, 75.0);
  Eigen::Matrix3d C_b_n;
  C_b_n << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0;
  Eigen::Quaterniond q_b_n = Eigen::Quaterniond(C_b_n);
  Eigen::Vector3d p_nsTP_n = Eigen::Vector3d(0.0, 0.0, 0.0);

  // Generate a new USBL measurement
  usbl_simulator::UsblMeasurement usblMeasurement =
      pUsblSimulator->generateUsblMeasurement(p_nb_n, q_b_n, p_nsTP_n);

  // Expected position fix accuracy
  double accuracy_expected = 0.355300936631068;

  // Compare the position fix in USB centroid frame
  EXPECT_NEAR(usblMeasurement.accuracy, accuracy_expected, 1e-10);

  // Enable AHRS noise model
  pUsblSimulator->setEnableAhrsNoiseModel(true);

  // Generate a new USBL measurement
  usblMeasurement =
      pUsblSimulator->generateUsblMeasurement(p_nb_n, q_b_n, p_nsTP_n);

  // Expected position fix accuracy with AHRS noise model
  accuracy_expected = 1.3694995541513;

  // Compare the position fix in USB centroid frame
  EXPECT_NEAR(usblMeasurement.accuracy, accuracy_expected, 1e-10);
}

/**
 * @brief Tests the fix loss rate model.
 */
TEST_F(UsblSimulatorTest, FixLossRateModelTest) {
  // Enable the RTT and TDOA noise model
  pUsblSimulator->setEnableFixLossRateModel(true);

  Eigen::Vector3d p_nb_n = Eigen::Vector3d(500.0, 250.0, 100.0);
  Eigen::Matrix3d C_b_n = Eigen::Matrix3d::Identity();
  Eigen::Quaterniond q_b_n = Eigen::Quaterniond(C_b_n);
  Eigen::Vector3d p_nsTP_n = Eigen::Vector3d(0.0, 0.0, 0.0);

  // Generate a new USBL measurement
  usbl_simulator::UsblMeasurement usblMeasurement =
      pUsblSimulator->generateUsblMeasurement(p_nb_n, q_b_n, p_nsTP_n);

  // Expected valid flag
  bool valid_expected = false;

  // Compare the valid flag
  EXPECT_EQ(usblMeasurement.is_valid, valid_expected);

  // Set new position
  p_nb_n = Eigen::Vector3d(400.0, 0.0, 0.0);

  // Generate a new USBL measurement
  usblMeasurement =
      pUsblSimulator->generateUsblMeasurement(p_nb_n, q_b_n, p_nsTP_n);

  // Create a vector of valid flags of size 10000
  std::vector<bool> validFlags(10000, true);

  // Generate 10000 USBL measurements
  for (int i = 0; i < 10000; i++) {
    usblMeasurement =
        pUsblSimulator->generateUsblMeasurement(p_nb_n, q_b_n, p_nsTP_n);
    validFlags[i] = usblMeasurement.is_valid;
  }

  // Check the percentage of valid measurements
  int valid_count = std::count(validFlags.begin(), validFlags.end(), true);
  double valid_percentage = valid_count / 10000.0 * 100;

  // Check if number of valid measurements is within reasonable bounds
  EXPECT_TRUE(valid_percentage < 75.0);
  EXPECT_TRUE(valid_percentage > 25.0);
}
