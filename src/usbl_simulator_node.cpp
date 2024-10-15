/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#include "usbl_simulator_node.hpp"

namespace usbl_simulator {

UsblSimulatorNode::UsblSimulatorNode(
    std::shared_ptr<UsblSimulator> pUsblSimulator)
    : Node("usbl_simulator_node"),
      pUsblSimulator_(pUsblSimulator),
      sample_time_(1.0),
      first_odometry_received_(false),
      odometry_timeout_(false) {
  RCLCPP_INFO(this->get_logger(), "Configuring USBL simulator node...");

  // Declare and retrieve parameters and load them into the USBL simulator
  declareAndRetrieveGeneralSettings();
  declareAndRetrieveSimParams();
  declareAndRetrievFixLossSettings();
  declareAndRetrievEnableSettings();

  // Print USBL simulator parameters
  std::stringstream ss = pUsblSimulator_->printSimulatorParams();
  RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

  RCLCPP_INFO(this->get_logger(),
              "Parameters from YAML config loaded successfully.");

  // Declare the value of the topic_name parameter
  this->declare_parameter<std::string>("topic_name");

  // Retrieve the value of the topic_name parameter
  std::string topic_name = this->get_parameter("topic_name").as_string();

  // Get topic name from launch file or use default
  std::string ground_truth_topic_name;
  this->get_parameter_or("topic_name", ground_truth_topic_name,
                         std::string("/nanoauv/odometry"));

  RCLCPP_INFO(this->get_logger(),
              "Subscribing to ground truth odometry topic: %s",
              ground_truth_topic_name.c_str());

  // Initialize the odometry subscriber
  pOdometrySubscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      ground_truth_topic_name, 10,
      std::bind(&UsblSimulatorNode::odometryCallback, this,
                std::placeholders::_1));

  // Get node namespace
  std::string ns = this->get_namespace();

// Initialize the publishers
#ifdef USE_NANOAUV_SENSOR_DRIVER_INTERFACES
  pUsblLongPublisher_ =
      this->create_publisher<nanoauv_sensor_driver_interfaces::msg::UsblLong>(
          ns + "/usbllong", 10);
  pUsblAnglesPublisher_ =
      this->create_publisher<nanoauv_sensor_driver_interfaces::msg::UsblAngles>(
          ns + "/usblangles", 10);
#else
  pRoundTripTimePublisher_ = this->create_publisher<std_msgs::msg::Float64>(
      ns + "/round_trip_time", 10);
  pTimeDifferencesOfArrivalPublisher_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
          ns + "/time_differences_of_arrival", 10);
  pAhrsPublisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      ns + "/ahrs", 10);
  pDirectionVectorPublisher_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(
          ns + "/direction_vector", 10);
  pPosFixCartesianUsblFramePublisher_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(
          ns + "/pos_fix_cartesian_usbl_frame", 10);
  pPosFixCartesianNedFramePublisher_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(
          ns + "/pos_fix_cartesian_ned_frame", 10);
  pPosFixSphericalUsblFramePublisher_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(
          ns + "/pos_fix_spherical_usbl_frame", 10);
  pPosFixSphericalNedFramePublisher_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(
          ns + "/pos_fix_spherical_ned_frame", 10);
  pAccuracyPublisher_ =
      this->create_publisher<std_msgs::msg::Float64>(ns + "/accuracy", 10);
  pDiagnosticPublisher_ =
      this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          ns + "/diagnostic", 10);
#endif

  pPosFixCartesianNedFramePublisher_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(
          ns + "/pos_fix_cartesian_ned_frame", 10);

  // Initialize the tf2 broadcaster
  pStaticTf2Broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  // Publish static tf2 transformations
  this->publishStaticTf2Transforms();

  // Declare sample time parameter
  this->declare_parameter<double>(
      "usbl_simulator."
      "general_settings."
      "sample_time");

  // Retrieve sample time parameter from YAML file
  double sample_time = get_parameter(
                           "usbl_simulator."
                           "general_settings."
                           "sample_time")
                           .as_double();

  // Set class member sample time
  sample_time_ = sample_time;

  // Convert sample time to milliseconds and cast to int
  int sample_time_integer = static_cast<int>(sample_time_ * 1e3);

  RCLCPP_INFO(this->get_logger(), "USBL simulator node executing with %dms.",
              sample_time_integer);

  // Create a timer to call the USBL simulator callback function
  pTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(sample_time_integer),
      std::bind(&UsblSimulatorNode::UsblSimulatorLoopCallback, this));

  // Create timer for timeout
  pOdometryTimeOutTimer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&UsblSimulatorNode::odometryTimeOutCallback, this));

  RCLCPP_INFO(
      this->get_logger(),
      "USBL simulator node initialized. Waiting for first odometry message...");
}
/**
 * @brief Destructor for the USBL simulator node.
*/
UsblSimulatorNode::~UsblSimulatorNode() {}

/**
 * @brief Declare and retrieve general settings from the YAML file.
*/
void UsblSimulatorNode::declareAndRetrieveGeneralSettings() {
  // General settings
  int seed;
  bool use_constand_seed;

  // Declare general settings
  this->declare_parameter<int>("usbl_simulator.general_settings.seed");
  this->declare_parameter<bool>(
      "usbl_simulator.general_settings.use_constant_seed");

  // Retrieve general settings
  this->get_parameter("usbl_simulator.general_settings.seed", seed);
  this->get_parameter("usbl_simulator.general_settings.use_constant_seed",
                      use_constand_seed);

  // Set seed depending on the use_constand_seed flag
  if (use_constand_seed == false) {
    // Draw a random seed from the random device
    std::random_device randomDevice;
    seed = randomDevice();
    pUsblSimulator_->setSeed(seed);
    RCLCPP_INFO(this->get_logger(), "Using random seed: %d", seed);
  } else {
    // Set the random number generator seed
    pUsblSimulator_->setSeed(seed);
    RCLCPP_INFO(this->get_logger(), "Using seed from config file: %d", seed);
  }
}

/**
 * @brief Declare and retrieve simulation parameters from the YAML file.
*/
void UsblSimulatorNode::declareAndRetrieveSimParams() {
  // USBL model parameters
  UsblSimParams usblSimParams;

  // Declare USBL model parameters
  this->declare_parameter<double>(
      "usbl_simulator."
      "model_parameter_settings."
      "speed_of_sound");
  this->declare_parameter<double>(
      "usbl_simulator."
      "model_parameter_settings."
      "max_operating_range");
  this->declare_parameter<double>(
      "usbl_simulator."
      "model_parameter_settings."
      "rtt_std_dev");
  this->declare_parameter<double>(
      "usbl_simulator."
      "model_parameter_settings."
      "tdoa_std_dev");
  this->declare_parameter<double>(
      "usbl_simulator."
      "model_parameter_settings."
      "rtt_resolution");
  this->declare_parameter<double>(
      "usbl_simulator."
      "model_parameter_settings."
      "tdoa_resolution");
  this->declare_parameter<std::vector<double>>(
      "usbl_simulator."
      "model_parameter_settings."
      "ahrs_noise_std_dev");
  this->declare_parameter<std::vector<double>>(
      "usbl_simulator."
      "model_parameter_settings."
      "lever_arm_body_to_usbl_centroid_frame");
  this->declare_parameter<std::vector<double>>(
      "usbl_simulator."
      "model_parameter_settings."
      "rotation_body_to_usbl_centroid_frame");
  this->declare_parameter<std::vector<double>>(
      "usbl_simulator."
      "model_parameter_settings."
      "transceiver_modem_transducer_position");
  this->declare_parameter<std::vector<double>>(
      "usbl_simulator."
      "model_parameter_settings."
      "transceiver_hydrophone_positions");

  // Retrieve USBL model parameters
  usblSimParams.speed_of_sound = this->get_parameter(
                                         "usbl_simulator."
                                         "model_parameter_settings."
                                         "speed_of_sound")
                                     .as_double();
  usblSimParams.max_range = this->get_parameter(
                                    "usbl_simulator."
                                    "model_parameter_settings."
                                    "max_operating_range")
                                .as_double();
  usblSimParams.rtt_std_dev = this->get_parameter(
                                      "usbl_simulator."
                                      "model_parameter_settings."
                                      "rtt_std_dev")
                                  .as_double();
  usblSimParams.tdoa_std_dev = this->get_parameter(
                                       "usbl_simulator."
                                       "model_parameter_settings."
                                       "tdoa_std_dev")
                                   .as_double();
  usblSimParams.rtt_resolution = this->get_parameter(
                                         "usbl_simulator."
                                         "model_parameter_settings."
                                         "rtt_resolution")
                                     .as_double();
  usblSimParams.tdoa_resolution = this->get_parameter(
                                          "usbl_simulator."
                                          "model_parameter_settings."
                                          "tdoa_resolution")
                                      .as_double();
  std::vector<double> ahrsNoiseStdDev = this->get_parameter(
                                                "usbl_simulator."
                                                "model_parameter_settings."
                                                "ahrs_noise_std_dev")
                                            .as_double_array();
  std::vector<double> leverArmBodyToUsblCentroidFrame =
      this->get_parameter(
              "usbl_simulator."
              "model_parameter_settings."
              "lever_arm_body_to_usbl_centroid_frame")
          .as_double_array();
  std::vector<double> rotationBodyToUsblCentroidFrame =
      this->get_parameter(
              "usbl_simulator."
              "model_parameter_settings."
              "rotation_body_to_usbl_centroid_frame")
          .as_double_array();
  std::vector<double> transceiverModemTransducerPosition =
      this->get_parameter(
              "usbl_simulator."
              "model_parameter_settings."
              "transceiver_modem_transducer_position")
          .as_double_array();
  std::vector<double> transceiverHydrophonePositions =
      this->get_parameter(
              "usbl_simulator."
              "model_parameter_settings."
              "transceiver_hydrophone_positions")
          .as_double_array();

  // Assign double array values to Eigen::Vector3d
  usblSimParams.usblAhrsNoiseStdDev << ahrsNoiseStdDev[0] * M_PI / 180.0,
      ahrsNoiseStdDev[1] * M_PI / 180.0, ahrsNoiseStdDev[2] * M_PI / 180.0;

  // Assign double array values to Eigen::Vector3d
  usblSimParams.p_bu_b << leverArmBodyToUsblCentroidFrame[0],
      leverArmBodyToUsblCentroidFrame[1], leverArmBodyToUsblCentroidFrame[2];

  // Assign double array values to Eigen::Vector3d
  Eigen::Vector3d sensorRotationEulerRpy;
  sensorRotationEulerRpy << rotationBodyToUsblCentroidFrame[0] * M_PI / 180.0,
      rotationBodyToUsblCentroidFrame[1] * M_PI / 180.0,
      rotationBodyToUsblCentroidFrame[2] * M_PI / 180.0;

  // Convert Euler angles to rotation matrix
  usblSimParams.C_u_b =
      Eigen::AngleAxisd(sensorRotationEulerRpy[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(sensorRotationEulerRpy[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(sensorRotationEulerRpy[2], Eigen::Vector3d::UnitX());

  // Assign TC modem transducer position and TC hydrophone positions
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 3; ++j) {
      usblSimParams.p_usH_u(i, j) = transceiverHydrophonePositions[i * 3 + j];
      usblSimParams.p_usTC_u(j) = transceiverModemTransducerPosition[j];
    }
  }

  // Set USBL simulator parameters
  pUsblSimulator_->setUsblSimParams(usblSimParams);
}

/**
 * @brief Declare and retrieve fix loss settings from the YAML file.
*/
void UsblSimulatorNode::declareAndRetrievFixLossSettings() {
  // USBL fix loss model simulation parameters
  UsblFixLossModelSimParams usblFixLossModelSimParams;

  // Declare USBL fix loss model simulation parameters
  this->declare_parameter<double>(
      "usbl_simulator.model_path_fix_loss_settings.a");
  this->declare_parameter<double>(
      "usbl_simulator.model_path_fix_loss_settings.b");
  this->declare_parameter<double>(
      "usbl_simulator.model_path_fix_loss_settings.c");
  this->declare_parameter<double>(
      "usbl_simulator.model_path_fix_loss_settings.d");

  // Retrieve USBL fix loss model simulation parameters
  usblFixLossModelSimParams.a =
      this->get_parameter("usbl_simulator.model_path_fix_loss_settings.a")
          .as_double();
  usblFixLossModelSimParams.b =
      this->get_parameter("usbl_simulator.model_path_fix_loss_settings.b")
          .as_double();
  usblFixLossModelSimParams.c =
      this->get_parameter("usbl_simulator.model_path_fix_loss_settings.c")
          .as_double();
  usblFixLossModelSimParams.d =
      this->get_parameter("usbl_simulator.model_path_fix_loss_settings.d")
          .as_double();

  // Set USBL fix loss model simulation parameters
  pUsblSimulator_->setUsblFixLossModelSimParams(usblFixLossModelSimParams);
}

/**
 * @brief Declare and retrieve enable settings from the YAML file.
*/
void UsblSimulatorNode::declareAndRetrievEnableSettings() {
  // USBL model enable settings
  UsblModelEnableSettings usblModelEnableSettings;

  // Declare USBL model enable settings
  this->declare_parameter<bool>(
      "usbl_simulator."
      "model_enable_settings."
      "enable_rtt_noise_model");
  this->declare_parameter<bool>(
      "usbl_simulator."
      "model_enable_settings."
      "enable_tdoa_noise_model");
  this->declare_parameter<bool>(
      "usbl_simulator."
      "model_enable_settings."
      "enable_ahrs_noise_model");
  this->declare_parameter<bool>(
      "usbl_simulator."
      "model_enable_settings."
      "enable_rtt_quantization_model");
  this->declare_parameter<bool>(
      "usbl_simulator."
      "model_enable_settings."
      "enable_tdoa_quantization_model");
  this->declare_parameter<bool>(
      "usbl_simulator."
      "model_enable_settings."
      "enable_acoustic_path_fix_delay_model");
  this->declare_parameter<bool>(
      "usbl_simulator."
      "model_enable_settings."
      "enable_acoustic_path_fix_loss_model");
  this->declare_parameter<bool>(
      "usbl_simulator."
      "model_enable_settings."
      "enable_acoustic_path_fix_outlier_model");

  // Retrieve USBL model enable settings
  usblModelEnableSettings.enable_rtt_quantization_model =
      this->get_parameter(
              "usbl_simulator."
              "model_enable_settings."
              "enable_rtt_noise_model")
          .as_bool();
  usblModelEnableSettings.enable_tdoa_noise_model =
      this->get_parameter(
              "usbl_simulator."
              "model_enable_settings."
              "enable_tdoa_noise_model")
          .as_bool();
  usblModelEnableSettings.enable_ahrs_noise_model =
      this->get_parameter(
              "usbl_simulator."
              "model_enable_settings."
              "enable_ahrs_noise_model")
          .as_bool();
  usblModelEnableSettings.enable_rtt_quantization_model =
      this->get_parameter(
              "usbl_simulator."
              "model_enable_settings."
              "enable_rtt_quantization_model")
          .as_bool();
  usblModelEnableSettings.enable_tdoa_quantization_model =
      this->get_parameter(
              "usbl_simulator."
              "model_enable_settings."
              "enable_tdoa_quantization_model")
          .as_bool();
  usblModelEnableSettings.enable_acoustic_path_delay =
      this->get_parameter(
              "usbl_simulator."
              "model_enable_settings."
              "enable_acoustic_path_fix_delay_model")
          .as_bool();
  usblModelEnableSettings.enable_fix_loss_rate_model =
      this->get_parameter(
              "usbl_simulator."
              "model_enable_settings."
              "enable_acoustic_path_fix_loss_model")
          .as_bool();
  usblModelEnableSettings.enable_fix_outlier_model =
      this->get_parameter(
              "usbl_simulator."
              "model_enable_settings."
              "enable_acoustic_path_fix_outlier_model")
          .as_bool();

  // Set USBL model enable settings
  pUsblSimulator_->setUsblModelEnableSettings(usblModelEnableSettings);
}

/**
 * @brief Main callback function for the USBL simulator node.
*/
void UsblSimulatorNode::UsblSimulatorLoopCallback() {
  // Get current timestamp
  rclcpp::Time currentTimestamp = now();

  // Create ROS diagnostic message and array message
  diagnostic_msgs::msg::DiagnosticStatus diagnosticMsg;
  diagnostic_msgs::msg::DiagnosticArray diagnosticArrayMsg;

#ifdef USE_NANOAUV_SENSOR_DRIVER_INTERFACES
  // ***************  Create custom USBLLONG message  ************* //
  nanoauv_sensor_driver_interfaces::msg::UsblLong usblLongMsg;

  usblLongMsg.header.stamp = currentTimestamp;
  usblLongMsg.header.frame_id = "world_ned";

  usblLongMsg.current_time = currentTimestamp;
  usblLongMsg.measurement_time = currentTimestamp;

  usblLongMsg.remote_address.data = 0;

  usblLongMsg.position_xyz.x = 0.0;
  usblLongMsg.position_xyz.y = 0.0;
  usblLongMsg.position_xyz.z = 0.0;

  usblLongMsg.position_ned.x = 0.0;
  usblLongMsg.position_ned.y = 0.0;
  usblLongMsg.position_ned.z = 0.0;

  usblLongMsg.attitude_rpy_xyz_to_ned.x = 0.0;
  usblLongMsg.attitude_rpy_xyz_to_ned.y = 0.0;
  usblLongMsg.attitude_rpy_xyz_to_ned.z = 0.0;

  usblLongMsg.propagation_time.data = 0.0;
  usblLongMsg.received_signal_strength_indicator.data = 0.0;
  usblLongMsg.signal_integrity_level.data = 0.0;

  usblLongMsg.accuracy_position_fix.data = 0.0;

  usblLongMsg.is_valid.data = false;
  // ****************************************************************** //

  // **************  Create custom USBLANGLES message  ************ //
  nanoauv_sensor_driver_interfaces::msg::UsblAngles usblAnglesMsg;

  usblAnglesMsg.header.stamp = currentTimestamp;
  usblAnglesMsg.header.frame_id = "world_ned";

  usblAnglesMsg.current_time = currentTimestamp;
  usblAnglesMsg.measurement_time = currentTimestamp;

  usblAnglesMsg.remote_address.data = 0;

  usblAnglesMsg.bearing_local.data = 0.0;
  usblAnglesMsg.elevation_local.data = 0.0;

  usblAnglesMsg.bearing_ned.data = 0.0;
  usblAnglesMsg.elevation_ned.data = 0.0;

  usblAnglesMsg.attitude_rpy_xyz_to_ned.x = 0.0;
  usblAnglesMsg.attitude_rpy_xyz_to_ned.y = 0.0;
  usblAnglesMsg.attitude_rpy_xyz_to_ned.z = 0.0;

  usblAnglesMsg.received_signal_strength_indicator.data = 0.0;
  usblAnglesMsg.signal_integrity_level.data = 0.0;

  usblAnglesMsg.accuracy_position_fix.data = 0.0;

  usblAnglesMsg.is_valid.data = false;
  // ****************************************************************** //
#endif

  // Read out odometry message
  Eigen::Vector3d p_nb_n_true;
  Eigen::Quaterniond q_b_n_true;

  if (groundTruthOdomMsg_ == nullptr) {
    // Print STALE diagnostic message when no ground truth odometry message
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    diagnosticMsg.name = "USBL Simulator";
    diagnosticMsg.message = "Waiting for first ground truth odometry message!";

    // Add diagnostic message to diagnostic array message
    diagnosticArrayMsg.status.push_back(diagnosticMsg);
    diagnosticArrayMsg.header.stamp = currentTimestamp;

#ifdef USE_NANOAUV_SENSOR_DRIVER_INTERFACES
    // Add diagnostic array message to custom USBLLONG message
    usblLongMsg.diagnostic_array = diagnosticArrayMsg;

    // Add diagnostic array message to custom USBLANGLES message
    usblAnglesMsg.diagnostic_array = diagnosticArrayMsg;

    // Publish the custom USBLLONG message
    pUsblLongPublisher_->publish(usblLongMsg);

    // Publish the custom USBLANGLES message
    pUsblAnglesPublisher_->publish(usblAnglesMsg);
#else
    // Publish the diagnostic array message
    pDiagnosticPublisher_->publish(diagnosticArrayMsg);
#endif

    // Reset odometry timeout timer since waiting for first message
    pOdometryTimeOutTimer_->cancel();
    pOdometryTimeOutTimer_->reset();

    return;
  } else {
    // Assign ground truth odometry message to DVL simulator inputs
    p_nb_n_true.x() = groundTruthOdomMsg_.get()->pose.pose.position.x;
    p_nb_n_true.y() = groundTruthOdomMsg_.get()->pose.pose.position.y;
    p_nb_n_true.z() = groundTruthOdomMsg_.get()->pose.pose.position.z;

    // Generate USBL measurement
    UsblMeasurement usblMeasurement = pUsblSimulator_->generateUsblMeasurement(
        Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), p_nb_n_true);

    // Check if generated measurement is valid
    if (usblMeasurement.is_valid) {
      diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      diagnosticMsg.name = "USBL Simulator";
      diagnosticMsg.message = "USBL simulator running nominal!";

      if (is_measurement_invalid_ == true) {
        RCLCPP_INFO(this->get_logger(),
                    "USBL simulator running nominal! Simulator started "
                    "publishing again after valid measurement!");
      }

      // Reset measurement invalid flag
      is_measurement_invalid_ = false;

    } else {
      diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diagnosticMsg.name = "USBL Simulator";
      diagnosticMsg.message =
          "Measurement invalid! Simulator stopped publishing until next valid "
          "measurement!";

      if (is_measurement_invalid_ == false) {
        RCLCPP_ERROR(this->get_logger(),
                     "Measurement invalid! Simulator stopped publishing until "
                     "next valid measurement!");
      }

      // Reset measurement invalid flag
      is_measurement_invalid_ = true;

      // Add diagnostic message to diagnostic array message
      diagnosticArrayMsg.status.push_back(diagnosticMsg);
      diagnosticArrayMsg.header.stamp = currentTimestamp;

#ifdef USE_NANOAUV_SENSOR_DRIVER_INTERFACES
      // Add diagnostic array message to custom USBLLONG message
      usblLongMsg.diagnostic_array = diagnosticArrayMsg;

      // Add diagnostic array message to custom USBLANGLES message
      usblAnglesMsg.diagnostic_array = diagnosticArrayMsg;

      // Publish the custom USBLLONG message
      pUsblLongPublisher_->publish(usblLongMsg);

      // Publish the custom USBLANGLES message
      pUsblAnglesPublisher_->publish(usblAnglesMsg);
#else
      // Publish the diagnostic array message
      pDiagnosticPublisher_->publish(diagnosticArrayMsg);
#endif
      return;
    }

    // Calculate acoustic path delay to delay the publishers
    std::chrono::duration<double> acousticPathDelay(
        usblMeasurement.round_trip_time);

    // If acoustic path delay is enabled, delay the publishers
    if (pUsblSimulator_->getUsblModelEnableSettings()
            .enable_acoustic_path_delay) {
      // Delay the publishers by letting the thread sleep
      std::this_thread::sleep_for(acousticPathDelay);

      // Set current timestamp to time after delay when signal is received
      rclcpp::Time currentTimestamp = now();
    }

    // Fill position fix cartesian NED frame message
    geometry_msgs::msg::PointStamped posFixCartesianNedFrameMsg;
    posFixCartesianNedFrameMsg.header.stamp = currentTimestamp;
    posFixCartesianNedFrameMsg.header.frame_id = "world_ned";
    posFixCartesianNedFrameMsg.point.x = usblMeasurement.p_nu_n[0];
    posFixCartesianNedFrameMsg.point.y = usblMeasurement.p_nu_n[1];
    posFixCartesianNedFrameMsg.point.z = usblMeasurement.p_nu_n[2];

    pPosFixCartesianNedFramePublisher_->publish(posFixCartesianNedFrameMsg);

#ifdef USE_NANOAUV_SENSOR_DRIVER_INTERFACES
    // ******************  Fill custom USBLLONG message  **************** //
    // Position fix cartesian USBL frame message
    usblLongMsg.position_xyz.x = usblMeasurement.p_nu_u[0];
    usblLongMsg.position_xyz.y = usblMeasurement.p_nu_u[1];
    usblLongMsg.position_xyz.z = usblMeasurement.p_nu_u[2];

    // Position fix cartesian NED frame message
    usblLongMsg.position_ned.x = usblMeasurement.p_nu_n[0];
    usblLongMsg.position_ned.y = usblMeasurement.p_nu_n[1];
    usblLongMsg.position_ned.z = usblMeasurement.p_nu_n[2];

    // Euler angles (roll, pitch, yaw) of the internal AHRS
    usblLongMsg.attitude_rpy_xyz_to_ned.x =
        usblMeasurement.ahrsEulRpyAngles.x();
    usblLongMsg.attitude_rpy_xyz_to_ned.y =
        usblMeasurement.ahrsEulRpyAngles.y();
    usblLongMsg.attitude_rpy_xyz_to_ned.z =
        usblMeasurement.ahrsEulRpyAngles.z();

    // Propagation time of the acoustic signal
    usblLongMsg.propagation_time.data =
        usblMeasurement.round_trip_time * 1e6;  // in microseconds

    // RSSI and signal integrity level
    usblLongMsg.received_signal_strength_indicator.data =
        -56.0;                                      // Acceptable dummy value
    usblLongMsg.signal_integrity_level.data = 145;  // Acceptable dummy value

    // Accuracy of the position fix
    usblLongMsg.accuracy_position_fix.data = usblMeasurement.accuracy;

    // Measurement valid flag
    // usblLongMsg.is_valid.data = usblMeasurement.is_valid;
    usblLongMsg.is_valid.data = true;
    // ****************************************************************** //

    // *****************  Fill custom USBLANGLES message  *************** //
    usblAnglesMsg.bearing_local.data = usblMeasurement.p_nu_u_spherical[2];
    usblAnglesMsg.elevation_local.data = usblMeasurement.p_nu_u_spherical[1];

    usblAnglesMsg.bearing_ned.data = usblMeasurement.p_nu_n_spherical[2];
    usblAnglesMsg.elevation_ned.data = usblMeasurement.p_nu_n_spherical[1];

    // Euler angles (roll, pitch, yaw) of the internal AHRS
    usblAnglesMsg.attitude_rpy_xyz_to_ned = usblLongMsg.attitude_rpy_xyz_to_ned;

    // RSSI and signal integrity level
    usblAnglesMsg.received_signal_strength_indicator.data =
        usblLongMsg.received_signal_strength_indicator.data;
    usblAnglesMsg.signal_integrity_level.data =
        usblLongMsg.signal_integrity_level.data;

    // Accuracy of the position fix
    usblAnglesMsg.accuracy_position_fix.data =
        usblMeasurement.accuracy_spherical;

    // Measurement valid flag
    usblAnglesMsg.is_valid.data = usblLongMsg.is_valid.data;
// ****************************************************************** //
#else
    // **************  Publish standard ROS 2 messages  ************ //

    // *********  Publish Round Trip Time message  ******* //
    std_msgs::msg::Float64 roundTripTimeMsg;

    roundTripTimeMsg.data =
        usblMeasurement.round_trip_time * 1e6;  // in microseconds

    pRoundTripTimePublisher_->publish(roundTripTimeMsg);
    // *************************************************** //

    // ***  Publish time differences of arrival message ** //
    std_msgs::msg::Float64MultiArray timeDifferencesOfArrivalMsg;

    for (int i = 0; i < 10; i++) {
      timeDifferencesOfArrivalMsg.data.push_back(
          usblMeasurement.timeDifferencesOfArrival(i));
    }

    pTimeDifferencesOfArrivalPublisher_->publish(timeDifferencesOfArrivalMsg);
    // *************************************************** //

    // **************  Publish AHRS message  ************* //
    geometry_msgs::msg::PointStamped ahrsMsg;

    ahrsMsg.header.stamp = currentTimestamp;
    ahrsMsg.header.frame_id = "usbl_transceiver_link";
    ahrsMsg.point.x = usblMeasurement.ahrsEulRpyAngles.x();
    ahrsMsg.point.y = usblMeasurement.ahrsEulRpyAngles.y();
    ahrsMsg.point.z = usblMeasurement.ahrsEulRpyAngles.z();

    pAhrsPublisher_->publish(ahrsMsg);
    // *************************************************** //

    // ********  Publish direction vector message  ******* //
    geometry_msgs::msg::PointStamped directionVectorMsg;

    directionVectorMsg.header.stamp = currentTimestamp;
    directionVectorMsg.header.frame_id = "usbl_transponder_link";
    directionVectorMsg.point.x = usblMeasurement.directionVector.x();
    directionVectorMsg.point.y = usblMeasurement.directionVector.y();
    directionVectorMsg.point.z = usblMeasurement.directionVector.z();

    pDirectionVectorPublisher_->publish(directionVectorMsg);
    // *************************************************** //

    // ****  Publish Position Cartesian USBL message  **** //
    geometry_msgs::msg::PointStamped posFixCartesianUsblFrameMsg;

    posFixCartesianUsblFrameMsg.header.stamp = currentTimestamp;
    posFixCartesianUsblFrameMsg.header.frame_id = "usbl_transponder_link";
    posFixCartesianUsblFrameMsg.point.x = usblMeasurement.p_nu_u[0];
    posFixCartesianUsblFrameMsg.point.y = usblMeasurement.p_nu_u[1];
    posFixCartesianUsblFrameMsg.point.z = usblMeasurement.p_nu_u[2];

    pPosFixCartesianUsblFramePublisher_->publish(posFixCartesianUsblFrameMsg);
    // *************************************************** //

    // ****  Publish Position Spherical USBL message  **** //
    geometry_msgs::msg::PointStamped posFixSphericalUsblFrameMsg;

    posFixSphericalUsblFrameMsg.header.stamp = currentTimestamp;
    posFixSphericalUsblFrameMsg.header.frame_id = "usbl_transponder_link";
    posFixSphericalUsblFrameMsg.point.x = usblMeasurement.p_nu_u_spherical[0];
    posFixSphericalUsblFrameMsg.point.y = usblMeasurement.p_nu_u_spherical[1];
    posFixSphericalUsblFrameMsg.point.z = usblMeasurement.p_nu_u_spherical[2];

    pPosFixSphericalUsblFramePublisher_->publish(posFixSphericalUsblFrameMsg);
    // *************************************************** //

    // ****  Publish Position Spherical NED message  ***** //
    geometry_msgs::msg::PointStamped posFixSphericalNedFrameMsg;

    posFixSphericalNedFrameMsg.header.stamp = currentTimestamp;
    posFixSphericalNedFrameMsg.header.frame_id = "world_ned";
    posFixSphericalNedFrameMsg.point.x = usblMeasurement.p_nu_n_spherical[0];
    posFixSphericalNedFrameMsg.point.y = usblMeasurement.p_nu_n_spherical[1];
    posFixSphericalNedFrameMsg.point.z = usblMeasurement.p_nu_n_spherical[2];

    pPosFixSphericalNedFramePublisher_->publish(posFixSphericalNedFrameMsg);
    // *************************************************** //

    // ************  Publish accuracy message  *********** //
    std_msgs::msg::Float64 accuracyMsg;

    accuracyMsg.data = usblMeasurement.accuracy;

    pAccuracyPublisher_->publish(accuracyMsg);
    // *************************************************** //

    // ************************************************************ //
#endif
  }

  // Calculate time since last odometry message
  rclcpp::Duration timeSinceLastOdom =
      rclcpp::Duration(currentTimestamp - lastOdomTimestamp_);

  if (timeSinceLastOdom.seconds() > sample_time_ &&
      odometry_timeout_ == false) {
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diagnosticMsg.name = "USBL Simulator";
    diagnosticMsg.message =
        "USBL truth odometry message frequency is too slow!"
        " DVL simulator ground truth frequency higher than odometry!"
        " Increase odometry message frequency!";

    RCLCPP_WARN(this->get_logger(),
                "Ground truth odometry message frequency is too slow!");
  }

  if (odometry_timeout_ == true) {
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    diagnosticMsg.name = "USBL Simulator";
    diagnosticMsg.message =
        "No ground truth odometry message received since than 5 seconds!"
        " USBL simulator stalling!";

    RCLCPP_WARN(this->get_logger(),
                "No ground truth odometry message since more than 5 seconds!");
  }

  // Add diagnostic message to diagnostic array message
  diagnosticArrayMsg.status.push_back(diagnosticMsg);
  diagnosticArrayMsg.header.stamp = currentTimestamp;

#ifdef USE_NANOAUV_SENSOR_DRIVER_INTERFACES
  // Add diagnostic array message to custom USBLLONG message
  usblLongMsg.diagnostic_array = diagnosticArrayMsg;

  // Add diagnostic array message to custom USBLANGLES message
  usblAnglesMsg.diagnostic_array = diagnosticArrayMsg;

  // Publish the custom USBLLONG message
  pUsblLongPublisher_->publish(usblLongMsg);

  // Publsih the custom USBLANGLES message
  pUsblAnglesPublisher_->publish(usblAnglesMsg);
#else
  // Publish the diagnostic array message
  pDiagnosticPublisher_->publish(diagnosticArrayMsg);
#endif
}

/**
 * @brief Odometry callback function.
 * 
 * This function is called when a new odometry message is received. The ground
 * truth odometry message is assigned to the ground truth odometry message
 * member variable of the node class.
 * 
 * @param[in] msg Pointer to the odometry message
*/
void UsblSimulatorNode::odometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Reset odometry timeout timer
  pOdometryTimeOutTimer_->cancel();
  pOdometryTimeOutTimer_->reset();

  // Set first odometry received flag
  if (first_odometry_received_ == false) {
    first_odometry_received_ = true;

    RCLCPP_INFO(this->get_logger(),
                "First ground truth odometry message received! USBL simulator "
                "now running nominal!");
  }
  // Reset odometry timeout flag
  if (odometry_timeout_ == true) {
    odometry_timeout_ = false;

    RCLCPP_INFO(this->get_logger(),
                "Ground truth odometry message received after timeout! USBL "
                "simulator now running nominal!");
  }

  // Assign ground truth odometry message
  groundTruthOdomMsg_ = msg;

  // Assign last odometry timestamp
  lastOdomTimestamp_ = msg->header.stamp;
}

/**
 * @brief Odometry timeout callback function.
 * 
 * This function is called when no ground truth odometry message is received.
 * 
*/
void UsblSimulatorNode::odometryTimeOutCallback() {
  // Set odometry timeout flag
  odometry_timeout_ = true;

  RCLCPP_WARN(this->get_logger(),
              "No ground truth odometry message since more than 5 "
              "seconds! USBL simulator now starting to stale!");
}

/**
 * @brief Publish static tf2 transformations.
*/
void UsblSimulatorNode::publishStaticTf2Transforms() {
  // Get current timestamp
  rclcpp::Time currentTimestamp = now();

  // Get USBL simulator parameters
  UsblSimParams usblSimParams = pUsblSimulator_->getUsblSimParams();

  // Convert rotation matrix to quaternion (USBL centroid to body frame)
  Eigen::Quaterniond q_u_b = Eigen::Quaterniond(usblSimParams.C_u_b);

  // Fill tf2 transform message between base_link and usbl_transponder_link
  geometry_msgs::msg::TransformStamped tfMsg;
  tfMsg.header.stamp = currentTimestamp;

  tfMsg.header.frame_id = "base_link_sname";
  tfMsg.child_frame_id = "usbl_transponder_link";

  tfMsg.transform.translation.x = usblSimParams.p_bu_b[0];
  tfMsg.transform.translation.y = usblSimParams.p_bu_b[1];
  tfMsg.transform.translation.z = usblSimParams.p_bu_b[2];

  tfMsg.transform.rotation.w = q_u_b.w();
  tfMsg.transform.rotation.x = q_u_b.y();
  tfMsg.transform.rotation.y = q_u_b.x();
  tfMsg.transform.rotation.z = q_u_b.z();

  pStaticTf2Broadcaster_->sendTransform(tfMsg);

  // Fill tf2 transform message between world and usbl_transceiver_link
  tfMsg.header.stamp = currentTimestamp;

  tfMsg.header.frame_id = "world_ned";
  tfMsg.child_frame_id = "usbl_transceiver_link";

  tfMsg.transform.translation.x = 0.0;
  tfMsg.transform.translation.y = 0.0;
  tfMsg.transform.translation.z = 0.0;

  tfMsg.transform.rotation.w = 1.0;
  tfMsg.transform.rotation.x = 0.0;
  tfMsg.transform.rotation.y = 0.0;
  tfMsg.transform.rotation.z = 0.0;

  pStaticTf2Broadcaster_->sendTransform(tfMsg);
}

}  // namespace usbl_simulator

/**
 * @brief Main function of the USBL simulator node.
 *
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 *
 * @return int Return value
 */
int main(int argc, char** argv) {
  // Create USBL simulator object
  std::shared_ptr<usbl_simulator::UsblSimulator> pUsblSimulator =
      std::make_shared<usbl_simulator::UsblSimulator>();

  rclcpp::init(argc, argv);

  // Create USBL simulator node
  auto node =
      std::make_shared<usbl_simulator::UsblSimulatorNode>(pUsblSimulator);

  // Create multi-threaded executor
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  executor->add_node(node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}
