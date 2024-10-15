/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#pragma once

#include "usbl_simulator.hpp"

// C++ includes
// NOLINTNEXTLINE(build/c++11)
#include <chrono>
// NOLINTNEXTLINE(build/c++11)
#include <memory>
// NOLINTNEXTLINE(build/c++11)
#include <thread>

// ROS2 core includes
#include <rclcpp/rclcpp.hpp>

// Standard messages
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// tf2 includes
#include <tf2_ros/static_transform_broadcaster.h>

#ifdef USE_NANOAUV_SENSOR_DRIVER_INTERFACES
// Custom messages from nanoauv_sensor_driver_interfaces package
#include "nanoauv_sensor_driver_interfaces/msg/usbl_angles.hpp"
#include "nanoauv_sensor_driver_interfaces/msg/usbl_long.hpp"
#endif

namespace usbl_simulator {

class UsblSimulatorNode : public rclcpp::Node {
 public:
  /* ***************************************************************************
   * Public member functions
   ****************************************************************************/
  // Constructor
  explicit UsblSimulatorNode(std::shared_ptr<UsblSimulator> pUsblSimulator);

  // Destructor
  ~UsblSimulatorNode();

 private:
  /* ***************************************************************************
   * Private member functions
   ****************************************************************************/

  // Declaration and retrieval for parameters from YAML file
  void declareAndRetrieveGeneralSettings();
  void declareAndRetrieveSimParams();
  void declareAndRetrievFixLossSettings();
  void declareAndRetrievEnableSettings();

  // Timer callback function
  void UsblSimulatorLoopCallback();

  // Odometry callback function
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void odometryTimeOutCallback();

  // tf2 static broadcaster callback function
  void publishStaticTf2Transforms();

  /* ***************************************************************************
   * Private member variables
   ****************************************************************************/

  // USBL simulator class object
  std::shared_ptr<UsblSimulator> pUsblSimulator_;

  // Ground truth odometry message
  nav_msgs::msg::Odometry::SharedPtr groundTruthOdomMsg_;

  // Last odometry timestamp
  rclcpp::Time lastOdomTimestamp_;

// USBL message publishers
#ifdef USE_NANOAUV_SENSOR_DRIVER_INTERFACES
  rclcpp::Publisher<nanoauv_sensor_driver_interfaces::msg::UsblLong>::SharedPtr
      pUsblLongPublisher_;
  rclcpp::Publisher<nanoauv_sensor_driver_interfaces::msg::UsblAngles>::
      SharedPtr pUsblAnglesPublisher_;
#else
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pRoundTripTimePublisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      pTimeDifferencesOfArrivalPublisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
      pAhrsPublisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
      pDirectionVectorPublisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
      pPosFixCartesianUsblFramePublisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
      pPosFixSphericalUsblFramePublisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
      pPosFixSphericalNedFramePublisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pAccuracyPublisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      pDiagnosticPublisher_;
#endif

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
      pPosFixCartesianNedFramePublisher_;

  // Vehicle odometry subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pOdometrySubscriber_;

  // Static tf2 broadcaster
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> pStaticTf2Broadcaster_;

  // Timers
  rclcpp::TimerBase::SharedPtr pTimer_;
  rclcpp::TimerBase::SharedPtr pOdometryTimeOutTimer_;

  // Sample time
  double sample_time_;

  // Odometry flags
  bool first_odometry_received_;
  bool odometry_timeout_;

  // Warning/error flags
  bool is_measurement_invalid_;
};

}  // namespace usbl_simulator
