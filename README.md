# C++/ROS 2 Ultra-Short Baseline (USBL) Simulator

[![License](https://img.shields.io/badge/license-BSD--3-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

![](./data/icon.svg)

<!--- protected region package header begins -->
**Author:**
- Maximilian Nitsch <m.nitsch@irt.rwth-aachen.de>

**Affiliation:** Institute of Automatic Control - RWTH Aachen University

**Maintainer:**
  - Maximilian Nitsch <m.nitsch@irt.rwth-aachen.de>
<!--- protected region package header ends -->

## Description
This project provides a high-fidelity USBL simulator written in C++ and a ROS 2 node that acts as a wrapper for the simulator.

The simulator implements the following **features**:
- USBL measurement simulation in USBL-and NED-frame
- USBL array simulation with transducer and hydrophone positions
- Round-trip-time (RTT) noise and quantization simulation
- Time-Difference-of-Arrival (TDOA) noise and quantization simulation
- Internal Attitude and Heading Reference System (AHRS) simulation (Xsens MTi-100 Inertial Measurement Unit)
- Acoustic path delay simulation
- Acoustic position fix loss simulation (exponential loss model)
- Acoustic position fix outlier simulation (future release)
- All parameters for the USBL can be configured in a YAML file
- All models and effects can be enabled/disabled separately

An example config file for an OEM 120/180 OEM USBL is provided. Parameters for an S2C R 7/17 USBL are found in the references below. 
For other parameters, we refer to EvoLogics.

> **Scope and Limitations of this Simulator:**
> - This simulator does not model the exact acoustic signal propagation and interactions with the environment. This would require a full acoustic simulation of the environment i.e. using ray tracing techniques.
> - Instead, the simulator propagates white noise measurements through the geometry of the USBL to provide realistic measurements under ideal (datasheet) conditions.
> - This simulator is intended for testing and validation of navigation algorithms (sensor fusion) and their real-time performance.
> - This simulator only simulates the localization feature of the USBL. The acoustic communication capability of EvoLogics modems is not simulated. 

## Table of Contents

- [Dependencies](#dependencies)
- [ROS 2 Node](#ros-2-node)
  - [Publishers](#publishers)
  - [Subscribers](#subscribers)
- [Installation](#installation)
- [Usage](#usage)
- [Coding Guidelines](#coding-guidelines)
- [References](#references)
- [Reports](#reports)
- [Contributing](#contributing)
- [License](#license)

# Dependencies

This project depends on the following literature and libraries:

- **Eigen3**: Eigen is a C++ template library for linear algebra: [Eigen website](https://eigen.tuxfamily.org/).
- **ROS 2 Humble**: ROS 2 is a set of software libraries and tools for building robot applications: [ROS 2 Installation page](https://docs.ros.org/en/humble/Installation.html).
- **nanoauv_sensor_driver_interfaces**: TRIPLE-project specific ROS 2 package with nanoAUV interfaces. You don't need to build that package.


## ROS 2 Node Description

The USBL simulator node implements several publishers, depending on the **CMake build macro**, and subscribes to one topic.
ROS 2 services or actions are not provided.

### Publishers

This node publishes the following topics:

| Topic Name       | Message Type        | Description                        | Link     |
|------------------|---------------------|------------------------------------|----------|
| `*/usbllong`   | `nanoauv_sensor_driver_interfaces/UsblLong.msg`   | Custom USBL Cartesian position message holding USBLLONG.| [UsblLong.msg](https://gitlab.informatik.uni-bremen.de/triple/gnc/interfaces/-/blob/b95efc88d33a9e439025056c988c6459589b86e5/nanoauv_sensor_driver_interfaces/msg/UsblLong.msg) |
| `*/usblangles`   | `nanoauv_sensor_driver_interfaces/UsblAngles.msg`   | Custom USBL spherical position message holding USBLANGLES.| [UsblAngles.msg](https://gitlab.informatik.uni-bremen.de/triple/gnc/interfaces/-/blob/main/nanoauv_sensor_driver_interfaces/msg/UsblAngles.msg?ref_type=heads) |
| `*/pos_fix_cartesian_ned_frame`   | `geometry_msgs/PointStamped.msg`   | Position vector in Cartesian coordinates w.r.t. NED frame.| [PointStamped.msg](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PointStamped.html) |

If the nanoAUV interface package is not used, the following publishers are available:
| Topic Name       | Message Type        | Description                        | Link     |
|------------------|---------------------|------------------------------------|----------|
| `*/round_trip_time`   | `std_msgs/Float64.msg`   | Round-trip-time measurement of USBL.| [Float64.msg](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html) |
| `*/time_differences_of_arrival`   | `std_msgs/Float64MultiArray.msg`   | Time-differences-of-arrival measurements of USBL.| [Float64MultiArray.msg](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64MultiArray.html) |
| `*/ahrs`   | `std_msgs/Float64.msg`   | Euler angles from internal Attitude Heading Reference System (AHRS).| [PointStamped.msg](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PointStamped.html) |
| `*/direction_vector`   | `diagnostic_msgs/DiagnosticArray.msg`   | Direction vector pointing from USBL transceiver to transponder.| [PointStamped.msg](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PointStamped.html) |
| `*/pos_fix_cartesian_ned_frame`   | `geometry_msgs/PointStamped.msg`   | Position vector in Cartesian coordinates w.r.t. NED frame.| [PointStamped.msg](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PointStamped.html) |
| `*/pos_fix_cartesian_usbl_frame`   | `geometry_msgs/PointStamped.msg`   | Position vector in Cartesian coordinates w.r.t. USBL frame.| [PointStamped.msg](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PointStamped.html) |
| `*/pos_fix_spherical_ned_frame`   | `geometry_msgs/PointStamped.msg`   | Position vector in spherical coordinates w.r.t. NED frame.| [PointStamped.msg](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PointStamped.html) |
| `*/pos_fix_spherical_usbl_frame`   | `geometry_msgs/PointStamped.msg`   | Position vector in spherical coordinates w.r.t. USBL frame.| [PointStamped.msg](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PointStamped.html) |
| `*/accuracy`   | `std_msgs/Float64.msg`   | Accuracy (standard deviation) of position fix.| [Float64.msg](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html) |
| `*/diagnostic`   | `diagnostic_msgs/DiagnosticArray.msg`   | Array with diagnostic information about the state of ROS 2 node.| [DiagnosticArray.msg](http://docs.ros.org/en/melodic/api/diagnostic_msgs/html/msg/DiagnosticArray.html) |

### Subscribers

This node subscribes to the following topics:

| Topic Name        | Message Type        | Description                        | Link     |
|-------------------|---------------------|------------------------------------|----------|
| `*/odometry`| `nav_msgs/Odometry.msg`| Estimate of a position and velocity in free space.| [Odometry.msg](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) |



# Installation

To install the `usbl_simulator_package`, you need to follow these steps:

1. **Install Eigen3**: Eigen3 is a dependency for your package. You can install it using your package manager. For example, on Ubuntu, you can install it using the following command:

    ```bash
    sudo apt-get install libeigen3-dev
    ```

2. **Install ROS 2 Humble**: Ensure you have ROS 2 (Humble) installed. You can follow the official installation instructions provided by ROS 2. Visit [ROS 2 Humble Installation page](https://docs.ros.org/en/humble/Installation.html) for detailed installation instructions tailored to your platform.

3. **Clone the Package**: Clone the package repository to your ROS 2 workspace. If you don't have a ROS 2 workspace yet, you can create one using the following commands:

    ```bash
    mkdir -p /path/to/ros2_workspace/src
    cd /path/to/ros2_workspace/src
    ```

    Now, clone the package repository:

    ```bash
    git clone <repository_url>
    ```

    Replace `<repository_url>` with the URL of your package repository.

4. **Build the Package**: Once the package is cloned, you must build it using colcon, the default build system for ROS 2. Navigate to your ROS 2 workspace and run the following command:

    ```bash
    cd /path/to/ros2_workspace
    colcon build
    ```

    This command will build all the packages in your workspace, including the newly added package.

5. **Source the Workspace**: After building the package, you need to source your ROS 2 workspace to make the package available in your ROS 2 environment. Run the following command:

    ```bash
    source /path/to/ros2_workspace/install/setup.bash
    ```

    Replace `/path/to/ros2_workspace` with the actual path to your ROS 2 workspace.

That's it! Your `usbl_simulator_package` should now be installed along with its dependencies and ready to use in your ROS 2 environment.

## Usage

1. **Configure your YAML file** for your USBL or use the default file.

2. **Start the USBL simulator** with the launch file:
    ```bash
    ros2 launch usbl_simulator_package usbl_simulator.launch.py
    ```
  The USBL simulator prints your settings and waits for a ground truth odometry message.

3. **Provide an odometry publisher** from you vehicle simulation.

4. **Check ROS 2 topics** the USBL values should now be published.

**Important Usage Information**:
- The odometry message must be published with at least the USBL data rate/sample time.
- The message `*/diagnostic` will show `WARN` if the odometry rate is lower.
- If no odometry message is published, the message `*/diagnostic` will show `STALE`.
- If everything is correct `*/diagnostic` will show `OK`. 

## Coding Guidelines

This project follows these coding guidelines:
- https://google.github.io/styleguide/cppguide.html
- http://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html 

## References

The USBL simulator implementation follows the following publications and datasheets:

- M. Nitsch, "Navigation of a miniaturized autonomous underwater vehicle exploring waters under ice," Dissertation, Rheinisch-Westfälische Technische Hochschule Aachen, Aachen, RWTH Aachen University, 2024. [DOI: 10.18154/RWTH-2024-05964](https://www.researchgate.net/publication/382562855_Navigation_of_a_Miniaturized_Autonomous_Underwater_Vehicle_Exploring_Waters_Under_Ice?_sg%5B0%5D=xNyP6RXVcEfazembhPB6cRxGQTBAvWqw6qMza26FExHUVzWcV9VUd35T4l6KjUqbo1a7W6okgPi3zqqUQYww5dmfZgsQcoJlvBE3ss1T.JLcM4K_iQyfJO7N73P9ebOmEd0xchppKYQemo5hh6ecobLxw5ZSaPgwEvlqYcQtr25iVtPvdMorpxfHK_Oldag&_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6ImhvbWUiLCJwYWdlIjoicHJvZmlsZSIsInBvc2l0aW9uIjoicGFnZUNvbnRlbnQifX0).
- Bannasch, R.; Kebkal, K.; Yakovlev, S.; Kebkal, A. "Fast and Reliable Underwater Communication: Successful Applications of Biologically Inspired Techniques." In: *Volume 1: Offshore Technology; Offshore Wind Energy; Ocean Research Technology; LNG Specialty Symposium*, 25th International Conference on Offshore Mechanics and Arctic Engineering, Hamburg, Germany: ASMEDC, Jan. 1, 2006, pp. 741–747. doi: [10.1115/OMAE2006-92550](https://doi.org/10.1115/OMAE2006-92550).
- Caiti, A.; Di Corato, F.; Fenucci, D.; Allotta, B.; Costanzi, R.; Monni, N.; Pugi, L.; Ridolfi, A. "Experimental Results with a Mixed USBL/LBL System for AUV Navigation." In: *2014 Underwater Communications and Networking (UComms)*, Sept. 2014, pp. 1–4. doi: [10.1109/UComms.2014.7017129](https://doi.org/10.1109/UComms.2014.7017129).
- Ehlers, F. "Autonomous Underwater Vehicles: Design and Practice." The Institution of Engineering and Technology, 2020.
- Evologics GmbH, ed. "S2C Reference Manual, Edition Standard, Firmware Version 2.0." Evologics GmbH, Nov. 2019.
- Evologics GmbH, ed. "S2CR 48/78 USBL Product Information Datasheet." Evologics GmbH, June 2012.
- Evologics GmbH, ed. "USBL Positioning and Communication Systems Product Information Guide." Evologics GmbH, Oct. 2021.
- Evologics GmbH, ed. "S2C T 30/60 Underwater Acoustic Modem Datasheet." Evologics GmbH, Aug. 2022.
- Evologics GmbH, ed. "USBL Positioning and Communication System S2CR 30/60 USBL Product Information Datasheet." Evologics GmbH, July 2022.
- Hildebrandt, M.; Creutz, T.; Wehbe, B.; Wirtz, M.; Zipper, M. "Under-Ice Field Tests with an AUV in Abisko/Torneträsk." In: *OCEANS 2022*, Hampton Roads, Oct. 2022, pp. 1–7. doi: [10.1109/OCEANS47191.2022.9977094](https://doi.org/10.1109/OCEANS47191.2022.9977094).
- Jakuba, M. V.; Roman, C. N.; Singh, H.; Murphy, C.; Kunz, C.; Willis, C.; Sato, T.; Sohn, R. A. "Long-Baseline Acoustic Navigation for Under-Ice Autonomous Underwater Vehicle Operations." In: *Journal of Field Robotics*, vol. 25, no. 11-12 (2008), pp. 861–879. doi: [10.1002/rob.20250](https://doi.org/10.1002/rob.20250).
- Kebkal, K. G.; Kebkal, O. G.; Bannasch, R.; Yakovlev, S. G. "Performance of a Combined USBL Positioning and Communication System Using S2C Technology." In: *2012 Oceans - Yeosu*, May 2012, pp. 1–7. doi: [10.1109/OCEANS-Yeosu.2012.6263376](https://doi.org/10.1109/OCEANS-Yeosu.2012.6263376).
- Kebkal, K. G.; Mashoshin, A. I. "AUV Acoustic Positioning Methods." In: *Gyroscopy and Navigation*, vol. 8, no. 1 (Jan. 1, 2017), pp. 80–89. doi: [10.1134/S2075108717010059](https://doi.org/10.1134/S2075108717010059).
- Knapp, C.; Carter, G. "The Generalized Correlation Method for Estimation of Time Delay." In: *IEEE Transactions on Acoustics, Speech, and Signal Processing*, vol. 24, no. 4 (Aug. 1976), pp. 320–327. doi: [10.1109/TASSP.1976.1162830](https://doi.org/10.1109/TASSP.1976.1162830).
- Morgado, M.; Oliveira, P.; Silvestre, C. "Tightly Coupled Ultrashort Baseline and Inertial Navigation System for Underwater Vehicles: An Experimental Validation." In: *Journal of Field Robotics*, vol. 30, no. 1 (2013), pp. 142–170. doi: [10.1002/rob.21442](https://doi.org/10.1002/rob.21442).
- Morgado, M.; Oliveira, P.; Silvestre, C.; Vasconcelos, J. F. "Embedded Vehicle Dynamics Aiding for USBL/INS Underwater Navigation System." In: *IEEE Transactions on Control Systems Technology*, vol. 22, no. 1 (Jan. 2014), pp. 322–330. doi: [10.1109/TCST.2013.2245133](https://doi.org/10.1109/TCST.2013.2245133).
- Morgado, M.; Oliveira, P.; Silvestre, C.; Vasconcelos, J. "USBL/INS Tightly-Coupled Integration Technique for Underwater Vehicles." In: *2006 9th International Conference on Information Fusion*, July 2006, pp. 1–8. doi: [10.1109/ICIF.2006.301607](https://doi.org/10.1109/ICIF.2006.301607).
- Richmond, K.; Gulati, S.; Flesher, C.; Hogan, B. P.; Stone, W. C. "Navigation, Control, and Recovery of the ENDURANCE Under-Ice Hovering AUV." In: *International Symposium on Unmanned Untethered Submersible Technology (UUST)*, Aug. 25, 2009, p. 13.
- Tan, H.-P.; Diamant, R.; Seah, W. K.; Waldmeyer, M. "A Survey of Techniques and Challenges in Underwater Localization." In: *Ocean Engineering*, vol. 38, no. 14-15 (Oct. 2011), pp. 1663–1676. doi: [10.1016/j.oceaneng.2011.07.017](https://doi.org/10.1016/j.oceaneng.2011.07.017).
- Xsens. "MTi Series: A Complete Line of MEMS Motion Trackers - IMU, VRU, AHRS and GNSS/INS." Xsens Technologies B.V.

## Contributing

If you want to contribute to the project, see the [CONTRIBUTING](CONTRIBUTING) file for details.

## License

This project is licensed under the BSD-3-Clause License. Please take a look at the [LICENSE](LICENSE) file for details.
