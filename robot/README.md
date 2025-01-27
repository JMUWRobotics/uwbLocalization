# Robot

This folder contains all code running on the PC, that is attached to the volksbot robot.

The [robot.launch](launch/robot.launch) file launches all code to run the robot and has multiple arguments:
- `useRiegl`: a boolean, that sets if the nodes responsible for the Riegl Scanner are launched. (default:`false`)
- `useHectorMapping`: a boolean, that sets if the Hector SLAM is started. (default:`false`) There are further arguments for the Hector SLAM algorithm, which all have sensible default values. For more info click [here](https://wiki.ros.org/hector_slam).
- `useUwb`: a boolean, that sets if the nodes responsible for the UWB Localization are to be launched. (default:`false`) This includes the `rosserial_server` with further arguments `numberPorts` (default:3), `port0` (default: `/dev/UWB-back`), `port1` (default: `/dev/UWB-front-right`), `port2` (default: `/dev/UWB-front-left`), `baud` (default: `115200`). It also starts the EKF according to [this](src/uwb/launch).

Within the [src](src/) folder there are multiple packages, each responsible for a specific task:
- `lms100`: gather data from the 2D SICK LMS100 laser scanner
- `rclock`, `riegl`, `riegl_scan_retriever`, `rivlib`: interface with the Rigel VZ-400 laser scanner
- `rosserial_server`: serial connection to the UWB boards
- `turtlesim_representation`: python turtle based live visualization of the localization systems
- `uwb`: uwb localization implementation and evaluation
- `volksbot`: base robot code