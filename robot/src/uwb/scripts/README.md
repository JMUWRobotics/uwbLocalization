# UWB message processing scripts

The scripts [calc](calc.py) and [makeSimpleMapFromData](makeSimpleMapFromData.py) produce information about the timing of a single UWB message and a map of UWB nodes based on all inter-node distance measurements respectively. They only use one off data directly written into the script.

The scripts [getAvgDistance](getAvgDistance.py) and [timingAnalysis](timingAnalysis.py) offer information about the distance between two nodes and the response time respectively. They run with a reduced system of UWB nodes.

The nodes [evalAnchorPoints](evalAnchorPoints.py) and [evalEKF](evalEKF.py) save data about the determined position of the anchors and the result of the EKF. The results of the EKF are saved in a csv file once the ros-node shuts down (after ros.spin() no longer blocks the script), the results of the evalAnchorPoints are printed to the command line. These nodes can run during the actual localization on the robot, but also for evaluation purposes using a rosbag.

The nodes [anchorPosDeterm](anchorPosDeterm.py) and [ekfSlam](ekfSlam.py) represent the actual localization system. The [anchorPosDeterm](anchorPosDeterm.py) nodes reads out the UWB distance messages from which is determines the anchors positions and publishes these to a ros topic. The [ekfSlam](ekfSlam.py) node uses these anchor positions along with inter-anchor distance measurements and the wheel odometry to run the EKF and produce a pose estimation. Both scripts have some constants, that have to be set manually, among others the number of used anchors.

The [onlyUWBPoseEst](onlyUwbPoseEst.py) node uses the latest valid position and only the UWB measurements to create a position estimation.