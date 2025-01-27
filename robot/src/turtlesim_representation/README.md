# Turtlesim Visualization

This package includes two real time visualization scripts, that use the python Turtle library.
The [anchorPoints](scripts/anchorPoints.py) script plots the position of the determined anchor positions for a given number of anchors. It also plots the mean and an ellipse representing the standard deviation along the x- and y-axis.
The [scenario](scripts/scenario.py) script plots varying localization methods in different colors (blue: only wheel odom, red: only UWB measurements, black: EKF, yellow: Hector-SLAM). It also shows a fixed ground truth for the location of the laser scans and the position of the anchors. This data must be manually written into the variable `truth_pos` in the script.