# Unscented Kalman Filter Project

In this project I implemented an Uscented Kalman Filter for the position and velocity estimation of a car. The motion model used is the Constant Turn Rate and Velocity magnitude model (CTRV).

Measurements for lidar and radar are used to update the car state once a predicition is made using the CTRV model. 
The standard deviations for the longitudinal acceleration and yaw rate noises were chosen to be 2m/s2 and pi/4 rad/s2, respectively.

In this project, I:

* implemented the time update function `Update` in which the motion model predicts the sigma points and the resulting state and covariance matrix
* implemented the `UpdateLidar` and `UpdateRadar` methods in which the measurement models for each instrument were used to project the sigma points in the measurements space and calculate the predicted measurement and the covariance matrix.
* implemented the `UpdateAll` method in which the cross-correlation between the state and the measurements sigma points is used to calculate the Kalman gain, which ultimately is used to calculate the final state mean and covariance update. In this same method, the Normalized Innovation Squared value is also calculated in order to check if the filter over or understimates the uncertainty of the measurements. 
* an important factor in tuning the filter was the initial state covariance matrix.

The RMSE for Dataset1 obtained with these settings is [0.07, 0.08, 0.31, 0.23]'.