#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_.fill(0.0);
  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_.fill(0.0);
  P_(0, 0) = 1;
  P_(1, 1) = 1;
  P_(2, 2) = 25;
  P_(3, 3) = M_PI * M_PI/4;
  P_(4, 4) = M_PI * M_PI/4;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = M_PI/4;

  // DO NOT MODIFY measurement noise values below these are provided by the
  // sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  // DO NOT MODIFY measurement noise values above these are provided by the
  // sensor manufacturer.

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  n_x_ = 5;
  n_aug_ = 7;

  lambda_ = 3 - n_aug_;

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //initializing weights
  weights_ = VectorXd(2 * n_aug_ + 1);
  double weight_0 = lambda_ / (lambda_ + n_aug_);

  weights_(0) = weight_0;
  for (int i = 1; i< 2*n_aug_ +1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug_ + lambda_);
    weights_(i) = weight;
  }

  //initialize measurement noises
  R_radar = MatrixXd(3, 3);
  R_radar <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;

  R_lidar = MatrixXd(2, 2);
  R_lidar <<    std_laspx_*std_laspx_, 0,
          0, std_laspy_*std_laspy_;

  //initialize state error vector
  x_diff = MatrixXd(n_x_, 2*n_aug_+1);
  x_diff.fill(0.0);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_initialized_) {
    // first measurement
    cout << "UKF: " << endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      cout << "radar measurement init :" << endl;
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float tan_phi = tan(phi);
      double rhod = measurement_pack.raw_measurements_[2];

      double pxx = rho * rho / (1 + tan_phi * tan_phi);
      double pyy = rho * rho - pxx;

      double px = sqrt(pxx);
      double py = sqrt(pyy);

      x_ << px, py, 0, 0, 0;
      cout << " init x : " << endl;
      cout << x_ << endl;
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      cout << "lidar measurement init : " << endl;
      x_ << measurement_pack.raw_measurements_[0],
          measurement_pack.raw_measurements_[1], 0, 0, 0;
      cout << " init x : " << endl;
      cout << x_ << endl;
    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  Prediction(dt);

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
     UpdateRadar(measurement_pack);
  }
  else
     UpdateLidar(measurement_pack);

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double dt) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  // time step


  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  MatrixXd x_sig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  x_sig_aug.col(0) = x_aug;

  MatrixXd P_aug = MatrixXd(7, 7);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  MatrixXd Psqrt = P_aug.llt().matrixL();
  
  // generate sigma points
  for (int i = 0; i < n_aug_; i++) {
    x_sig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * Psqrt.col(i);
    x_sig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * Psqrt.col(i);
  }
  
  // predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    double p_x = x_sig_aug(0, i);
    double p_y = x_sig_aug(1, i);
    double v = x_sig_aug(2, i);
    double psi = x_sig_aug(3, i);
    double psidot = x_sig_aug(4, i);
    double nu_a = x_sig_aug(5, i);
    double nu_psiddot = x_sig_aug(6, i);

    double px_p, py_p;

    // integration step
    if (fabs(psidot) > 0.001) {
      px_p = p_x + v / psidot * (sin(psi + psidot * dt) - sin(psi));
      py_p = p_y + v / psidot * (-cos(psi + psidot * dt) + cos(psi));
    } else {
      px_p = p_x + v * cos(psi) * dt;
      py_p = p_y + v * sin(psi) * dt;
    }
  
    double v_p = v;
    double psi_p = psi + psidot*dt;
    double psidot_p = psidot;

    // add noise
    px_p = px_p + 0.5 * nu_a * dt * dt * cos(psi);
    py_p = py_p + 0.5 * nu_a * dt * dt * sin(psi);
    v_p = v_p + nu_a * dt;

    psi_p = psi_p + 0.5 * nu_psiddot * dt * dt;
    psidot_p = psidot_p + nu_psiddot * dt;

    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = psi_p;
    Xsig_pred_(4, i) = psidot_p;
  }
  //predict mean and covariance from the predicted sigma points
  VectorXd x_pred = VectorXd(n_x_);
  MatrixXd P_pred = MatrixXd(n_x_, n_x_);

  x_pred.fill(0.0);
  for (int i = 0; i<2 * n_aug_ + 1; i++) {
    x_pred += weights_(i) * Xsig_pred_.col(i);
  }

  P_pred.fill(0.0);

  x_diff = MatrixXd(n_x_, 2*n_aug_+1);
  for (int i = 0; i<2 * n_aug_ + 1; i++) {
    x_diff.col(i) = Xsig_pred_.col(i) - x_pred;
    while (x_diff(3,i)> M_PI) x_diff(3,i)-=2.*M_PI;
    while (x_diff(3,i)<-M_PI) x_diff(3,i)+=2.*M_PI;

    P_pred += weights_(i) * x_diff.col(i) * x_diff.col(i).transpose();
  }
  x_ = x_pred;
  P_ = P_pred;
}

void UKF::UpdateAll() {
  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  x_ = x_ + K * innov;
  P_ = P_ - K*S*K.transpose();

  //calculate NIS
  //square root of measurement covariance
  MatrixXd Ssqrt = S.llt().matrixL();

  double NIS = innov.transpose() * Ssqrt * innov;
  cout << "NIS: "<<NIS<<endl;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage measurement_pack) {
  /**
  TODO:
  
  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  int n_z = 2;
  
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    
    // measurement model
    Zsig(0,i) = p_x;                        //px
    Zsig(1,i) = p_y;                        //py
  }

  //predicted mean measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  S = MatrixXd(n_z,n_z);
  S.fill(0.0);

  MatrixXd z_diff = MatrixXd(n_z, 2*n_aug_ + 1);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    z_diff.col(i) = Zsig.col(i) - z_pred;
    S = S + weights_(i) * z_diff.col(i) * z_diff.col(i).transpose();
  }

  //add measurement noise covariance matrix
  S = S + R_lidar;
  
  //get the measurements
  VectorXd z = measurement_pack.raw_measurements_;
  //calculate innovation
  innov = VectorXd(n_z);
  innov = z - z_pred;

  //cross - correlation matrix
  Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    Tc = Tc + weights_(i) * x_diff.col(i) * z_diff.col(i).transpose();
  }
  //Kalman gain, final update
  cout << "-- update LIDAR --"<<endl;
  UpdateAll();
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage measurement_pack) {
  /**
  TODO:
  
  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  int n_z = 3;

  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v   = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    double pxx = p_x*p_x;
    double pyy = p_y*p_y;

    // measurement model
    Zsig(0,i) = sqrt(pxx + pyy);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(pxx + pyy);   //r_dot
  }
  //predicted mean measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  

  //innovation covariance matrix S
  S = MatrixXd(n_z,n_z);
  S.fill(0.0);

  MatrixXd z_diff = MatrixXd(n_z, 2*n_aug_ + 1);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    z_diff.col(i) = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1,i)> M_PI) z_diff(1,i)-=2.*M_PI;
    while (z_diff(1,i)<-M_PI) z_diff(1,i)+=2.*M_PI;

    S = S + weights_(i) * z_diff.col(i) * z_diff.col(i).transpose();
  }

  //add measurement noise covariance matrix
  S = S + R_radar;

  VectorXd z = measurement_pack.raw_measurements_;
  //calculate innovation
  innov = VectorXd(n_z);
  innov = z - z_pred;

  //angle normalization
  while (innov(1)> M_PI) innov(1) -= 2.*M_PI;
  while (innov(1)<-M_PI) innov(1) += 2.*M_PI;

  //cross - correlation  
  Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    Tc = Tc + weights_(i) * x_diff.col(i) * z_diff.col(i).transpose();
  }

  //Kalman gain, final update
  cout << "-- update RADAR --"<<endl;
  UpdateAll();
}