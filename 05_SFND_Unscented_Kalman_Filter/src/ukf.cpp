#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;
  n_sig_points_ = 2*n_aug_+1;
  weights_ = VectorXd(n_sig_points_);

  
  // set it to identity matrix for P_
  P_.setIdentity();
    
  // fill the weights vector
  // Corrected weights calculation
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; ++i) {
    weights_(i) = 0.5 / (n_aug_ + lambda_);
  }


  Xsig_pred_ = MatrixXd(n_x_, n_sig_points_);
  // fill with ones not zeros for Xsig_pred_
  // Xsig_pred_.setOnes();
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

    // Check if the filter is not initialized
    if (!is_initialized_) {
        // Initialize the state x_ with the first measurement
        if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            // For laser, position coordinates are directly used
            x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
        } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            // For radar, convert polar to cartesian coordinates
            double rho = meas_package.raw_measurements_[0];
            double phi = meas_package.raw_measurements_[1];
            x_ << rho * cos(phi), rho * sin(phi), 0, 0, 0;
        }

        // Initialize state covariance matrix P_
        P_.setIdentity();
        P_(2, 2) = 80.;  // High uncertainty in initial velocity
        P_(3, 3) = 15.;   // High uncertainty in yaw angle

        // Record the initial time stamp
        time_us_ = meas_package.timestamp_;

        // Mark the filter as initialized
        is_initialized_ = true;
        return;
    }

    // Compute the time elapsed between the current and previous measurements
    double delta_t = (meas_package.timestamp_ - time_us_) / 1e6;  // expressed in seconds
    time_us_ = meas_package.timestamp_;

    // Predict the state and the state covariance matrix
    Prediction(delta_t);

    // Update the state and covariance matrices based on sensor type
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        // Update the state and covariance matrices using Lidar data
        UpdateLidar(meas_package);
    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        // Update the state and covariance matrices using Radar data
        UpdateRadar(meas_package);
    }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  // Augment the state vector and covariance matrix to include process noise
  VectorXd x_aug = VectorXd(7);
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  MatrixXd P_aug = MatrixXd(7, 7);

    // Fill the augmented state vector and covariance matrix
  x_aug.head(n_x_) = x_;
  x_aug.tail(2) << 0, 0;

    // Construct the augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

  // Create augmented sigma points Reference: https://learn.udacity.com/nanodegrees/nd313/parts/cd0684/lessons/afd3219a-ed8c-46e4-8bc7-670dfb986bbe/concepts/5da4c2e6-d8c6-40ff-8629-02846f1453ea
// https://learn.udacity.com/nanodegrees/nd313/parts/cd0684/lessons/afd3219a-ed8c-46e4-8bc7-670dfb986bbe/concepts/ccde2dfe-abfe-4afa-9059-0dffda9efe6d
  MatrixXd L = P_aug.llt().matrixL();
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; ++i) {
    // Calculate and add/subtract the spread of sigma points
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  // Predict sigma points after delta_t time
  // reference: https://learn.udacity.com/nanodegrees/nd313/parts/cd0684/lessons/afd3219a-ed8c-46e4-8bc7-670dfb986bbe/concepts/b3e1bb9b-ab46-4215-bee6-c8db2f009917
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    double p_x = Xsig_aug(0, i), p_y = Xsig_aug(1, i);
    double v = Xsig_aug(2, i), yaw = Xsig_aug(3, i), yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i), nu_yawdd = Xsig_aug(6, i);

    // Predicted state values
    double px_p, py_p, v_p = v, yaw_p = yaw + yawd * delta_t, yawd_p = yawd;
    // Handle division by zero in case yawd is close to zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    } else {
      px_p = p_x + v * delta_t * cos(yaw);
      py_p = p_y + v * delta_t * sin(yaw);
    }

    // Add process noise to the predicted state
    px_p += 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p += 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p += nu_a * delta_t;
    yaw_p += 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p += nu_yawdd * delta_t;

    // Write the predicted sigma points into the matrix
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }
  // Predict state mean and covariance
  // Reset state and covariance to zero before calculating mean and covariance  
  
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; ++i)
  {
    weights_(i) = 1 / (2 * lambda_ + 2 * n_aug_);
  } // set weights
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    x_ += weights_(i) * Xsig_pred_.col(i);
  } // predict state mean
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    AngleNormalization(x_diff, 3);
    P_ += weights_(i) * x_diff * x_diff.transpose();
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  // Extract the raw lidar measurements
  VectorXd z = meas_package.raw_measurements_;
  // Set measurement dimension for lidar (px, py)
  int n_z = 2;
  // Sigma points in measurement space, directly extracting the relevant rows from predicted sigma points
  
  MatrixXd Zsig = Xsig_pred_.block(0, 0, n_z, 2 * n_aug_ + 1);  // Direct assignment
  // Initialize variables for predicted measurement mean and innovation covariance matrix
  VectorXd z_pred = VectorXd::Zero(n_z);
  MatrixXd S = MatrixXd::Zero(n_z, n_z);
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

  // Calculate the mean predicted measurement by weighting the sigma points
  z_pred = Zsig * weights_;

  // Calculate the covariance matrix S and the cross-correlation matrix Tc
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // Uncomment the following line if angle normalization is required
    // AngleNormalization(x_diff, 3);

    S += weights_(i) * z_diff * z_diff.transpose();
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // Add lidar measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_laspx_ * std_laspx_, 0,
       0, std_laspy_ * std_laspy_;
  S += R; // Incorporate measurement noise into the innovation covariance matrix

  // Calculate Kalman gain K, update state mean and covariance matrix
  MatrixXd K = Tc * S.inverse();
  VectorXd z_diff = z - z_pred; // Residual for the actual measurement
  x_ += K * z_diff; // Update the state mean
  P_ -= K * S * K.transpose(); // Update the state covariance matrix
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  // ref: https://learn.udacity.com/nanodegrees/nd313/parts/cd0684/lessons/afd3219a-ed8c-46e4-8bc7-670dfb986bbe/concepts/118b6bde-427a-4365-bd15-8efa3db67908
  // Extract radar measurement as a vector (rho, phi, rho_dot)
  VectorXd z = meas_package.raw_measurements_;
  // Set radar measurement dimension (r, phi, and r_dot)
  int n_z = 3;
  
  // Create matrix for sigma points in radar measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  VectorXd z_pred = VectorXd::Zero(n_z);
  MatrixXd S = MatrixXd::Zero(n_z, n_z);
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

  // Transform sigma points into radar measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);
    
    // Calculate velocity components in x and y direction
    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    // Transform sigma points into radar measurement space
    Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);
    Zsig(1, i) = atan2(p_y, p_x);
    Zsig(2, i) = (p_x * v1 + p_y * v2) / Zsig(0, i);
  }

  // Calculate mean predicted radar measurement
  z_pred = Zsig * weights_;

  // Calculate covariance matrix S and cross correlation matrix Tc
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // Residual for measurement space
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // Angle normalization for phi
    AngleNormalization(z_diff, 1);
    S += weights_(i) * z_diff * z_diff.transpose();

    // Residual for state space
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    AngleNormalization(x_diff, 3);
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // Add radar measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_ * std_radr_, 0, 0,
       0, std_radphi_ * std_radphi_, 0,
       0, 0, std_radrd_ * std_radrd_;
  S += R;

  MatrixXd K = Tc * S.inverse();   // Calculate Kalman gain K
  VectorXd z_diff = z - z_pred; // Update state mean and covariance matrix
  AngleNormalization(z_diff, 1); // Angle normalization for phi
  x_ += K * z_diff; // Update state mean
  P_ -= K * S * K.transpose(); // Update state covariance matrix
}

void UKF::AngleNormalization(Eigen::VectorXd &vector, int index) {
  // Reference: https://learn.udacity.com/nanodegrees/nd313/parts/cd0684/lessons/afd3219a-ed8c-46e4-8bc7-670dfb986bbe/concepts/5acad793-4de4-4ef8-aadb-b7eeb070de15
  while (vector(index)> M_PI) vector(index)-=2.*M_PI;
  while (vector(index)<-M_PI) vector(index)+=2.*M_PI;
}