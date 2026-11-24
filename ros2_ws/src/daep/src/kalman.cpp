/**
* Implementation of KalmanFilter class.
*
* @author: Hayk Martirosyan
* @date: 2014.11.15

Inspired by 
https://github.com/hmartiro/kalman-cpp

Modified by Emil Wiman.
*/

#include <iostream>
#include <stdexcept>
#include <iomanip>


#include "kalman.h"

KalmanFilter::KalmanFilter(
    double dt,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
  : A(A), C(C), Q(Q), R(R), P(P),
    m(C.rows()), n(A.rows()), dt(dt), initialized(false),
    I(n, n), x_hat(n), x_hat_new(n), previous_measurement(m)
{
  I.setIdentity();
  previous_measurement.setZero();
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0){
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  // Initialize previous_measurement with the measured part of x0 (first m elements)
  // m is the number of measurements (2 for walking obstacles, 3 for flying obstacles)
  previous_measurement = x0.head(m);
  initialized = true;
}

void KalmanFilter::init() {
  x_hat.setZero();
  t0 = 0;
  t = t0;
  initialized = true;
  // Initialize previous_measurement with size m (number of measurements)
  previous_measurement = Eigen::VectorXd::Zero(m);
}

void KalmanFilter::predict()
{
  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");
  
  x_hat = A * x_hat;
  P = A*P*A.transpose() + Q;
  t += dt;
}

std::pair<Eigen::VectorXd, Eigen::MatrixXd> KalmanFilter::predict(Eigen::VectorXd x_hat_input, Eigen::MatrixXd P_input)
{

  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  Eigen::VectorXd x_hat_new = A * x_hat_input;
  Eigen::MatrixXd P_new = A*P_input*A.transpose() + Q;
  
  return std::make_pair(x_hat_new, P_new);
}


void KalmanFilter::update(const Eigen::VectorXd& y) {

  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  Eigen::VectorXd innovation = y - C*x_hat;

  if(innovation.norm() > 1.0)
  {
    // Only compute velocity for 2D measurements (walking obstacles)
    // For 3D measurements (flying obstacles), this code path shouldn't be used
    if (y.size() == 2) {
      Eigen::VectorXd new_velocity = (y - previous_measurement) / dt;
      x_hat(0) = y(0);
      x_hat(1) = y(1);
      x_hat(2) = new_velocity(0);
      x_hat(3) = new_velocity(1);
    } else {
      // For 3D case, just update positions directly
      Eigen::VectorXd new_velocity = (y - previous_measurement) / dt;
      x_hat(0) = y(0);
      x_hat(1) = y(1);
      x_hat(2) = y(2);
      x_hat(3) = new_velocity(0);
      x_hat(4) = new_velocity(1);
      x_hat(5) = new_velocity(2);
    }
  }

  K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
  x_hat += K * (y - C*x_hat);
  P = (I - K*C)*P;

  // Store current measurement for next velocity calculation
  previous_measurement = y;
}

void KalmanFilter::update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A, const Eigen::MatrixXd Q) {

  this->A = A;
  this->dt = dt;
  this->Q = Q;
  update(y);
}

void KalmanFilter::print_x_hat()
{
  std::cout << "----------X_HAT----------" << std::endl;
  std::cout << x_hat;
  std::cout << "-------------------------" << std::endl;
}

void KalmanFilter::print_P()
{
  std::cout << "------------P---------" << std::endl;
  std::cout << P;
  std::cout << "-------------------------" << std::endl;
}
