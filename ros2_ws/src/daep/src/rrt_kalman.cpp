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

#include "rrt_kalman.h"

KalmanFilter1::KalmanFilter1(
    double dt,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
  : A(A), C(C), Q(Q), R(R), P(P),
    m(C.rows()), n(A.rows()), dt(dt), initialized(false),
    I(n, n), x_hat(n), x_hat_new(n)
{
  I.setIdentity();
}

KalmanFilter1::KalmanFilter1() {}

void KalmanFilter1::init(double t0, const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0){
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  previous_measurement = x0.head(2);
  initialized = true;
}

void KalmanFilter1::init() {
  x_hat.setZero();
  t0 = 0;
  t = t0;
  initialized = true;
  previous_measurement.setZero();
}

void KalmanFilter1::predict()
{
  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");
  
  x_hat = A * x_hat;
  P = A*P*A.transpose() + Q;
  t += dt;
}

std::pair<Eigen::VectorXd, Eigen::MatrixXd> KalmanFilter1::predict(Eigen::VectorXd x_hat_input, Eigen::MatrixXd P_input)
{

  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  Eigen::VectorXd x_hat_new = A * x_hat_input;
  Eigen::MatrixXd P_new = A*P_input*A.transpose() + Q;

  return std::make_pair(x_hat_new, P_new);
}


void KalmanFilter1::update(const Eigen::VectorXd& y) {

  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  Eigen::VectorXd innovation = y - C*x_hat;

  if(innovation.norm() > 1.0)
  {
    Eigen::VectorXd new_velocity = (y - previous_measurement) / dt;
    x_hat(0) = y(0);
    x_hat(1) = y(1);
    x_hat(2) = new_velocity(0);
    x_hat(3) = new_velocity(1);
  }

  K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
  this->x_hat += K * (y - C*x_hat);
  this->P = (I - K*C)*P;

  previous_measurement = y;
}

void KalmanFilter1::update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A,const Eigen::MatrixXd Q) {

  this->A = A;
  this->dt = dt;
  this->Q = Q;
  this->update(y);
}

void KalmanFilter1::print_x_hat()
{
  std::cout << "----------X_HAT----------" << std::endl;
  std::cout << x_hat;
  std::cout << "-------------------------" << std::endl;
}

void KalmanFilter1::print_P()
{
  std::cout << "------------P---------" << std::endl;
  std::cout << P;
  std::cout << "-------------------------" << std::endl;
}
