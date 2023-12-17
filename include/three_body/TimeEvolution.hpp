#pragma once

/**
 * @brief This file defines general functions to simulate time evolution.
 */

#include <Eigen/Dense>
#include <cmath>

/**
 * This structure gives constants known as a Butcher tableau of Runge-Kutta-Fehlberg method.
 */
template <typename T>
struct Constants {
  /* clang-format off */
  static inline const size_t RANK = 6;
  static inline const Eigen::Matrix<T, RANK, RANK> A {
    { 0.0,            0.0,            0.0,            0.0,            0.0,       0.0},
    { 1.0/4.0,        0.0,            0.0,            0.0,            0.0,       0.0},
    { 3.0/32.0,       9.0/32.0,       0.0,            0.0,            0.0,       0.0},
    { 1932.0/2197.0, -7200.0/2197.0,  7296.0/2197.0,  0.0,            0.0,       0.0},
    { 439.0/216.0,   -8.0,            3680.0/513.0,  -845.0/4104.0,   0.0,       0.0},
    {-8.0/27.0,       2.0,           -3544.0/2565.0,  1859.0/4104.0, -11.0/40.0, 0.0}
  };
  // Upper triangle elements will not be used;
  // this means k_i can be calculated only by k_j (j < i),
  // namely, "explicit".

  static inline const Eigen::Vector<T, RANK> B4 {
    25.0/216.0, 0.0, 1408.0/2565.0, 2197.0/4104.0, -1.0/5.0, 0.0
  };

  static inline const Eigen::Vector<T, RANK> B5 {
    16.0/135.0, 0.0, 6656.0/12825.0, 28561.0/56430.0, -9.0/50.0, 2.0/55.0
  };

  static inline const Eigen::Vector<T, RANK> C {
    0.0, 1.0/4.0, 3.0/8.0, 12.0/13.0, 1.0, 1.0/2.0
  };
  /* clang-format on */
};

template <typename T>
T evolute(Eigen::VectorX<T>& x, const std::function<Eigen::VectorX<T>(const Eigen::VectorX<T>&)>& f, T delta_t) {
  const static T eps = 1e-16;
  const static T min_delta_t = 1e-16;
  const static Eigen::Matrix<T, 1, Constants<T>::RANK> DELTA_B = (Constants<T>::B5 - Constants<T>::B4).transpose();

  const size_t n = x.size();
  Eigen::MatrixX<T> k = Eigen::MatrixX<T>::Zero(Constants<T>::RANK, n);

  for (size_t i = 0; i < Constants<T>::RANK; ++i) {
    k.row(i) = f(x + delta_t * (Constants<T>::A.row(i) * k).transpose()).transpose();
  }

  T torr = (delta_t * DELTA_B * k).norm();
  T next_delta_t = delta_t;

  if (torr > eps) {
    delta_t = std::max(min_delta_t, 0.8 * delta_t * std::pow(eps / torr, 1.0 / (Constants<T>::RANK - 1)));
    next_delta_t = delta_t;
    k.setZero();

    // Recalculation
    for (size_t i = 0; i < Constants<T>::RANK; ++i) {
      k.row(i) = f(x + delta_t * (Constants<T>::A.row(i) * k).transpose()).transpose();
    }
  } else if (torr < eps * 0.1) {
    next_delta_t = 2 * delta_t;
  }

  x += delta_t * Constants<T>::B5.transpose() * k;
  return next_delta_t;
}