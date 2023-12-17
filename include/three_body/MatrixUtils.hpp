#pragma once

/**
 * @brief This file defines utility functions for matrix operation.
 */

#include <Eigen/Dense>

/**
 * Generate a 4x4 rotation matrix.
 *
 * @param angle the angle to rotate
 * @param x the x component of the direction to rotate to
 * @param y the y component of the direction to rotate to
 * @param z the z component of the direction to rotate to
 *
 * @return the rotation matrix
 */
template <typename T>
inline Eigen::Matrix<T, 4, 4> rotationMatrix(T angle, T x, T y, T z) {
  return Eigen::Transform<T, 3, Eigen::Affine>(Eigen::AngleAxis<T>(angle, Eigen::Vector3<T>(x, y, z))).matrix();
}

/**
 * Generate a 4x4 translation matrix.
 *
 * @param x the x component of the direction to translate to
 * @param y the y component of the direction to translate to
 * @param z the z component of the direction to translate to
 *
 * @return  the translation matrix
 */
template <typename T, typename GLFP_T>
inline Eigen::Matrix<GLFP_T, 4, 4> translationMatrix(const Eigen::Vector3<T>& p) {
  Eigen::Matrix<GLFP_T, 4, 4> t = Eigen::Matrix<GLFP_T, 4, 4>::Identity();
  t(0, 3) = p[0];
  t(1, 3) = p[1];
  t(2, 3) = p[2];
  return t;
}