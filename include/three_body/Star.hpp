#pragma once

/**
 * @brief This file defines a star object to be drawn.
 */

#include <Eigen/Dense>
#include <cmath>
#include <vector>

#include "Polygon.hpp"
#include "three_body/Gravity.hpp"
#include "three_body/MatrixUtils.hpp"

template <typename T, typename GLFP_T>
class Star : public MassiveObject<T> {
  /** Trajectory */
  Trajectory<T, GLFP_T> trajectory;

  /** Polygon model of the star */
  Polygon<GLFP_T> polygon;

 public:
  Star(GLFP_T radius, size_t nSlices, size_t nStacks, const Eigen::Vector4<GLFP_T>& color, T mass)
      : MassiveObject<T>(mass),
        trajectory(color),
        polygon(Polygon<GLFP_T>::createSphere(radius, nSlices, nStacks, color)) {}

  /** Add a trajectory point */
  void pushTrajectory() {
    this->trajectory.pushTrajectory(this->getPosition());
  }

  /** Get the trajectory polygon data */
  auto& getTrajectory() const {
    return this->trajectory;
  }

  /**  Store the vertices as a vertex buffer object (VBO) */
  void storeToBuffer() {
    this->polygon.storeToBuffer();
  }

  auto& getPolygon() const {
    return this->polygon;
  }
};

template <typename T, typename GLFP_T>
inline std::vector<std::unique_ptr<Star<T, GLFP_T>>> getPythagoreanThreeBody() {
  std::vector<std::unique_ptr<Star<T, GLFP_T>>> stars;
  const Eigen::Vector4<GLFP_T> white{1.0, 1.0, 1.0, 1.0};
  const Eigen::Vector4<GLFP_T> orange{1.0, 0.5, 0.0, 1.0};
  const Eigen::Vector4<GLFP_T> blue{0.8, 0.9, 1.0, 1.0};

  const T scale = 2.0;
  stars.emplace_back(new Star<T, GLFP_T>(0.5, 24, 24, white, 3));
  stars.back()->setPosition(Eigen::Vector3<T>{1, 3, 0} * scale);

  stars.emplace_back(new Star<T, GLFP_T>(0.5, 24, 24, orange, 4));
  stars.back()->setPosition(Eigen::Vector3<T>{-2, -1, 0} * scale);

  stars.emplace_back(new Star<T, GLFP_T>(0.5, 24, 24, blue, 5));
  stars.back()->setPosition(Eigen::Vector3<T>{1, -1, 0} * scale);

  return stars;
}
