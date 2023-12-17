#pragma once

/**
 * @brief This file defines classes and functions for physical systems with gravitation.
 */

#include <Eigen/Dense>
#include <functional>
#include <memory>
#include <three_body/TimeEvolution.hpp>
#include <vector>

/**
 * Class to express a 3D mass point.
 */
template <typename T>
class MassiveObject {
 private:
  /** Mass */
  T mass;
  /** Postion in world coordinate */
  Eigen::Vector3<T> position;
  /** Velocity in world coordinate */
  Eigen::Vector3<T> velocity;

 public:
  MassiveObject(T mass) : mass(mass), position({0, 0, 0}), velocity({0, 0, 0}) {}

  virtual ~MassiveObject() = default;

  auto getMass() const {
    return this->mass;
  }

  const auto& getPosition() const {
    return this->position;
  }

  void setPosition(const decltype(MassiveObject::position)& position) {
    this->position = position;
  }

  const auto& getVelocity() const {
    return this->velocity;
  }

  void setVelocity(const decltype(MassiveObject::velocity)& velocity) {
    this->velocity = velocity;
  }
};

template <typename T>
inline auto gravityAcceleration(const std::vector<std::unique_ptr<MassiveObject<T>>>& objects) {
  return [&objects](const Eigen::VectorX<T>& x) {
    const size_t n = x.size() / 6;
    Eigen::VectorX<T> x_diff = Eigen::VectorX<T>::Zero(x.size());

    /* Update position differential by velocity */
    for (size_t i = 0; i < n; ++i) {
      x_diff[n * 3 + i * 3 + 0] = x[i * 3 + 0];
      x_diff[n * 3 + i * 3 + 1] = x[i * 3 + 1];
      x_diff[n * 3 + i * 3 + 2] = x[i * 3 + 2];
    }

    /* Update velocity differential by position */
    for (size_t i = 0; i < n; ++i) {
      for (size_t j = 0; j < n; ++j) {
        if (i == j) {
          continue;
        }

        T rx = x[n * 3 + i * 3 + 0] - x[n * 3 + j * 3 + 0];
        T ry = x[n * 3 + i * 3 + 1] - x[n * 3 + j * 3 + 1];
        T rz = x[n * 3 + i * 3 + 2] - x[n * 3 + j * 3 + 2];
        T r3 = std::pow(rx * rx + ry * ry + rz * rz, 3.0 / 2.0);

        const auto mass = objects[j]->getMass();

        x_diff[i * 3 + 0] += -mass * rx / r3;
        x_diff[i * 3 + 1] += -mass * ry / r3;
        x_diff[i * 3 + 2] += -mass * rz / r3;
      }
    }

    return x_diff;
  };
}

template <typename T>
inline void proceed(std::vector<std::unique_ptr<MassiveObject<T>>>& objects, T delta_t_initial) {
  const size_t n = objects.size();
  static Eigen::VectorX<T> x(n * 6);

  /* Convert object positions and velocities into 1D vector */
  for (size_t i = 0; i < n; ++i) {
    const Eigen::Vector3<T>& position = objects[i]->getPosition();
    const Eigen::Vector3<T>& velocity = objects[i]->getVelocity();
    x[i * 3 + 0] = velocity[0];
    x[i * 3 + 1] = velocity[1];
    x[i * 3 + 2] = velocity[2];
    x[n * 3 + i * 3 + 0] = position[0];
    x[n * 3 + i * 3 + 1] = position[1];
    x[n * 3 + i * 3 + 2] = position[2];
  }

  T proceeded = 0.0;
  T delta_t = std::min(delta_t_initial, 1e-3);
  while (proceeded < delta_t_initial) {
    delta_t = evolute<T>(x, gravityAcceleration(objects), delta_t);
    proceeded += delta_t;
  }

  /* Convert back 1D vector into positions and velocities */
  for (size_t i = 0; i < n; ++i) {
    objects[i]->setVelocity(Eigen::Vector3<T>{x[i * 3 + 0], x[i * 3 + 1], x[i * 3 + 2]});
    objects[i]->setPosition(Eigen::Vector3<T>{x[n * 3 + i * 3 + 0], x[n * 3 + i * 3 + 1], x[n * 3 + i * 3 + 2]});
  }
}