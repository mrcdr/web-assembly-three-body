#pragma once

/**
 * @brief This file defines utility classes to build up polygons.
 */

#include <GL/gl.h>

#include <Eigen/Dense>
#include <array>
#include <vector>

/* Each vertex consists of 3 components of its coordinate and 3 components of its normal vector */
template <typename T>
using Vertex = std::array<T, 6>;

/**
 * Struct describing the vertices in triangle strips
 */
struct VertexStrip {
  /** The first vertex index in the strip */
  size_t first;
  /** The number of vertices in the strip */
  size_t count;

  VertexStrip(size_t first, size_t count) : first(first), count(count) {}
};

/**
 * @brief This class helps to construct 3D graphic polygon.
 */
template <typename T>
class PolygonBuilder {
 private:
  /** Point list */
  std::vector<Eigen::Vector3<T>> p;
  /** Current vertex normal vector */
  Eigen::Vector3<T> normal;
  /** Staring index of the strip that is being processed */
  size_t strip_start_index = 0;
  /** Vertices of strip the strip that is being processed */
  std::vector<Vertex<T>> vertices;
  /** Resulted strip information */
  std::vector<VertexStrip> strips;

 public:
  void addPoint(T x, T y, T z) {
    this->p.emplace_back(x, y, z);
  }

  PolygonBuilder& addVertex(size_t point) {
    this->vertices.push_back(
        {this->p[point](0), this->p[point](1), this->p[point](2), this->normal(0), this->normal(1), this->normal(2)});

    return *this;
  }

  PolygonBuilder& startStrip() {
    this->strip_start_index = this->vertices.size();

    return *this;
  }

  void endStrip() {
    size_t count = this->vertices.size() - strip_start_index;
    this->strips.emplace_back(strip_start_index, count);
  }

  PolygonBuilder& setNormal(const Eigen::Vector3<T>& normal) {
    this->normal = normal;

    return *this;
  }

  PolygonBuilder& setNormal(size_t index) {
    this->setNormal(this->p[index].normalized());

    return *this;
  }

  const auto& getVertices() const {
    return this->vertices;
  }

  const auto& getStrips() const {
    return this->strips;
  }
};

/**
 * @brief This class stores a graphic polygon information in 3D.
 */
template <typename T>
class Polygon {
 private:
  /** The array of vertices */
  std::vector<Vertex<T>> vertices;

  /** The array of triangle strips */
  std::vector<VertexStrip> strips;

  /** The Vertex Buffer Object holding the vertices in the graphics card */
  GLuint vbo;

  /* Color */
  Eigen::Vector4<T> color;

 public:
  Polygon(const PolygonBuilder<T>& builder, const Eigen::Vector4<T>& color)
      : vertices(builder.getVertices()), strips(builder.getStrips()), color(color) {}

  /**  Store the vertices as a vertex buffer object (VBO) */
  void storeToBuffer() {
    glGenBuffers(1, &this->vbo);
    glBindBuffer(GL_ARRAY_BUFFER, this->vbo);
    glBufferData(GL_ARRAY_BUFFER, this->vertices.size() * sizeof(Vertex<T>), this->vertices.data(), GL_STATIC_DRAW);
  }

  auto getVBO() const {
    return this->vbo;
  }

  const auto& getStrips() const {
    return this->strips;
  }

  auto getColor() const {
    return this->color.data();
  }

  static Polygon<T> createSphere(T radius, size_t num_phi, size_t num_theta, const Eigen::Vector4<T>& color) {
    PolygonBuilder<T> builder;

    for (size_t j = 0; j <= num_theta; ++j) {
      T theta = M_PI * (T)j / (T)num_theta;
      T z = std::cos(theta);
      T rxy = std::sin(theta);  // radius in xy-plane

      for (size_t i = 0; i <= num_phi; ++i) {
        T phi = 2.0 * M_PI * (T)i / (T)num_phi;
        T x = rxy * std::cos(phi);
        T y = rxy * std::sin(phi);

        builder.addPoint(radius * x, radius * y, radius * z);
      }
    }

    builder.startStrip();
    for (size_t j = 0; j < num_theta; ++j) {
      for (size_t i = 0; i <= num_phi; ++i) {
        size_t offset = (num_phi + 1) * j + i;

        builder.setNormal(offset);
        builder.addVertex(offset);
        builder.setNormal(offset + num_phi + 1);
        builder.addVertex(offset + num_phi + 1);
      }
    }
    builder.endStrip();

    return Polygon(builder, color);
  }
};

/**
 * @brief This class manages a trajectory and its graphic polygon in 3D.
 */
template <typename T, typename GLFP_T>
class Trajectory {
 private:
  /** Trajectory polygon in world coordinate */
  std::vector<Eigen::Vector3<GLFP_T>> polygon;

  /** Polygon color */
  Eigen::Vector4<GLFP_T> color;

  /** Previous position in world coordinate */
  Eigen::Vector3<T> previous_position{0, 0, 0};

  /** Previous cutting plane of the polygon */
  std::vector<Eigen::Vector3<T>> previous_cut;

 public:
  Trajectory(const Eigen::Vector4<GLFP_T>& color) : color(color) {}

  /** Pushes an point that compose the trajectory.
   * @param position The position to be added
   *
   * @note Trajectory is expressed as a regular polygon tube
   */
  void pushTrajectory(const Eigen::Vector3<T>& position) {
    const size_t num_corner = 3;
    const T radius = 0.02;

    Eigen::Vector3<T> direction = (position - this->previous_position).normalized();
    Eigen::Vector3<T> orthonormal{direction[1], -direction[0], 0};
    if (orthonormal[0] == 0 && orthonormal[1] == 0) {
      orthonormal[0] = 1;
    }
    orthonormal.normalize();
    orthonormal *= radius;

    std::vector<Eigen::Vector3<T>> cut;
    for (size_t i = 0; i <= num_corner; ++i) {
      const T angle = 2.0 * M_PI * i / num_corner;
      Eigen::Vector3<T> e =
          Eigen::Transform<T, 3, Eigen::Affine>(Eigen::AngleAxis<T>(angle, direction)).linear() * orthonormal;
      cut.push_back(position + e);
    }

    if (this->previous_cut.size() > 0) {  // If the point is not the first one, construct the tube
      for (size_t i = 0; i <= num_corner; ++i) {
        this->polygon.push_back(
            {(GLFP_T)this->previous_cut[i][0], (GLFP_T)this->previous_cut[i][1], (GLFP_T)this->previous_cut[i][2]});
        this->polygon.push_back({(GLFP_T)cut[i][0], (GLFP_T)cut[i][1], (GLFP_T)cut[i][2]});
      }
    }

    this->previous_position = position;
    this->previous_cut = cut;
  }

  const std::vector<Eigen::Vector3<GLFP_T>>& getPolygon() const {
    return this->polygon;
  }

  auto getColor() const {
    return this->color.data();
  };
};