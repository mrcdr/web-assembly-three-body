#define GL_GLEXT_PROTOTYPES

#include <GL/gl.h>
#include <GL/glut.h>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <three_body/Gravity.hpp>
#include <three_body/MatrixUtils.hpp>
#include <three_body/Star.hpp>
#include <three_body/TimeEvolution.hpp>

using Eigen::Matrix;
using Eigen::Vector3;
using Eigen::Vector4;

using ft = double;

static GLuint program0, program1;

static Vector3<GLfloat> view_rotation = {0.0, 0.0, 0.0};
static Vector3<GLfloat> view_translation = {0.0, 0.0, -20};

Eigen::Vector2<int> mouse_down_at;
Eigen::Vector2<int> rotation_offset;
bool is_mouse_down = false;

static std::vector<std::unique_ptr<Star<ft, GLfloat>>> stars;

/** Projection matrix */
static Matrix<GLfloat, 4, 4> projection_matrix;

/** Vertex Buffer Object for trajectory drawing */
static GLuint trajectory_vbo;

void set_perspective_matrix(Matrix<GLfloat, 4, 4>& m, GLfloat fovy, GLfloat aspect, GLfloat z_near, GLfloat z_far) {
  m = Matrix<GLfloat, 4, 4>::Identity();

  GLfloat radians = fovy / 2 * M_PI / 180;

  GLfloat delta_z = z_far - z_near;
  GLfloat cosine = std::cos(radians);
  GLfloat sine = std::sin(radians);

  GLfloat cotangent = cosine / sine;

  m(0, 0) = cotangent / aspect;
  m(1, 1) = cotangent;
  m(2, 2) = -(z_far + z_near) / delta_z;
  m(3, 2) = -1;
  m(2, 3) = -2 * z_near * z_far / delta_z;
  m(3, 3) = 0;
}

static void draw_trajectory(const Star<ft, GLfloat>& star, const Matrix<GLfloat, 4, 4>& transform) {
  const auto& trajectory = star.getTrajectory();
  glUseProgram(program1);

  glUniform4fv(glGetUniformLocation(program1, "MaterialColor"), 1, trajectory.getColor());

  glUniformMatrix4fv(glGetUniformLocation(program1, "ModelViewMatrix"), 1, GL_FALSE, transform.data());

  Matrix<GLfloat, 4, 4> view_projection = projection_matrix * transform;
  glUniformMatrix4fv(glGetUniformLocation(program1, "ModelViewProjectionMatrix"), 1, GL_FALSE, view_projection.data());

  Matrix<GLfloat, 4, 4> normal_matrix(transform.inverse());
  normal_matrix.transposeInPlace();
  glUniformMatrix4fv(glGetUniformLocation(program1, "NormalMatrix"), 1, GL_FALSE, normal_matrix.data());

  glBindBuffer(GL_ARRAY_BUFFER, trajectory_vbo);
  glBufferData(GL_ARRAY_BUFFER,
               trajectory.getPolygon().size() * sizeof(GLfloat) * 3,
               trajectory.getPolygon().data(),
               GL_DYNAMIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLfloat*)nullptr);

  glEnableVertexAttribArray(0);

  glDrawArrays(GL_TRIANGLE_STRIP, 0, trajectory.getPolygon().size());

  glDisableVertexAttribArray(0);
}

static void draw_star(const Star<ft, GLfloat>& star, const Matrix<GLfloat, 4, 4>& transform) {
  glUseProgram(program0);

  Matrix<GLfloat, 4, 4> model_view(transform);
  model_view *= translationMatrix<ft, GLfloat>(star.getPosition());

  Matrix<GLfloat, 4, 4> model_view_inv = model_view.inverse();
  glUniformMatrix4fv(glGetUniformLocation(program0, "ModelViewInvMatrix"), 1, GL_FALSE, model_view_inv.data());

  Matrix<GLfloat, 4, 4> model_view_projection(projection_matrix);
  model_view_projection *= model_view;

  glUniformMatrix4fv(
      glGetUniformLocation(program0, "ModelViewProjectionMatrix"), 1, GL_FALSE, model_view_projection.data());

  Matrix<GLfloat, 4, 4> normal_matrix = model_view.inverse();
  normal_matrix.transposeInPlace();
  glUniformMatrix4fv(glGetUniformLocation(program0, "NormalMatrix"), 1, GL_FALSE, normal_matrix.data());

  const auto& polygon = star.getPolygon();

  glUniform4fv(glGetUniformLocation(program0, "MaterialColor"), 1, polygon.getColor());

  glBindBuffer(GL_ARRAY_BUFFER, polygon.getVBO());

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLfloat*)0);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLfloat*)0 + 3);

  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);

  for (const auto& strip : polygon.getStrips()) {
    glDrawArrays(GL_TRIANGLE_STRIP, strip.first, strip.count);
  }

  glDisableVertexAttribArray(1);
  glDisableVertexAttribArray(0);
}

static void display() {
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  Matrix<GLfloat, 4, 4> transform = Matrix<GLfloat, 4, 4>::Identity();
  transform = translationMatrix<GLfloat, GLfloat>(view_translation);
  transform *= rotationMatrix<GLfloat>(2 * M_PI * view_rotation[0] / 360.0, 1, 0, 0);
  transform *= rotationMatrix<GLfloat>(2 * M_PI * view_rotation[1] / 360.0, 0, 1, 0);
  transform *= rotationMatrix<GLfloat>(2 * M_PI * view_rotation[2] / 360.0, 0, 0, 1);

  for (auto& star : stars) {
    draw_trajectory(*star, transform);
  }

  for (auto& star : stars) {
    draw_star(*star, transform);
  }

  glutSwapBuffers();
}

static void reshape(int width, int height) {
  set_perspective_matrix(projection_matrix, 60.0, width / (float)height, 1.0, 1024.0);
  glViewport(0, 0, (GLint)width, (GLint)height);
}

static void special(int special, [[maybe_unused]] int x, [[maybe_unused]] int y) {
  switch (special) {
    case GLUT_KEY_LEFT:
      view_translation[0] -= 0.1;
      break;
    case GLUT_KEY_RIGHT:
      view_translation[0] += 0.1;
      break;
    case GLUT_KEY_UP:
      view_translation[1] += 0.1;
      break;
    case GLUT_KEY_DOWN:
      view_translation[1] -= 0.1;
      break;
  }
}

static void keyboard(unsigned char key, [[maybe_unused]] int x, [[maybe_unused]] int y) {
  switch (key) {
    case 'r':
      stars = getPythagoreanThreeBody<ft, GLfloat>();

      for (auto& star : stars) {
        star->storeToBuffer();
      }
      break;
    case '0':
      view_translation[0] = 0;
      view_translation[1] = 0;
      view_rotation[0] = 0;
      view_rotation[1] = 0;
      rotation_offset[0] = 0;
      rotation_offset[1] = 0;
      break;
  }
}

static void motion(int x, int y) {
  view_rotation[0] = y - mouse_down_at[1] + rotation_offset[0];
  view_rotation[1] = x - mouse_down_at[0] + rotation_offset[1];
}

static void mouse(int button, int state, int x, int y) {
  if (button == GLUT_LEFT_BUTTON) {
    if (!is_mouse_down && state == GLUT_DOWN) {
      mouse_down_at[0] = x;
      mouse_down_at[1] = y;
      is_mouse_down = true;
    } else if (is_mouse_down && state == GLUT_UP) {
      rotation_offset[0] = view_rotation[0];
      rotation_offset[1] = view_rotation[1];
      is_mouse_down = false;
    }
  }
}

static void idle() {
  const ft simulation_time_factor = std::pow(2.0, 1.5);  // Simulation time / Real time
  const ft trajectory_period = 0.01;                     // Sampling period of trajectory in simulation time

  static ft t_last = 0.0;
  static ft trajectory_time = 0.0;
  ft t = glutGet(GLUT_ELAPSED_TIME) / 1000.0;

  const ft dt = (t - t_last) * simulation_time_factor;
  proceed((std::vector<std::unique_ptr<MassiveObject<ft>>>&)stars, dt);
  trajectory_time += dt;
  glutPostRedisplay();

  if (trajectory_time > trajectory_period) {
    for (auto& star : stars) {
      star->pushTrajectory();
    }
    trajectory_time = 0;
  }

  t_last = t;
}

static const std::string vertex_shader_polygon = R"---(#version 300 es

#ifdef GL_ES
precision mediump float;
#endif

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;

uniform mat4 ModelViewProjectionMatrix;
uniform vec4 MaterialColor;

out vec4 Color;
out vec3 Position;
out vec3 Normal;

void main(void) {
  Color = MaterialColor;
  Position = position;
  Normal = normal;
  gl_Position = ModelViewProjectionMatrix * vec4(position, 1.0);
}
)---";

static const std::string fragment_shader_polygon = R"---(#version 300 es

#ifdef GL_ES
precision mediump float;
#endif

uniform mat4 ModelViewInvMatrix;
uniform mat4 NormalMatrix;
uniform vec4 CameraPosition;

in vec4 Color;
in vec3 Position;

out vec4 fragColor;

void main(void) {
  vec3 R = Position;
  vec3 C = normalize(vec3(ModelViewInvMatrix * CameraPosition));
  float radius = sqrt(dot(R, R));
  float s = sqrt(radius*radius - pow(dot(R, C), 2.0));
  float t = max(0.0, s - 0.5 * radius) / radius;
  float th = 0.4;
  float glow = (clamp(0.4 / (s / radius), th, 1.0) - th) / (1.0 - th);
  fragColor = vec4(Color.rgb, Color.a * glow);
}
)---";

static const std::string vertex_shader_trajectory = R"---(#version 300 es

#ifdef GL_ES
precision mediump float;
#endif

layout (location = 0) in vec3 position;

uniform mat4 ModelViewProjectionMatrix;
uniform vec4 MaterialColor;

out vec4 Color;

void main(void) {
  Color = MaterialColor;
  gl_Position = ModelViewProjectionMatrix * vec4(position, 1.0);
}
)---";

static const std::string fragment_shader_trajectory = R"---(#version 300 es

#ifdef GL_ES
precision mediump float;
#endif

in vec4 Color;

out vec4 fragColor;

void main(void) {
  fragColor = Color;
}
)---";

static void initGL() {
  const char* char_pointer_store;
  std::string info_log(512, '\0');

  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  char_pointer_store = vertex_shader_polygon.c_str();
  const GLint v0 = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(v0, 1, &char_pointer_store, NULL);
  glCompileShader(v0);
  glGetShaderInfoLog(v0, info_log.size(), NULL, info_log.data());
  std::cout << "Vertex shader for polygons: " << info_log << std::endl;

  char_pointer_store = fragment_shader_polygon.c_str();
  const GLint f0 = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(f0, 1, &char_pointer_store, NULL);
  glCompileShader(f0);
  glGetShaderInfoLog(f0, info_log.size(), NULL, info_log.data());
  std::cout << "Fragment shader for polygons: " << info_log << std::endl;

  char_pointer_store = vertex_shader_trajectory.c_str();
  const GLint v1 = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(v1, 1, &char_pointer_store, NULL);
  glCompileShader(v1);
  glGetShaderInfoLog(v1, info_log.size(), NULL, info_log.data());
  std::cout << "Vertex shader for trajectories: " << info_log << std::endl;

  char_pointer_store = fragment_shader_trajectory.c_str();
  const GLint f1 = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(f1, 1, &char_pointer_store, NULL);
  glCompileShader(f1);
  glGetShaderInfoLog(f1, info_log.size(), NULL, info_log.data());
  std::cout << "Fragment shader for trajectories: " << info_log << std::endl;

  program0 = glCreateProgram();
  glAttachShader(program0, v0);
  glAttachShader(program0, f0);

  glLinkProgram(program0);
  glGetProgramInfoLog(program0, info_log.size(), NULL, info_log.data());
  std::cout << "Program for polygons: " << info_log << std::endl;

  glUseProgram(program0);

  program1 = glCreateProgram();
  glAttachShader(program1, v1);
  glAttachShader(program1, f1);

  glLinkProgram(program1);
  glGetProgramInfoLog(program1, info_log.size(), NULL, info_log.data());
  std::cout << "Program for trajectories: " << info_log << std::endl;

  const Vector4<GLfloat> CameraPosition = {0.0, 0.0, 0.0, 1.0};
  glUniform4fv(glGetUniformLocation(program0, "CameraPosition"), 1, CameraPosition.data());
}

static void initModels() {
  stars = getPythagoreanThreeBody<ft, GLfloat>();

  for (auto& star : stars) {
    star->storeToBuffer();
  }
  glGenBuffers(1, &trajectory_vbo);
}

int main(int argc, char* argv[]) {
  glutInit(&argc, argv);
  glutInitWindowSize(1000, 1000);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE);

  glutCreateWindow("Three Body");

  /* Set up glut callback functions */
  glutIdleFunc(idle);
  glutReshapeFunc(reshape);
  glutDisplayFunc(display);
  glutSpecialFunc(special);
  glutMotionFunc(motion);
  glutMouseFunc(mouse);
  glutKeyboardFunc(keyboard);

  initGL();
  initModels();

  glutMainLoop();

  return 0;
}
