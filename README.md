# Overview

Web Assembly and Emscripten sample to simulate the Pythagorean three-body problem.

The demo is available [here](https://mrcdr.github.io/web-assembly-three-body/).

# Compile commands

## Web Assembly

```sh
em++ three_body.cpp -std=c++17 -Iinclude -o index.html -O3 -sMAX_WEBGL_VERSION=2 -sALLOW_MEMORY_GROWTH
```

## Linux executable

```sh
c++ three_body.cpp -std=c++17 -Iinclude -O3 -lglut -lGL
```

# Dependencies

- OpenGL
- OpenGL Utility Toolkit (GLUT)
- Eigen
