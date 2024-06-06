# 3D Engine Demo

![](https://github.com/vv1141/3d_engine_demo/blob/master/demo.gif)

3D Engine Demo is a custom 3D physics and graphics engine with a simple testbed attached. It is mostly written as a side effect of learning about techniques to simulate physics for game development purposes, and as such is neither a finished library nor optimised for performance.

## Features

### Physics
* Collision detection of arbitrary polyhedrons using separating axis tests
* Efficient complex mesh collision detection
* Single-frame contact manifold generation, joining, and point reduction
* Collision response with positional correction
* Iterative constraint solver using sequential impulse method
* Vehicle physics using springs and constraints

### Graphics
* VBO indexing
* Phong shading
* Normal mapping
* Relief mapping
* Cascaded shadow maps
* Multisample anti-aliasing
* Gamma correction

## Building from source

### Dependencies

* [OpenGL](https://www.opengl.org/)
* [GLEW](https://glew.sourceforge.net/)
* [GLM](https://github.com/g-truc/glm)
* [SFML](https://www.sfml-dev.org/)

The required packages on Debian, for example:

```bash
libgl1-mesa-dev
libglu1-mesa-dev
libglew-dev
libglm-dev
libsfml-dev
```

### Build and run:

```bash
git clone https://github.com/vv1141/3d_engine_demo.git
cd 3d_engine_demo
cmake .
make
cd bin
./3d_engine
```

## Pre-built binaries

Pre-built binaries (x86 only) are available for Linux and Windows.
