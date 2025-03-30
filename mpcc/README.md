# CMake Project with an Installed Drake

This uses the CMake `find_package(drake)`
mechanism to find an installed instance of Drake.

# Instructions

## Download and Install Prerequisites

First, run the `install_prereqs` script to download the
Drake source to `$HOME/drake/`. This also run's Drake's
setup script to install the required packages depending
on your operating system:

```bash
setup/install_prereqs
```

See [below](#alternative-versions) for alternative versions of
Drake to download.

## Build Everything

This project is set up to be built as an integrated Python package using scikit-build-core.
Because scikit-build-core is basically just a thin wrapper around cmake, it is still mostly
possible to build using the traditional method of 
```bash
mkdir build && cd build && cmake .. && cmake --build .
```
However, it is preferred to build using scikit build and uv.

The first method to use creates a build directory and builds inside that
directory. This helps for IDE integration by outputting `compile_commands.json`,
which tells your IDE where to look for `#includes`. It also seems to work better
for caching.

```bash
uv venv # create a virtual environment
uv pip install scikit-build-core pytest
mkdir build
uv pip install -ve . --no-build-isolation
pytest # run tests
```

To use `mpcc` with other libraries (like the groundstation backend), you can optionally
compile a wheel using

```bash
uv build
```

which will build all source code and put a wheel in the `dist` directory.

# Current State and TODO

## Dubins Code

The Dubins code creates constant-curvature paths and solves for feasible configurations.
This code powers the path editing functionality on the frontend.

* [Dubins Segments](src/cpp/include/dubins_segment.h)
    * Working implementation for line and circular segments
* [Path](src/cpp/include/dubins_path.h)
    * Working implementation for Dubins path
* [Solver](src/cpp/include/dubins_solver.h)
    * Currently working in bare minimum form
    * TODO: Add knot-point dragging code to adjust solver behaviour
    * TODO: Explore improvements to straightforward gradient descent (eg conjugate gradients, Gauss-Newton update) 
      to increase radius of convergence and convergence speed
    * TODO: Better documentation of failure conditions

## Controller 

* [Dynamic Models](src/cpp/include/dynamics.h)
    * Working implementation of 2D and 3D coordinated turn models
    * TODO: Add tests
* TODO: Drake dynamic systems wrappers
* TODO: Drake multiple shooting trajectory optimization