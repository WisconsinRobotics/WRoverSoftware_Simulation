# Simulation

## Description

This is the official repository for WRover Software Simulation.

## Tools and Lanugages
- Rosviz
- Gazebo for simulation.
  - ROS2GZ bridge
  - Xacro udrf
- Custom VSCode build scripts

Currently using Rosviz with Gazebo for simulation.

## Setup

- Clone the dev branch:

  ```bash
  git clone -b dev git@github.com:WisconsinRobotics/WRoverSoftware_Simulation.git
  ```

- Initialize pre-commit hooks:

  ```bash
  pre-commit install
  ```

- To run pre-commit checks manually:

  ```bash
  pre-commit run -a
  ```

  > We use pre-commit hooks to format code and check for simple mistakes. They automatically run before every commit.

## Workflow

- All development work should be done on the dev branch. Pull frequently and resolve merge conflicts when they arise.

- Code that is free of syntax errors and basic bugs should be merged into the test branch by opening a pull request.

## Notes
Documentation about the software being used, resources for development, things that need to be done, can all be found in [/docs/](./docs/)


# Simulation Setup
starting from the root of the WiscRobo directory (where this readme is).
- NOTE: for development purposes, it is recommended to use VScode seeing as there are custom build commands and compile tasks in VSCode
- WSL has varying support for different graphics card. If there is trouble running the software, please refer to [/docs/graphics.md](./docs/graphics.md) and contribute to such file.

### Prereqs
These are required to be installed on the system in order for the simulation to work.
```
sudo apt install python3-colcon-common-extensions
sudo apt-get install python3-rosdep2
sudo apt install ros-humble-joint-state-publisher-gui
```

### Raw Build and Run Commands

```
# Install package dependencies
source /opt/ros/humble/setup.sh
sudo apt-get update
rosdep install --from-path src

# Build projcet
colcon build

# Run Sim
source install/setup.sh
ros2 launch simulation simulation.launch.py
```

### Shortcuts
``` bash
# For fast builds use fast_build script at the top level
# This will do the entire build, source, and run routine for you
# make sure to chmod the script
./fast_build

# clean repo
make clean

# install ros dependencies
make inst_dep

# build project
make build_env

# run after source
make run
```
