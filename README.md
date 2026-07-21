# Simulation

## Description

This is the official repository for WRover Software Simulation.

## Tools and Lanugages

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

- The test branch will be periodically merged into main after thorough testing.
