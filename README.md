# Walker ROS2 Package

## Overview
The `walker` package implements a simple walker behavior using the State Design Pattern. The walker navigates within a Gazebo simulation environment and transitions between states like "Idle" and "Moving" based on certain criteria.

This package is built using **ROS2 Humble** and adheres to **Google C++ Style Guide**. It includes:
- Object-Oriented Programming (OOP) principles for modularity.
- State Design Pattern to manage walker behavior.
- Integration with `rosbag2` for recording simulation data.
- Comprehensive documentation with Doxygen comments.

---

## Features
- **State Design Pattern**: Implements a state machine for walker behavior with extendable states.
- **Launch File**: Simplifies running the walker node and recording rosbag files.
- **Rosbag Recording**: Records all topics except `/camera/*` to avoid large file sizes, with an option to enable/disable recording.
- **Code Quality**: Follows Google C++ Style Guide, validated with `cpplint` and `clang-tidy`.

---

## Requirements
- **ROS2 Humble**: Ensure ROS2 Humble is installed and sourced.
- **Gazebo**: Required for simulation.
- **colcon**: Build tool for ROS2 packages.

---

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/<your-username>/my_gazebo_tutorials.git
   cd my_gazebo_tutorials
   ```
2. Build the package:
   ```bash
   colcon build
   ```

3.  Source overlay:
   ```bash
   source install/setup.bash
   ```
## Usage
### Run the Walker Node

To start the walker node in simulation:
 ```bash
 ros2 launch walker walker_launch.py
```

### Enable Rosbag Recording
To enable rosbag recording:

```bash
ros2 launch walker walker_launch.py record_bag:=true
```

## Run cppcheck and cpplint

For cppcheck (Execute from root of the package)
```bash
cppcheck --enable=all --std=c++11 --std=c++17 --enable=information --check-config --suppress=missingInclude --suppress=*:*test*/ --suppress=unmatchedSuppression $( find . -name *.cpp | grep -vE -e "^./build/")
```
For cpplint (Execute from root of the package)
```bash
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order src/*.cpp
`````
