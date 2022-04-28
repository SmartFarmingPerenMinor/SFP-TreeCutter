# Welcome to the robot

## starting the repo

Once you've cloned this repo, run the [install script](https://github.com/SmartFarmingPerenMinor/SFP-TreeCutter/master/install.sh)(if you haven't already).
```bash
./install.sh
```
This sets up the environment for both the simulation and real robot.

To start working with the robot, check out our Documentation.

Currently we can:
- Simulate the UR10e
- Simulate the IntelRealsense D435
- Simulate movement
- Simulate an octomap
- Simulate a depth-map

- Connect to the real UR10e
- Move the real UR10e 
- Visualize an octomap from the real environment 

Interfaces:
- ROS: The core of our project. Learn about ROS to get started on development.
- RViz: Visualize the robot's perception. Allows for easier debugging, rather than consulting numbers and metrics.
- Gazebo: Simulation tool. Has options for physics and more.

Libraries:
- MoveIt!: Solves motion planning.

Requirements:
- Ubuntu [20.04](https://ubuntu.com/download/desktop/thank-you?version=20.04.4&architecture=amd64)

Recommended:
- Run Ubuntu natively. VMWare can be laggy and unstable.
- Consult the ROS wiki.
