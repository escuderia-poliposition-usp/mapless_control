# Mapless Control ROS Package for F1tenth

ROS Package with Implementation of mapless control strategies for the F1tenth.

## Description

This repository contains the implementation of mapless control strategies for the F1tenth - i.e. the Follow-The-Gap Method. The project aims to enable autonomous navigation of the F1tenth vehicle without relying on pre-built maps.

## Usage

To use the mapless control strategies, follow the instructions below:

1. Clone the repository:
   ```sh
   git clone https://github.com/ebva-autonomous-vehicles/mapless_control.git
   ```

2. Run Autodrive's simulation and ROS drivers

3. Run Follow-The-Gap node
   ```sh
      ros2 run mapless_control follow_the_gap
   ```