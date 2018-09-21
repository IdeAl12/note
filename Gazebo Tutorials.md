# Gazebo Tutorials

## Quick Start

### Run Gazebo

```
gazebo
```

### Run Gazebo with a robot

```
gazebo worlds/pioneer2dx.world
```

### Client and server separation

The gazebo command actually runs two different excutables gzserver, gzclient.

The gzserver executable runs the pythsics update-loop ans sensor data generation. This is  the coreof Gazebo, and can be used independently of a graphical interface.

The gzclient executable runs a QT based user interface. This application provides a nice visualization of simulation, and convenient controls over various simulation properites.

## Gazebo Components

### World Files

The world description file contains all the elements in a simulation, including robotsm lights, sensors, and static object. This file is formatted using SDF (Simulation Description Format), and typically has a .world extension.

The Gazebo server (gzserver) reads this file to generate and populate a world.

