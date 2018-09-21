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

### Model Files

A model file uses the same SDF format as world files, but should only contain a signal **<model> ... </model>**. The purpose of these files is to facilitate model reuse, and simplify world files. Once a model file is created, it can be included in a world file using the the following SDF syntax:

```xml
<include>
    <uri>model://model_file_name</uri>
</include>
```

### Environment Variables

Default values that work for most cases are compiled in.

Here are the variables:

- GAZEBO_MODEL_PATH: colon-separated set of directories where Gazebo will search for models
- GAZEBO_RESOURCE_PATH:  colon-separated set of directories where Gazebo will search for other resources such as world and media files.
- GAZEBO_MASTER_URI: URI of the Gazebo master. 
- GAZEBO_PLUGIN_PATH:  colon-separated set of directories where Gazebo will search for the plugin shared libraries at runtime.
- GAZEBO_MODEL_DATABASE_URI: URI of the online model database where Gazebo will download models from.

### Gazebo Server

The servevr is the workhorse of Gazebo. It parses a world description file given on the command line, and then simulates the world using a physics and sensor engine.

The server can be started using the following command:

```
gzserver <world_filename>
```

### Graphical Client

The graphical client connects to a running gzserver and visualizes the elements.

```
gzclient
```

### Plugins

Plugins provide a simple and convenient mechanism to interface with Gazebo. Plugins can either be loaded on the command line, or specified in an SDF file.

Plugins specified on the command line are loaded first, the plugins specified in the SDF files are loaded. Some plugins are loaded by the server, such as plugins which affect physics properties, while other lpugins are loaded by the graphical client to facilitate custom GUI generation.

Example of loading a system plugin via the command line:

```
gzserver -s <plugin_filename>
```

## Gazebo Architecture

Gazebo uses a distributed architecture with separate libraries for physics simulation, rendering, user interface, communication, and sensor  generation. 

The client and server communicate using the gazebo communicaton library.

### Communication Between Processes

The communication library currently uses the open source **Google Protobuf** for the message serializaton and **boost::ASIO** for the transport mechanism. It supports the publish/subsribe communication paradigm.

### Systom

#### Gazebo Master

This is essentially a topic name server. It provides name lookup, and topic management. A single master can handle multiple physics simulations, sensor generators, and GUIs.

#### Communication library

- Dependencies: Protobuf and boost::ASIO
- External API: Support communication with Gazebo nodes over named topics
- Advertised Topics: None
- Subscribed Topics: None

#### Physics Library

- Dependencies: Dynamics engine (with internal collision detection)
- External API: Provides a simple and generic interface to physics simulation
- Internal API: Defines a fundamental interface to the physics library for 3rd party dynamic engines.

The physics library provides a simple and generic interface to fundamental simulation components, including rigid bodies, collision shapes, and joints for representing articulation constraints. This interface has been integrated with four open-source physics engines:

- Open Dynamics Engine (ODE)
- Bullet
- Simbody
- Dynamic Animation and Robotics Toolfit (DART)

A model descibed in the SDF using XML can be loaded by each of these physics engines. This provides access to different algorithm implementations and simulation features.

#### Rendering Library

- Dependencies: OGRE
- External API: Allows for loading, initialization, and scene creation 
- Internal API: Store metadata for visualization, call the OGRE API for rendering.

The rendering library uses OGRE to provide a simple interface for rendering 3D scenes to both the GUI and sensor libraries. It includes lighting, textures, and sky simulation. It is possible to write plugins for the rendering engine.

#### Sensor Generation

- Dependencies: Rendering Library, Physics Library
- External API: Provide functionality to initialize and run a set of sensors
- Internal API: TBD

The sensor generation library implements all the various types of sensors, listens to world state updates from a physics simulator and produces output specified by the instantiated sensors.

#### GUI

- Dependencies: Rendering Library, QT
- External API: None
- Internal API: None

The GUI library uses Qt to create graphical widgets for users to interact with the simulation. 

#### Plugins

The physics, sensor, and rendering libraries support plugins. These plugins users with access to the respective libraries without using the communication system.