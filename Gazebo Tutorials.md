[TOC]

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

## Build a Robot

### Model structure and requirements

Gazebo is able to dynamically load models into simulation either programmatically or through the GUI.

Models in Gazebo define a physical entity with dynamic, kinematic, and visual properties.

Gazebo relies on a database to store and maintain models available for use within simulation.

#### The Model Database Repository

The model database is a bitbucket repository.

```
hg clone https://bitbucket.org/osrf/gazebo_models
```

#### Model Database Structure

A model database must abide by a specific directory and file structure. The root of a model database contains one directory for each model, and a **database.config** file with information about the database. Each model directory also has a **model.config** file that contains meta data about the model. A model directory also contains the SDF for the model and any materials, meshes, and plugins.

The structure is as follows:

- Database
  - database.config: Meta data about the database. This is now populated automatically from CMakeLists.txt
  - model_1: A directory for model_1
    - model.config: Meta-data about model_1
    - model.sdf: SDF description of the model
    - modef.sdf.erb: Ruby embedded SDF model description
    - meshes: A directory for all COLLADA and STL files
    - materials: A directory which should only contain the **textures** and **scripts** subdirectories
      - textures: A directory for image files (jpg, png, etc)
      - scripts: A directory for OGRE material scripts
    - plugins: A directory for plugin source and header files

##### Database Config

This file contains license information for the models, a name for the database, and a list of all the valid models.

Note: The database.config file is only required for online repositories.

The format of this database.config is:

```xml
<?xml version='1.0'?>
<database>
    <name>name_of_this_databae</name>
    <license>Creative Commons Attribution 3.0 Unported</license>
    <models>
        <uri>file://model_directory</uri>
    </models>
</database>
```

##### Model Config

Each model must have a model.config file in the model's root directory that contains meta information about the model. The format of this model.config is:

```xml
<?xml version="1.0?>
<model>
    <name>My Model Name</name>
    <version>1.0</version>
    <sdf version='1.5'>model.sdf</sdf>
    
    <author>
    	<name>My name</name>
        <email></email>
    </author>
    
    <description>
        A description of the model
    </description>
</model>
```

##### Model SDF 

Each model requires a model.sdf file that contains the Simulator Description Format of the model.

##### Model SDF.ERB

Standard SDF file which can contain ruby code embedded. This option is used to programatically generate SDF files using **Embedded Ruby code** templates.

## How to contribute a model

### Creating a model

Create a direcetory for your model under the gazebo_models directory. 

That directory must include the file model.config and at least one **.sdf** file. 

Also make sure you add the model directory to the **CMakeLists.txt** file.

### Contents of Model.config:

The model.config file provides information necessary to pick the pproper SDF file, information on authorship of the model, and a textual description of the model.

