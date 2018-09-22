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

The model.config file indicates that the simulator's definition of the model (i.e., visual, interial, kinematic, and geometric properties, among others) , is located in model.sdf.

### Adding the directory (files) to the repository

```
hg add mymodel
or:
hg add mymodel/model.config
hg add mymodel/model.sdf
```

## Make a model

SDF Models can range from simple shapes to complex robots. It refers to <model> SDF tag, and is  essentially a collection of links, joints, collision objects, visuals, and plugins. 

### Components of a SDF Models

Links: A link contains the physical properties of one body of the model. Each link may contain many collision and visual elements. Try to reduce the number of links in your models in order to improve performance and stability.

- Collision: A collision element encapsulates a geometry that is used to collision checking. This can be a simple shape or a triangle mesh. A link may contain many collision elements.
- visual: A visual element is used to visualize parts of a link. A link may contain 0 or more visual elements.
- Intertial: The inertial element describes the dynamic properties of the link, suck as mass and rotational intertia matrix.
- Sensor: A sensor collects data from the world for uuuse in plugins.
- Light: A light element describes a light source attached to a link.

Joints: A joint connects two links. A parent and child relationship is established along with other parameters such as axis of rotation and joint limits.

Plugins: A plugin is a shared library created by a third party to control a model.

### Building a Model

#### Step 1 : Collect meshes

This step involves gathering all the necessary 3D mesh files that are required to build your model. Gazebo provides a set of simple shapes: box, sphere, and cylinder. 

Tip: Keep meshes simple. This is especially true if you plan on using the mesh as a collision element. A common practice is to use a low polygon mesh for a collision element, and higher polygon mesh for the visual. An even better practice is to use one of the built-in shapes (box, sphere, cylinder) as the collision element.

#### Step 2: Make your model SDF file

Here is a very rudimentary minimum box model file with just a unit sized box shape as a collision geometry and the same unit box visual with unit inertias:

Create the **box.sdf** model file

```
gedit box.sdf
```

```xml
<?xml version='1.0'?>
<sdf version="1.4">
<model name="ny_model">
	<pose>0 0 0.5 0 0 0</pose>
    <static>true</static>
    	<link name="link">
            <inertial>
                <mass>1.0</mass>
                <inertia> 
                    <ixx>0.083</ixx> 
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>0.083</iyy>
                    <iyz>0.0</iyz>
                    <izz>0.083</izz>
                </inertia>
            </inertial>
            <collision name="collision">
            	<geometry>
                	<box>
                    	<size>1 1 1</size>
                    </box>
                </geometry>
            </collision>
            <visual name="visual">
            	<geometry>
                	<box>
                    	<size>1 1 1</size>	
                    </box>
                </geometry>
            </visual>
    	</link>
    </model>
</sdf>
```

Tip: The above example sets the simple bax model to be static, which makes the model immovable. Set the <static> tag to false if you want your model to be movable.

#### Step 3: Add to the model SDF file

With a working .sdf file, slowly start adding in more complexity. with each new addition. load model using the graphical client to make sure the model is correct.

Here is a good order in which to add features:

1. Add a link
2. Set the collision element
3. Set the visual element
4. Set the inertial properties
5. Go to #1 until all links have been added
6. Add all joints
7. Add all plugins

## Make a Mobile Robot

### Setup model directory

1. Create a model directory:

```
mkdir -p ~/.gazebo/models/my_robot
```

2. Create a model config file:

```
gedit ~/.gazebo/models/my_robot/model.config
```

3. model.config

```xml
<?xml version="1.0"?>
<model>
  <name>My Robot</name>
  <version>1.0</version>
  <sdf version='1.4'>model.sdf</sdf>

  <author>
   <name>My Name</name>
   <email>me@my.email</email>
  </author>

  <description>
    My awesome robot.
  </description>
</model>
```

4. Create a model.sdf file

```
gedit ~/.gazebo/models/my_robot/model.sdf
```

5. model.sdf

```xml
<?xml version='1.0'?>
<sdf version='1.4'>
  <model name="my_robot">
  </model>
</sdf>
```

### Builld the Model's Structure

This step will create a rectangular base with two wheels.

1. Make the model static by adding a static element to the model.sdf file:

```xml
<static>ture</static>
```

2. Add the rectangular base by editing the model.sdf file:

```xml
		<link name='chassis'>
            <pose>0 0 .1 0 0 0</pose>

            <collision name='collision'>
              <geometry>
                <box>
                  <size>.4 .2 .1</size>
                </box>
              </geometry>
            </collision>

            <visual name='visual'>
              <geometry>
                <box>
                  <size>.4 .2 .1</size>
                </box>
              </geometry>
            </visual>
          </link>
```

The most common use for different collision and visual elements is to have a simplified collision element paired with a visual element that uses a complex mesh. This will help improve performance.

3. Try out your model by running gazebo.
4. Add a caster to the robot. The caster is a sphere with no friction.

```xml
		<collision name='caster_collision'>
            <pose>-0.15 0 -0.05 0 0 0</pose>
            <geometry>
                <sphere>
                <radius>.05</radius>
              </sphere>
            </geometry>

            <surface>
              <friction>
                <ode>
                  <mu>0</mu>
                  <mu2>0</mu2>
                  <slip1>1.0</slip1>
                  <slip2>1.0</slip2>
                </ode>
              </friction>
            </surface>
          </collision>

          <visual name='caster_visual'>
            <pose>-0.15 0 -0.05 0 0 0</pose>
            <geometry>
              <sphere>
                <radius>.05</radius>
              </sphere>
            </geometry>
          </visual>
```

5. Add a left wheel.

```xml
<link name="left_wheel">
        <pose>0.1 0.13 0.1 0 1.5707 1.5707</pose>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>.1</radius>
              <length>.05</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>.1</radius>
              <length>.05</length>
            </cylinder>
          </geometry>
        </visual>
      </link>
```

6. Add a right wheel.

```xml
  <link name="right_wheel">
        <pose>0.1 -0.13 0.1 0 1.5707 1.5707</pose>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>.1</radius>
              <length>.05</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>.1</radius>
              <length>.05</length>
            </cylinder>
          </geometry>
        </visual>
      </link>
```

7. Make the model dynamic by setting <static> to false, and add two hinge joints for the left and right wheels.

```xml
   <joint type="revolute" name="left_wheel_hinge">
        <pose>0 0 -0.03 0 0 0</pose>
        <child>left_wheel</child>
        <parent>chassis</parent>
        <axis>
          <xyz>0 1 0</xyz>
        </axis>
      </joint>

      <joint type="revolute" name="right_wheel_hinge">
        <pose>0 0 0.03 0 0 0</pose>
        <child>right_wheel</child>
        <parent>chassis</parent>
        <axis>
          <xyz>0 1 0</xyz>
        </axis>
      </joint>
```

The two joints rotate about the y axis <xyz>0 1 0</xyz>, and connect each wheel to the chassis.

8. Try out on gazebo.
9. A new window should appear that contains various controllers for each joint. 
10. Under the Force tab, increase the force applied to each joint to about 0.1Nm.

## Import Meshes

### Prepare the Mesh

Gazebo uses a right-hand coordinate system.

Reduce Complexity

Center the mesh

Scale the mesh

### Export the Mesh

Once the mesh has been properly, export it as a Collada file. This format will contain all the 3D information and the materials.

#### Test the Mesh

The easiest way to test a mesh is to create a simple world file [my_mesh.world](http://bitbucket.org/osrf/gazebo_tutorials/raw/default/import_mesh/files/my_mesh.world) that loads the mesh. Replace `my_mesh.dae` with the actual filename of the mesh.

```xml
<?xml version="1.0"?>
<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <model name="my_mesh">
      <pose>0 0 0  0 0 0</pose>
      <static>true</static>
      <link name="body">
        <visual name="visual">
          <geometry>
            <mesh><uri>file://my_mesh.dae</uri></mesh>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Attach Meshes

