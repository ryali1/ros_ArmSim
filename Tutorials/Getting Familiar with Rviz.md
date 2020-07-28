  
# Rendering and Visualizing a URDF Model in rViz and Gazebo
This tutorial shows how to create a customizable URDF Model and visualize it in rViz and Gazebo. The URDF Model that is rendered will be a simple, two-wheeled robot model.

** Note ** 
If copy pasting the code from this tutorial leads to errors, then you can copy paste directly from the created files found within this repo.

## Step 1: Create a Catkin Workspace
A catkin workspace is a directory for creating and modifying catkin packages. Initializing directories as catkin workspaces simplifies the build and installation processes for ROS packages.

Below code creates a catkin workspace in a directory titled /robot_sim

```
source /opt/ros/melodic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
If catkin_make doesn't work, then you may not have installed catkin or have the necessary dependecies.

To install catkin:
```
sudo apt-get install ros-melodic-catkin
```

## Step 2: Create directory for simulations
Within the new catkin directory, create a ros package for simulations and build the packages in the catkin workspace.

```
cd ~catkin_ws/src
catkin_create_pkg simulations

cd ~/catkin_ws
catkin_make

```
## Step 3: Create a directory for storing URDF Files
Within the newly created simulations directory, create a new directory for storing URDF Files
```
cd ~/catkin_ws/src/simulations
mkdir urdf
cd urdf
```

## Step 4:Using Turtle Sim.

Turtle Sim is a default ROS package that will help us understand some of the fundementals of ROS.

Start by opening terminal and running
```
$ roscore

```
And leave this window open.

In another terminal tab run 
```

$ rosrun turtlesim turtlesim_node

```
This should have started your first ROS program.

## Step 5: Getting familiar with ROS and its commands

Now that Turtle Sim is open, put this command into the terminal

```
$ rosnode list
```

You should see 

/rosout
/turtlesim


Now put this into the terminal

```
$ rosnode info /turtlesim
```
This shows you all of the publications and subscriptions that are assigned with turtle sim.

Using the command
```
$ rostopic list 
```
We should see a list of topics that are associated with turtle sim. This is what we can use to understand how ROS communicates with other aspects of itself

## Step 7:


## Step 4: Understanding the Main Component of URDF Files
Two basic URDF components are used to describe robot models: Links and Joints. Link components are used to describe the physical properties (dimensions, origin position, color, etc) of a rigid body. Links are connected together by joints, which describe kinematic and dynamic properties for the connection (which links are connected to each other, the types of joint, axes o ration, amount of friction, damping, etc).

The URDF file contains information regarding a set of link elements, and a set joint elements that connect the links together.

## Step 5: Creating a robot chassis
The first component of the robot we're building is a chassis box. This can be thought of as a "base" link from which all subsequent links and joints will stem from.

In a text editor type the following code and save as a .urdf file in /robot_sim/src/simulations/urdf

For the purposes of this tutorial, title this file: "test_robot1.urdf"
```
<?xml version='1.0'?>
<robot name="test_robot1">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
          <box size="0.5 0.5 0.25"/>
      </geometry>
    </visual>
  </link>

</robot>
```
First two lines: Defines a robot named test_robot. 

The base link is the first component defined in the code. This creates the beginning of the robot's kinematic chain. The name of the link is "base_link". 

The base_link's initial position (origin) is at (x,y,z)=(0,0,0) and  rpy (roll, pitch, yaw) axes= (0,0,0).

Visually, the base link is represented as a box with dimensions: 0.5 meters length, 0.5 meters width, and 0.25 meters height.

## Step 6: Create Launch File to Visualize URDF File Information
roslaunch is a tool in ROS that simplifies the process of launching ROS nodes and setting parameters on the ROS Parameter Server.

Create a "launch" directory under the simulations package:
``` 
cd ~/catkin_ws/src/simulations
mkdir launch
cd launch
```
Within this new directory, create a .launch file to launch the created robot model in rViz.

For the purposes of this tutorial, title your created launch file: "testrobot_rviz.launch". 
Code for Launch File:
```
<launch>
   <!-- values passed by command line input -->     
   <arg name="model" />
   <arg name="gui" default="False" />

   <!-- set these parameters on Parameter Server -->
   <param name="robot_description" textfile="$(find simulations)/urdf/$(arg model)" />
   <param name="use_gui" value="$(arg gui)"/>

   <!-- Start 3 nodes: joint_state_publisher, robot_state_publisher and rviz -->
   <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />

   <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />

   <node name="rviz" pkg="rviz" type="rviz" args="-d $(find simulations)/urdf.rviz" required="true" />
   <!-- (required = "true") if rviz dies, entire roslaunch will be killed -->
</launch>
```
This launch file begins with finding the urdf file we created under the simulations directory. Next, it calls three Nodes for launching and controlling the robot in rViz: Joint_state_publisher, robot_state_publisher, and rViz

Brief descriptions of the node and shown below. We'll be exploring how to use the nodes later on in the tutorial.

### Joint_State_Publisher Node
This package publishes sensor_msgs/JointState messages for a robot. The package reads the robot_description parameter from the parameter server, finds all of the non-fixed joints and publishes a JointState message with all the defined joints.

Documentation on this Node: wiki.ros.org/joint_state_publisher

### Robot_State_Publisher Node
This nodes allows you to publish the state of the robot to the tf package. Once it gets published, it can be used with all components in the system that us tf. 

Documentation on this node: wiki.ros.org/robot_state_publisher

### rViz Node
rViz is essentially a 3D visualization tool for ROS.

Documentation: wiki.ros.org/rviz

## Step 7: SOURCE Bash Files 
Before visualizing the file and using roslaunch, make sure your files are sourced properly.
```
echo "source ~/robot_sim/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

```

## Step 8: Visualize File in rViz
Enter below code to open the URDF file in rViz:

```
roslaunch simulations testrobot_rviz.launch model:=test_robot1.urdf
```
At this stage, the rViz application should open up, however, you will still not be able to view the model.

To visualize the model, press ADD in the bottom left of the display section and select RobotModel and TF. You will need to add each of these separately (can't do it by just pressing add once). Next, under Global Options, change the Fixed Frame value from "map" to "base_link".

This should display a red chassis in the middle of your rViz workspace, and any other URDF files you are adding.

** Important Things to Note **
- The fixed frame is a transform frame where the origin (center) of the grid is located
- In the URDF Model, <origin> defines the reference frame of the visual element with respect to the reference frame of the link. In this model, the visual element (box) has an origin at the center of its geometry (half of the box is above the grid plane and half is below)

## Step 9: Adding Wheels to URDF Model

As mentioned before, whenever we add new link elements to the URDF file, we shouold also add joint elements to describe the relationships between the links. Joint elements are necessary to define the flexibility/inflexibility of the joints. There are 6 types of joints that can be defined in the URDF model.

### 6 Types of Joints
1. Fixed: This type of "joint" is rigid and all degrees of freedom are locked. This joint type doesn't require axis, calibration, dynamics, limits, or safety controllers.
2. Revolute: This joint rotatoes around a single axis and has a predefined range (upper and lower limits)
3. Continuous: This can be thought of as a revolute joint with no upper and lower limits on its range of motion
4. Prismatic: Sliding joint that slides along an acis and has a predefined range, similar to a revolute joint
5. Floating: Allows for motion in 6 degrees of friend
6. Planar: Allows for motion in a plane perpendicular to a certain axis.

The code below shows how to add wheels to the chassis.

```
<?xml version='1.0'?>
<robot name="test_robot2">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
          <box size="0.5 0.5 0.25"/>
      </geometry>
    </visual>
  </link>

  <!-- Right Wheel -->
  <link name="right_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
    </visual>
  </link>
  <joint name="joint_right_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.30 0" rpy="0 0 0" /> 
    <axis xyz="0 1 0" />
  </joint>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
    </visual>
  </link>
  <joint name="joint_left_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.30 0" rpy="0 0 0" /> 
    <axis xyz="0 1 0" />
  </joint>

</robot>
```
**Things to Note:
1.** Both of the right and left wheel links have assosicated joint descriptions provided after the link description. 
**2.** Each wheel is visually defined as a cylinder with a 0.2 meter radius and 0.1 meter length. The wheel's origin defines the location for the center of the wheel element. Each wheel's origin is defined at (0,0,0) and is rotated 90 degrees (or 1.5670795 radians) around the x axis
**3.** The demonstrated tree structure is defined in terms of a parent and a child with a singular root link (base_link in this case). The position of the wheels is determined in reference to this base_link
**4.** The joints for the wheels are determined in terms of the parent's reference frame. In this case, the left wheel's joint origin is 0.30 meters in the x direction and the right wheel's joint origin is -0.30 meters in the x direction. 
**5.** The axis of rotation is specified after the <axis ...> and in this case the wheel's joint axis of rotation is around the y  axis, therefore (0 1 0).
  
**6.** The joint elements define the complete kinematic model of the robot

## Step 10: Adding a caster
Now, we will be adding a caster element to the front of the robot to balance the chassis. This caster only serves as a visual element, and will not be defined as a joint. Visually, it will slide along the ground plane as the wheels of the robot move.

The Code below adds a caster element before the wheel definitions:
```
<?xml version='1.0'?>
<robot name="test_robot3">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
          <box size="0.5 0.5 0.25"/>
      </geometry>
    </visual>


    <!-- Caster -->
    <visual name="caster">
        <origin xyz="0.2 0 -0.125" rpy="0 0 0" />
        <geometry>
            <sphere radius="0.05" />
        </geometry>
    </visual>
  </link>

  <!-- Right Wheel -->
  <link name="right_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
    </visual>
  </link>
  <joint name="joint_right_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.30 0" rpy="0 0 0" /> 
    <axis xyz="0 1 0" />
  </joint>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
    </visual>
  </link>
  <joint name="joint_left_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.30 0" rpy="0 0 0" /> 
    <axis xyz="0 1 0" />
  </joint>

</robot>
```
**Important Things to Note**
**1.** The caster is visually defined as a sphere with radius 0.05 meters. The center of the caster is at 0.2 meters in the x direction and -0.125 meters in the z direction with respect to the origin of the base_link

## Step 11: Adding Color to the Model
Currently, our robot is completely red with no distinctive parts. The code below changes the color of the chassis to blue and the wheels to red.
```
<?xml version='1.0'?>
<robot name="test_robot4">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
          <box size="0.5 0.5 0.25"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0.5 1 1" />
      </material>
    </visual>


    <!-- Caster -->
    <visual name="caster">
        <origin xyz="0.2 0 -0.125" rpy="0 0 0" />
        <geometry>
            <sphere radius="0.05" />
        </geometry>
    </visual>
  </link>

  <!-- Right Wheel -->
  <link name="right_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
      <material name= "black">
        <color rgba="0.05 0.05 0.05 1" />
      </material>
    </visual>
  </link>
  <joint name="joint_right_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.30 0" rpy="0 0 0" /> 
    <axis xyz="0 1 0" />
  </joint>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
      <material name="black"/>
    </visual>
  </link>
  <joint name="joint_left_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.30 0" rpy="0 0 0" /> 
    <axis xyz="0 1 0" />
  </joint>

</robot>
```
**Important Things to Note**
**1.** Once a color is defined as a material name, that material name can just be listed in the code instead of redefining the properties of the color -> Look at how the black color is defined in the L and R wheels
**2.** The <material> tag can define <color> in terms of red, green, blue, and alpha, with each being in the range [0,1]. Alpha represents the transparency of the color with 1 being opague and 0 being transparent. Once specified and labeled with a name, the material can be reused without specifically applying color values
  
## Step 12: Adding Collisions
Even though we've defined all of our visual elements, we need to add a collision property that identifies the boundary of the robot for Gazebo's collision detection engine. Without this, Gazebo can't recognize the boundaries of a robot. The collision property must be defined for each visual element:
```
<?xml version='1.0'?>
<robot name="test_robot5">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
          <box size="0.5 0.5 0.25"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0.5 1 1" />
      </material>
    </visual>
    <!-- Base collision -->
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
          <box size="0.5 0.5 0.25"/>
      </geometry>
    </collision>    


    <!-- Caster -->
    <visual name="caster">
        <origin xyz="0.2 0 -0.125" rpy="0 0 0" />
        <geometry>
            <sphere radius="0.05" />
        </geometry>
    </visual>
    <!-- Caster collision -->
    <collision>
      <origin xyz="0.2 0 -0.125" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.05" />
      </geometry>
    </collision>    
  </link>

  <!-- Right Wheel -->
  <link name="right_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
      <material name= "black">
        <color rgba="0.05 0.05 0.05 1" />
      </material>
    </visual>
    <!-- Right Wheel collision -->
    <collision>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
    </collision>
  </link>
  <joint name="joint_right_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.30 0" rpy="0 0 0" /> 
    <axis xyz="0 1 0" />
  </joint>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
      <material name="black"/>
    </visual>
    <!-- Left Wheel collision -->
    <collision>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
    </collision>
  </link>
  <joint name="joint_left_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.30 0" rpy="0 0 0" /> 
    <axis xyz="0 1 0" />
  </joint>

</robot>




```

## Step 13: Moving the Robot Wheels in rViz (This doesn't work for some reason, need to look into why)
Once we have our right and left wheel joint definitions, it possible to use the joint_state_publisher and robot_state_publisher nodes defined in our launch file to control the movement of the robot.

The joint_state_publisher node, as defined before, finds all of the non-fixed joints and publishes a JointState message with all those joints defined. We can bring up a GUI interface to change the values of the JointState message to see the robot move. Currently, the JointState message values are constant.

```
roslaunch simulations testrobot_rviz.launch model:=test_robot5.urdf gui:=True
```


## Step 14: Adding Physical Properties to Robot
Before the robot can be launched in Gazebo, we need to specify physical properties such as mass and inertia. These properties are required by Gazebo's physics engine. Specifically, every simulated <link> elements needs an <inertial> tag.

The two elements of the inertial element are:
- <mass> : this is the weight defined in kilograms
- <inertia>: This is a 3x3 rotational inertia matrix.The matrix structure is: [ixx, ixy, ixz; ixy, iyy, iyz; ixz, iyz, and izz]
Since the inertia matrix is symmetric, only 6 of the values are needed.

en.wikipedia.org/wiki/List_of_moments_of_inertia provides us with numerous inertial matrices for a number of common shapes such as cylinders, boxes, spheres, etc

The following code contains the inertial matrices for each of the inertial matrices:
```
<?xml version='1.0'?>
<robot name="test_robot6">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
          <box size="0.5 0.5 0.25"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0.5 1 1"/>
      </material>
    </visual>
    <!-- Base collision, mass and inertia -->
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
          <box size="0.5 0.5 0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <inertia ixx="0.13" ixy="0.0" ixz="0.0" iyy="0.21" iyz="0.0" izz="0.13"/>
    </inertial>

    <!-- Caster -->
    <visual name="caster">
      <origin xyz="0.2 0 -0.125" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.05" />
      </geometry>
    </visual>
    <!-- Caster collision, mass and inertia -->
    <collision>
      <origin xyz="0.2 0 -0.125" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.05" />
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>

  </link>

  <!-- Right Wheel -->
  <link name="right_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
      <material name="darkgray">
        <color rgba=".2 .2 .2 1"/>
      </material>
    </visual>
    <!-- Right Wheel collision, mass and inertia -->
    <collision>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>

  </link>

  <!-- Right Wheel joint -->
  <joint name="joint_right_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.30 0" rpy="0 0 0" /> 
    <axis xyz="0 1 0" />
  </joint>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
      <material name="darkgray">
        <color rgba=".2 .2 .2 1"/>
      </material>
    </visual>
    <!-- Left Wheel collision, mass and inertia -->
    <collision>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Left Wheel joint -->
  <joint name="joint_left_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.30 0" rpy="0 0 0" /> 
    <axis xyz="0 1 0" />
  </joint>

</robot>
```
Adding inertial properties doesn't change what happens in rviz.

## Step 15: Exploring our URDF Model with basic command line tools (THIS is ALSO NOT WORKING)
ROS has command-line tools to help verify and visualize information from URDF models. 

First, make sure you have the necessary tools installed:
```
sudo apt-get install liburdfdom-tools
```

### check_urdf
check_urdf parses URDF file description and prints a description of the defined kinematic chain

```
check_urdf test_robot6.urdf
```

### urdf_to_graphiz
this tool creates a graphviz diagram of a URDF file and a diagram in .pdf format.
```
urdf_to_graphiz test_robot6.urdf
```
## Step 16: Preparing the URDF File for Gazebo
Gazebo expects the robot model to be in SDF format. SDF is similar to URDF and Gazebo will automatically convert URDF models, however, it requires certain descriptive tags within the URDF file code for a successful conversion.

The <gazebo> tag is added to the URDF model to specify additional elements needed for identifying elements found in the SDF format. If a <gazebo> tag is used independent of the reference= property, then it is assumed that the gazebo element refers to the whole robot model. Other <gazebo> elements may be added to links and joints to specify the physics, however, for our curernt purposes we will stick to just specifying color.
  
  The way that color is specified for rViz can't be identified for gazebo, so it requires a <gazebo> specification. The code below incorporates the appropriate gazebo tags to specify the colors for the base_link, right_wheel, and left_wheel.

```
<?xml version='1.0'?>
<robot name="testrobot">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
          <box size="0.5 0.5 0.25"/>
      </geometry>
    </visual>
    <!-- Base collision, mass and inertia -->
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
          <box size="0.5 0.5 0.25"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <inertia ixx="0.13" ixy="0.0" ixz="0.0" iyy="0.21" iyz="0.0" izz="0.13"/>
    </inertial>

    <!-- Caster -->
    <visual name="caster">
      <origin xyz="0.2 0 -0.125" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.05" />
      </geometry>
    </visual>
    <!-- Caster collision, mass and inertia -->
    <collision>
      <origin xyz="0.2 0 -0.125" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.05" />
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>

  </link>

  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <pose>0 0 3 0 0 0</pose>
  </gazebo>

  <!-- Right Wheel -->
  <link name="right_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
    </visual>
    <!-- Right Wheel collision, mass and inertia -->
    <collision>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>

  </link>

  <gazebo reference="right_wheel">
    <material>Gazebo/Black</material>
  </gazebo>

  <!-- Right Wheel joint -->
  <joint name="joint_right_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.30 0.025" rpy="0 0 0" /> 
    <axis xyz="0 1 0" />
  </joint>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
    </visual>
    <!-- Left Wheel collision, mass and inertia -->
    <collision>
      <origin xyz="0 0 0" rpy="1.570795 0 0" />
      <geometry>
          <cylinder length="0.1" radius="0.2" />
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>

  </link>

  <gazebo reference="left_wheel">
    <material>Gazebo/Black</material>
  </gazebo>

  <!-- Left Wheel joint -->
  <joint name="joint_left_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.30 0.025" rpy="0 0 0" /> 
    <axis xyz="0 1 0"/>
  </joint>

</robot>
```
^ Save the above code in a file titled: "test_robot.gazebo" under the URDF folder in simulations.

## Step 17: Creating the simplest world file for Gazebo
We now need to create a simple world file to launch our test robot into. To begin, create a new directory under simulations/src titled "worlds"

``` 
cd .../simulations
mkdir worlds
```
Within this directory, create a file titled testrobot.world and include the following code:
```
<?xml version="1.0" ?>
<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
  </world>
</sdf>
```

This file is basically creating a world with a name: "default" and is including a ground_plane model which specifies for (as you might guess) a ground plane. When this world is launched in gazebo you will just see a grid under your robot model.  If you want to add some more light you can add a sun model:
```
<?xml version="1.0" ?>
<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```




## Step 17: Creating a Launch File for Visualizing URDF in Gazebo
Create a new file titled, "testrobot_gazebo.launch" in the launch folder. This launch file is essentially calling the URDF file we have created and launching it into the customized world file we have created.

```
<launch>
  <!-- We resume the logic in gazebo_ros package empty_world.launch, -->
  <!-- changing only the name of the world to be launched -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find simulations)/worlds/testrobot.world"/>
   
    <arg name="paused" default="false"/>
    <arg name="use_sim_time" default="true"/>
    <arg name="gui" default="true"/>
    <arg name="headless" default="false"/>
    <arg name="debug" default="false"/>

  </include>

  <!-- Spawn dd_robot into Gazebo -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" output="screen"
     args="-file $(find simulations)/urdf/test_robot.gazebo -urdf -model testrobot" />

</launch>

```
The above code is finding the testrobot.world file within the simulations/worlds directory as well as the test_robot.gazebo file within the simulations/urdf directory.

## Step 18: Launch Test Robot Model in Gazebo
Following code should launch our robot model in Gazebo:
```
roslaunch simulations testrobot_gazebo.launch
```



















