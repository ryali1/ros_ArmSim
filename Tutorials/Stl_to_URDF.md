# Fusion360 to URDF
The process for converting CAD files to a format useful to ROS and Gazebo (URDF) can be done with an automatic converter such as the sw_urdf_exporter (http://wiki.ros.org/sw_urdf_exporter) which can convert solidworks parts files to URDF or manually with the process shown below.

## Step 1: Separate the 3D assembly into different components you can save as .stl files
Since a typical assembly contains various parts which can act as either links or joints in our URDF, it's useful to save an assembly as multiple different components depending on how you want to code your links and joints.

For a basic differential drive robot, you can save the chassis as an stl file and the wheels as separate stl files.

## Step 2: Create a "meshes" directory in the package in which you're describing the robot, and put all the components' stl files inside of this directory
## Step 3: Upload the stl file in the geometry definitions of the link/joint as shown below:
This is defining the "assem.stl" file found in the meshes folder in the lawnbot_description package.
```
  <link name="base_link">
    <visual>

      <origin xyz="0 0 0" rpy="0 0 0" />

      <geometry>
          <mesh scale="0.001 0.001 0.001" filename="package://lawnbot_description/meshes/assem.stl"/>
      </geometry>
    
    </visual>

    <!-- Base Link - Collision Properties -->
    <collision>

      <origin xyz="0 0 0" rpy="0 0 0" />

      <geometry>
        <mesh scale="0.001 0.001 0.001" filename="package://lawnbot_description/meshes/assem.stl"/> 
      </geometry>

    </collision>


 ```
 
 Make sure to include the mesh geometry inside both your visual AND collision property definitions. 
 
 ## Step 4: Find the Mass/Inertial Properties in Fusion 360 
 
Usually there's two moment of inertia's included with fusion 360: Moment of Inertia at Origin and Moment of Inertia around center of mass. For now, I've included values for moment of inertia around center of mass.

Mass values in kg, Inertial Values: kgm^2.

Include Mass/Inertial values as shown below:
```
    <!-- Base Link Inertial Properties-->
    <inertial>

      <mass value="5.61526"/>

      <inertia ixx="0.3542" ixy="0.0001185" ixz="0.000184" iyy="0.2256" iyz="0.04673" izz="0.3513"/>

    </inertial>  
```

## Step 5: Find origin values for parts in fusion 360 and include in collision and visual definitions
The origin values for the components in Fusion 360 are the same origin values that need to be included in the origin values found in the collision and visual definitions.

Following code shows an example of including this. 
```
    <collision>

      <origin xyz="-0.011413 0.242164 0.04440" rpy="0 0 -1.570795" />

      <geometry>
        <mesh scale="0.001 0.001 0.001" filename="package://lawnbot_description/meshes/motorframe.stl"/>
      </geometry>

    </collision>
```
## Step 6: General Notes
All of the numerical information in fusion 360 is in mm, while ROS/Gazebo takes values in meters, so make sure to convert accordingly when you include the origin information and mesh scale.

Mesh Scale should be .001 if converting from mm to meters and .0254 if converting from inches to meters.

Origin in Fusion 360: (211.45, 317.45, 33.21) -> Origin in URDF File: (.21145, .31745, .03321)

