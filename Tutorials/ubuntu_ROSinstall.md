  
# Ubuntu

ROS Melodic is primarily targeted at the Ubuntu 18.04 release.
Ubuntu 18.04 Documentation: https://wiki.ubuntu.com/BionicBeaver/ReleaseNotes

## Ubuntu Installation Instructions
Follow the tutorials below with the Ubuntu 18.04 LTS (https://releases.ubuntu.com/18.04.4/)

1. Windows Dual Boot Tutorial: https://itsfoss.com/install-ubuntu-1404-dual-boot-mode-windows-8-81-uefi/
2. Mac Dual Boot Tutorial: https://www.maketecheasier.com/install-dual-boot-ubuntu-mac/

# ROS
The following installation instructions will be specific to ROS Melodic. Compared to the prior release of ROS Kinetic, Melodic has minimal differences in terms of ROS basics. ROS Kinetic may have more support for certain hardware configurations and packages.

Ros Melodic works best with Ubuntu 18.04. ROS Kinetic works best with 

Documentation on ROS Melodic: 

## ROS Melodic Installation Instructions
### Step 1: Configure your Ubuntu Repositories
Configure Ubuntu repositories to allow for "restricted", "universe", and "multiverse". 

Instructions for configuring repos: https://help.ubuntu.com/community/Repositories/Ubuntu

*** If you have just installed Ubuntu on your workstation, then this should be a default setting and own't require you to do anything ***

### Step 2: Setup your sources.list
Set up computer to accept software from packages.ros.org.

Open terminal and type:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
If your Ubuntu installation was successful and you have your repositories configured properly, then this should work automatically.

### Step 3: Setup Network Keys
Connect to the Ubuntu Key server before beginning ROS Installation

In terminal type:
```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

If there any difficulties connecting to the keyserver, substitute above code with:
``` 
hkp://pgp.mit.edu:80 or hkp://keyserver.ubuntu.com:80
```
or if your network is behind a proxy server type:
```
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
```

### Step 4: ROS Installation
First, ensure Debian packages indexes are up-to-date:

```
sudo apt update
```

There are several installation options for ROS.

1. Desktop-Full Install (recommended installation): Includes ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators, and 2D/3D perception

```
sudo apt install ros-melodic-desktop-full
```

2. Desktop Install: ROS, rqt, rviz, and robot-generic libraries
```
sudo apt install ros-melodic-desktop
```
3. ROS-Base: (Bare Bones) ROS Package, build, and communication libraries. No GUI Tools.
```
sudo apt install ros-melodic-ros-base
```
4. Individual Packages: It's possible to individually install specific ROS packages
```
sudo apt install ros-melodic-PACKAGE
```

To find out available packages for download:
```
apt search ros-melodic
```

### Step 5: Setting up ROS Environments
It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched:
```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

If you just want to change environment of current shell, type:
```
source /opt/ros/melodic/setup.bash
```

### Step 6: Setting up Dependencies to Begin Building ROS Packages
Up till now, we have installed what we need to run core ROS packages. If we want to create and manage our own ROS workspaces, we require a variety of tools and requirements that are distributed separately. 

One frequently used command-line tool for downloading many source trees for ROS packages is "rosinstall". Install "rosinstall" and other similar dependencies that help with building ROS packages:
```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### Step 6.1 Initialize rosdep
rosdep is a command line tool for installing ROS dependencies. This is also required to run some core ROS components. Run:
```
sudo apt install python-rosdep
```
with the following you can initiliaze rosdep
```
sudo rosdep init
rosdep update
```
## Testing ROS Installation with a simple Gazebo Tutorial
For this tutorial, we'll be running a simple Gazebo simulation to ensure our ROS installation was successful. For sake of simplicity and testing, details on the structure of the code and fetched repository will be explained in subsequent documentation.

**1. Clone Repository for Tutorial on Github**
```
git clone -b base https://github.com/richardw05/mybot_ws.git
```
Cloning this repo will create a new folder called: "mybot_ws"

**2. Build a catkin workspace in mybot_ws**
```
cd mybot_ws/
ls 
catkin build
```
**3. Add the workspace to search path before launching Gazebo (ROS will not launch without sourcing it this way)**
```
echo "source ~/mybot_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
**4.Launch the tutorial's URDF model in a empty Gazebo World**
```
roslaunch mybot_gazebo mybot_world.launch
```
This should open up a model of the car in Gazebo

**5. Publish a Command to the Car to get it to start moving"**

Open a new tab in terminal and type the following command.
```
rostopic pub cmd_vel geometry_msgs/Twist " <TAB>
```
Press tab after the " and the rest of the command should appear like this:
```
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```
The 0.0's can be edited based on how you want to move the car. An example command is:
```
linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.1
  y: 0.0
  z: 0.0"
```
This is get the car to move in a circle. Different numbers will lead to different trajectories for the car



