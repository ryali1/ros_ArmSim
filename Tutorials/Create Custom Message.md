# Create a Custom Message in your ROS Package

## Creating your .msg file

All messages in ROS are defined in a file with a *.msg* extension. Therefore, we can begin creating a custom message by creating a creating a file with the name of your message and with an extension of *.msg*:
```
<message_name>.msg
```
This file should be placed within the **msg directory** of the package that you are working in. The <message_name> should follow the ROS convention of starting with a capital letter and an underscore being the only optional symbol to include. 

A message file simply consists of a list of **field types** and **field names**: 

```
fieldtype1 fieldname1
fieldtype2 fieldname2
fieldtype3 fieldname3
...
```

The *field type* is essentially the data type of that specific field. ROS messages support only the following field types:

- int8, int16, int32, int64, uint
- float32, float64
- string
- time, duration (these are ROS specific types)
- ROS Header
- other defined ROS messages
- fixed or variable length arrays of any of the above types (ex: string[] or int[5]) 

An example of message file that represents complex numbers called Complex.msg would look like this:
```
float32 real
float32 imaginary
```

## Modifying Package.xml
Now that we have defined a brand new custom ROS message, we must tell Catkin how to build this new message. Remember, Catkin has the responsibility of taking your message definition and converting it to C++ and Python implementations so that your C++ or Python node can use it. 

The first thing we must do is make sure that your package.xml file contains the following dependencies declared:
```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</run_depend>
```
If your package.xml does not already have the two dependencies listed above, please add them. 

## Modifying CMakeLists.txt
Next, we will make some modifications to CMakeLists.txt. To build custom messages, Catkin needs to use the *message_generation* package. We can instruct Catkin to find it by adding *message_generation* to the find_package function in CMakeLists.txt:
```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
```

Then, we must tell Catkin that we intend to use the defined message at runtime. We can do this by adding *message_runtime* to the *CATKIN_DEPENDS* field in the *catkin_package* function of CMakeLists.txt:
```
catkin_package(
  CATKIN_DEPENDS message_runtime
)
```
Please note that catkin_package might have additional fields other than CATKIN_DEPENDS and CATKIN_DEPENDS might also have additional dependencies inline with it. Please **do not** delete those if they are already present. Simply add message_runtime next to the other dependencies next to CATKIN_DEPENDS.

Next, we have to tell Catkin the name of the message file that contains your newly created custom message. We do this by adding it as a field in the *add_message_files* function of CMakeLists.txt: 

```
add_message_files(
  FILES
  <your message file name>.msg #ex: Complex.msg
)
```

Please keep the FILES field in *add_message_files*. This is a required field and should already be there by default. If for some reason it is not, please add it. 

The last thing we need to do is make sure the *generate_messages* function in CMakeLists.txt is uncommented and looks like this:

```
generate_messages(
  DEPENDENCIES
  std_msgs
)

```

Please note that there is no mention of the package that you are working on or the custom message you just created. The generate_messages function should look exactly as above and contain only DEPENDENCIES and std_msgs. It should contain these two by default, just make sure the whole function is uncommented. 

## Running catkin_make

Now that Catkin has all the information it needs to build your new message type, it is time to actually build it! Open up your command line and go to the **root of your Catkin workspace (not the root of your package!)**. In the root of your catkin workspace, run:
```
catkin_make
```

This command will actually build the C++ and Python implementations of your message. 

## Confirm Message Build

We can confirm that Catkin successfully created the language specific implementations by going to the following path:
```
<name of your catkin workspace>/devel/lib/python2.7/dist-packages/<your package>/msg
```
In this directory you should see the name of your custom message with a .py extension or a .cpp extension or both depending on if your package utilizes C++/Python/Both. If you see your custom message here, then Catkin has successfully located and built your new custom message! 

We can also confirm your message creation in ROS by running the following command:
```
rosmsg list
```
This command will output a list of available ROS messages and you should see your newly created custom message in the list

