# Nodes, Messages and Topics

**Problem statement.** Suppose we need to make a package that offers autonomous
driving functionality to a driver-less car. Let us call our ROS package `navigator`.
In our package we need to implement a `pilot` node. The node will subscribe to a
topic to listen for position and pose information, positions, poses and velocities of other
vehicles and other relevant information. The node expects that this information is posted
(by some other node) at a topic called `sensors`. The node will do its magic,
compute certain control actions (e.g., steering angle, accelerator position, gear, etc)
and publish its output at a topic called `commands`.


### Create package

The first step is to create a new package in our catkin workspace.

```sh
cd ~/catkin_ws/src  # always create packages here
catkin_create_pkg navigator std_msgs roscpp
```

This will generate the folder `~/catkin_ws/src/navigator`. Our package
depends on `std_msgs` and `roscpp`. We can add more dependencies later.


### After creation, customize `package.xml`

```xml
<?xml version="1.0"?>
<package format="2">
  <name>navigator</name>
  <version>0.0.0</version>
  <description>Autonomous vehicle navigator</description>
  <maintainer email="p.sopasakis@example.com">Pantelis Sopasakis</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <export></export>
</package>
```

### Define message templates (msg files)

We need to define the form of the messages associated with our node. The message
templates are simple text files with the extension `.msg` that need to be in
the directory `~/catkin_ws/src/navigator/msg`.

We define the message `SensorData` as follows:

```
# Contents of file ~/catkin_ws/src/navigator/msg/SensorData.msg
float64[]    gps_data            # you can write comments like that
                                 # gps_data is an array of float64,
                                 # without a specified length
float64[2]   vehicle_velocity    # vehicle_velocity is an array of float64
                                 # of length 2
bool         is_intersection     # whether we are at an intersection; this
                                 # a Boolean variable
uint8        status_code         # Some status code (unsigned integer)
```

There are better ways of structuring our message, but this is just an example.

Likewise, we define the message `DriverCommand` as follows

```
# Contents of file ~/catkin_ws/src/navigator/msg/DriverCommand.msg
float64    acceleration_set_point
int8       gear
float64    breaks
bool[3]    indicator_lights    
```

The reason we defined the above messages is that ROS will generate a C++ interface
that will allow us to:

- Read messages from the topic `sensors` and cast them directly as `SensorData`.
  ROS will parse the messages on `sensors` and return an **object** of type
  `SensorData`
- Publish messages of type `DriverCommand` directly to the topic `command`.
  We will not need to serialize the message - ROS will do that automatically.

To that end, we need to ask ROS to generate the necessary C++ interface. For
that we need to enable message generation by modifying two files.

### Enable message generation

First, we need to modify `package.xml` and add the following:


```xml
<!-- Messages -->
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

Then edit `CMakeLists.txt` and add `message_generation` to `find_package`, that is:

```.cmake
find_package(catkin REQUIRED COMPONENTS
 roscpp
 std_msgs
 message_generation
)
```

Additionally, add `message_runtime` to `catkin_package`

```.cmake
catkin_package(
...
CATKIN_DEPENDS message_runtime ...
...)
```

Thirdly, define the message:

```.cmake
add_message_files(
FILES
SensorData.msg
DriverCommand.msg
)
```

The last action is to uncomment the following:

```.cmake
generate_messages(
DEPENDENCIES
std_msgs
)
```

### Compile

Although the package is not ready year, let us compile what we have so far.
Do:

```sh
cd ~/catkin_ws
catkin_make
```
Have a look at `~/catkin_ws/devel/include/navigator`. You will find two header
files:

- `DriverCommand.h` and
- `SensorData.h`

These will come in handy in the next section where we will make our node.


### Create a node

We will create a node at `src/pilot.cpp`. But first we need to include this file
into `CMakeLists.txt`. Edit `~/catkin_make/src/navigator/CMakeLists.txt` and
add the following at the end of the file:

```.cmake
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pilot src/pilot.cpp)
target_link_libraries(pilot ${catkin_LIBRARIES})
add_dependencies(pilot ${PROJECT_NAME}_generate_messages_cpp)
```

Then, create the file `~/catkin_make/src/navigator/src/pilot.cpp` with the
following content:

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pilot");
  ros::NodeHandle private_nh_("~");
  ros::Publisher chatter_pub = private_nh_.advertise<std_msgs::String>("commands", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}
```
