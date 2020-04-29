<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" async src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>

# Nodes, Messages and Topics

**Problem statement.** Suppose we need to make a package that offers autonomous
driving functionality to a driver-less car. Let us call our ROS package `navigator`.
In our package we need to implement a `pilot` node. The node will subscribe to a
topic to listen for position and pose information, positions, poses and velocities of other
vehicles and other relevant information. The node expects that this information is posted
(by some other node) at a topic called `sensors`. The node will do its magic,
compute certain control actions (e.g., steering angle, accelerator position, gear, etc)
and publish its output at a topic called `commands`.


## Create package

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

## Define message templates (msg files)

### Create msg files
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
float64    acceleration_set_point   # acceleration set point (0 to 100)
float64    steering_wheel_angle     # steering wheel angle in rad
int8       gear                     # gear (0 to 5 and -1 for reverse)
float64    brakes                   # brakes (0 to 100)
bool[3]    indicator_lights         # 0: left, 1: right, 2: both
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
   CATKIN_DEPENDS message_runtime
)
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


## Create a node

### CMakeLists

We will create a node at `src/pilot.cpp`. But first we need to include this file
into `CMakeLists.txt`. Edit `~/catkin_make/src/navigator/CMakeLists.txt` and
add the following at the end of the file:

```.cmake
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pilot src/pilot.cpp)
target_link_libraries(pilot ${catkin_LIBRARIES})
add_dependencies(pilot ${PROJECT_NAME}_generate_messages_cpp)
```

### The Publisher

We start by creating the file `~/catkin_make/src/navigator/src/pilot.cpp`.
We will create a node that posts messages of type `DriverCommand` on the
"private" topic `navigator/commands`. The term "private" does not mean that
the topic is not accessible to other nodes. It rather means that it is a topic
associated with the current package.

```.cpp
#include "ros/ros.h"                /* always include ros/ros.h */
#include "navigator/SensorData.h"   /* include message classes  */
#include "navigator/DriverCommand.h"

int main(int argc, char **argv) {
  /**
   * Create an instance of DriverCommand. Note that DriverCommand belongs to
   * the namespace "navigator" (same name as the package).
   */
  navigator::DriverCommand driver_command;
  driver_command.acceleration_set_point = 5.0;
  driver_command.gear = 3;
  driver_command.brakes = 0.5;
  driver_command.indicator_lights[0] = 1;

  /**
   * Initialise the ROS node. The third argument is the name of this node.
   */
  ros::init(argc, argv, "pilot");
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * Here we create a private node handle
   */
  ros::NodeHandle private_nh_("~");
  /**
   * Create a publisher for objects of type DriverCommand.
   */
  ros::Publisher commands_pub
    = private_nh_.advertise<navigator::DriverCommand>("commands", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok()) {  
     /**
      * This shows how easy it easy to publish an object to a topic
      * We don't have to worry about serialization
      */
    commands_pub.publish(driver_command);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
```

Let us now compile it and use it.

Compile with:

```sh
cd ~/catkin_ws/
catkin_make
```

To run the node we just compiled, we first need to run `roscore` and then use
`rosrun`. You can do so as follows: first run

```sh
roscore
```

Then, in a different terminal, run

```sh
rosrun navigator pilot
```

In another terminal we can obtain a list of all active topics:

```sh
rostopic list
```

This should print:

```txt
/pilot/commands
/rosout
/rosout_agg
```

Indeed, `/pilot/commands` is listed as an active topic. We can connect to it and
print all messages that are announced on it:

```sh
rostopic echo /pilot/commands
```

You should see the following messages getting printed at a rate of 10Hz:

```
---
acceleration_set_point: 5.0
steering_wheel_angle: 0.0
gear: 3
brakes: 0.5
indicator_lights: [True, False, False]
---
acceleration_set_point: 5.0
steering_wheel_angle: 0.0
gear: 3
brakes: 0.5
indicator_lights: [True, False, False]
```

Hit Ctrl+C to exit. If you want to get just one of these values, say `gear`,
you can do

```sh
rostopic echo /pilot/commands/gear
```


### Publisher + Subscriber: Implementation of a controller

Let's do something more interesting... we'll subscribe to topic `pilot/sensors`,
read the announced messages (of type `SensorData`) and we will compute an
acceleration set point (`acceleration_set_point`) value according to

<div class="math">
\[\begin{split}\alpha = \begin{cases}30(v_1+v_2-10),&\text{ if at intersection}
\\
5(v_1-v_2-1),&\text{ otherwise}
\end{cases}\end{split}\]</div>


```.cpp
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "navigator/SensorData.h"
#include "navigator/DriverCommand.h"


/**
 * Static variable to store data obtained from
 * topic: `pilot/sensors`
 */
static navigator::SensorData sensorsMessage;

/**
 * Callback function to obtain data from topic
 * `pilot/sensors`. It takes one line of code to
 * parse a SensorData-type message.
 */
void receiveSensorDataCallback(
        const navigator::SensorData::ConstPtr& msg) {       
        sensorsMessage = *msg;
}

int main(int argc, char **argv)
{
  navigator::DriverCommand driver_command;
  driver_command.acceleration_set_point = 5.0;
  driver_command.gear = 3;
  driver_command.brakes = 0.5;
  driver_command.indicator_lights[0] = 1;

  ros::init(argc, argv, "pilot");
  ros::NodeHandle private_nh_("~");
  ros::Publisher commands_pub
     = private_nh_.advertise<navigator::DriverCommand>("commands", 1000);

  /**
   * Create a subscriber to the topic `~/sensors` (i.e., pilot/sensors)
   * and assign to it a callback function
   */
  ros::Subscriber sub
    = private_nh_.subscribe("sensors", 1000, receiveSensorDataCallback);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    double v = sensorsMessage.vehicle_velocity[0];
    ROS_INFO("status = %d", sensorsMessage.status_code);
    ROS_INFO("v = (%g, %g)", sensorsMessage.vehicle_velocity[0], sensorsMessage.vehicle_velocity[1]);
    commands_pub.publish(driver_command);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
```
