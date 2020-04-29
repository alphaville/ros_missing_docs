# Nodes, Messages and Topics

We will create a package called `rocket_control` which contains a node called
`controller` that subscribes to the topic `sensor` and publishes its results to
a "private" topic called `controller/action`.

The first step is to create a new package in our catkin workspace.

```sh
cd ~/catkin_ws/src  # always create packages here
catkin_create_pkg rocket_control std_msgs roscpp
```

### After creation, customize `package.xml`

```xml
<?xml version="1.0"?>
<package format="2">
  <name>rocket_control</name>
  <version>0.0.0</version>
  <description>MPC package</description>
  <maintainer email="p.sopasakis@gmail.com">chung</maintainer>
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
