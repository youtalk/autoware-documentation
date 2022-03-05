# ROS nodes coding guidelines

!!! warning

    Under Construction

### Naming in Autoware.Auto

The [Naming Guidelines](@ref autoware-common-naming-guidelines) provide for standard, reliable naming and namespacing conventions which should be used in all Autoware.Auto packages.

### On Topics and Parameters

In most cases, topics should receive a default name in code and be remapped if needed. Providing topic names as ROS parameters is an anti-pattern, with few exceptions.

Required parameters should not have default values but fail during construction if no value is provided.

#### Parameter File Syntax

To avoid the need to change parameter files based on the namespacing or node name of a node, use the "double-star" syntax. e.g.:

```{yaml}
/**:
  ros__parameters:
    param1: value
```

The above parameter file can be passed to any node regardless of namespace or name and the parameters will populate those of the node if the declared parameters match those in the file.

### ROS Components {#contributors-guidelines-components}

As of ROS Dashing, the recommended way to write Nodes in ROS 2 is using Components.
For more information about components and their use, see [the ROS Composition Guide](https://index.ros.org/doc/ros2/Tutorials/Composition/).
To implement your node as a Component, it must conform to the items below (using `ListenerNode` as an example):

- Must inherit from `rclcpp::Node` or a subclass (such as `rclcpp::LifecycleNode`)
- Must use a single-argument constructor in the form of:

```{cpp}
namespace composition_example
{
class ListenerNode: public rclcpp::Node {
  ListenerNode(const rclcpp::NodeOptions & options)
  : Node("listener", options)
  {
    ...
  }
}
}  // namespace composition_example
```

- Must contain a registration macro and header in a single translation unit. For example, the following at the bottom of your `cpp` file would suffice:

```{cpp}
// Insert at bottom of translation unit, e.g. listener_node.cpp
#include <rclcpp_components/register_node_macro.hpp>
// Use fully-qualified name in registration
RCLCPP_COMPONENTS_REGISTER_NODE(composition_example::ListenerNode)
```

- Must compile the components as a shared library and register them in your `CMakeLists.txt` file.
- Must depend on the `rclcpp_components` package.

#### Minimal CMake Example {#contributors-guidelines-minimal-cmake-example}

The following is a minimal `CMakeLists.txt` file which uses the recommended `ament_cmake_auto` macros, registers a single component, builds a stand-alone node which uses the component, and exports it as a dependency for downstream packages. It can be conveniently created by [`autoware_auto_create_pkg`](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/autoware_auto_create_pkg-package-design.html):

```{cmake}
cmake_minimum_required(VERSION 3.5)
project(composition_example)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(listener_node SHARED src/listener_node.cpp)
autoware_set_compile_options(listener_node)
rclcpp_components_register_nodes(listener_node "composition_example::ListenerNode")

ament_auto_add_executable(listener_node_exe src/listener_main.cpp)
autoware_set_compile_options(listener_node_exe)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
```

### 2-Tier Development Pattern

In all but the most trivial utilities, it is best to implement a code pattern with _at least_ two tiers of abstraction which would look something like:

1. A "core," pure C++ class which performs all basic algorithmic and utility functions which are not ROS-related.

   This class may use ROS utilities such as logging or message structures, but such use must be justified in terms of why it cannot be done via the class's external interface (e.g. the enclosing node uses information obtained via the class's external interface to populate log messages).

1. A "ROS Node" or "ROS Component" class which inherits from `rclcpp::Node` or a subclass, handles all ROS-specific functions.

   This class should instantiate the class defined in 1. and register the node as a [component](#contributors-guidelines-components), so it can be created with launch files.

In the rare case that fine-grained control over execution is desired, create a main function in a separate file with a [ROS Executor](http://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1Executor.html#details) to control provision of execution time of the node in some way (e.g. through calling `spin()`).

This design pattern helps to promote separation of concerns and code re-use. The core and the ROS node(s) can be implemented in separate packages; e.g. `foo` and `foo_nodes`. There are some trivial cases where a simple ROS Node that does not require a "core" are acceptable but these should be the exception, not the rule.

##### Compiler settings

The C++ standard is set in `autoware_auto_cmake.cmake` and becomes available to a package by depending on the `autoware_auto_cmake` package in `package.xml` as shown below. Compiler options and warning flags are set per target by calling the function `autoware_set_compile_options(${target})` defined in `autoware_auto_cmake.cmake` as well. It should be applied to every C++ target and in general Autoware.Auto code shall not generate warnings.

In case the warning flags are too strict for example when including external code, they can be selectively deactivated in special cases as follows:

```{cmake}
autoware_set_compile_options(${target})
target_compile_options(${target} PRIVATE -Wno-double-promotion)
```

#### Minimal Package.xml Example {#contributors-guidelines-minimal-package-xml-example}

The following is a minimal `package.xml` file to go with the above `CMakeLists.txt` example:

```{xml}
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>composition_example</name>
  <version>0.0.1</version>
  <description>Example of node composition</description>
  <maintainer email="my.email@example.com">The Autoware Foundation</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake_auto</buildtool_depend>
  <buildtool_depend>autoware_auto_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Resources

- [rclcpp_components in Dashing Diademata Release Notes](https://index.ros.org/doc/ros2/Releases/Release-Dashing-Diademata/#rclcpp-components)
