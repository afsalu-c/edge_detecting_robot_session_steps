Here's a **GitHub README** that walks users step-by-step through building the repo from scratch during the workshop. It's written clearly and concisely, with all required commands and brief explanations for each step.

---

# 🛠️ Build Your Own ROS2 Robot from Scratch – Workshop Starter Guide

Welcome to the **"Mastering ROS2 – Build an Edge-Avoiding Robot from Scratch"** workshop!
This guide will walk you through setting up your ROS2 package and building your first robot from **scratch**, step-by-step.

---

## 📁 1. Create Your Workspace

Open your terminal and run the following:

```bash
cd ~
mkdir -p bot_ws/src
cd bot_ws/src
```

---

## 🧱 2. Create a New ROS2 Package

```bash
ros2 pkg create bot_description --build-type ament_cmake
```

This creates the package `bot_description` with `ament_cmake` build system.

---

## 📁 3. Add Essential Folders

Navigate to the package and add necessary directories:

```bash
cd bot_description
mkdir launch models rviz urdf worlds
```

---

## 💻 4. Open in VS Code

Return to the `src` directory and open the workspace in VS Code:

```bash
cd ~/bot_ws/src
code .
```

---

## ⚙️ 5. Create the Base URDF (bot\_base.xacro)

Go to the `urdf` folder in VS Code and create a file named `bot_base.xacro`. Add the following step by step:

### Step 1: XML declaration and xacro root

```xml
<?xml version="1.0" ?>
<robot name="bot" xmlns:xacro="http://www.ros.org/wiki/xacro"></robot>
```

### Step 2: Add `base_footprint` link

```xml
<link name="base_footprint"/>
```

### Step 3: Add `base_link`

```xml
<link name="base_link">
    <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="5.0"/>
        <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <geometry>
            <box size="0.7 0.5 0.2"/>
        </geometry>
        <material name="red">
            <color rgba="1.0 0.0 0.0 1.0"/>
        </material>
    </visual>
    <collision>
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <geometry>
            <box size="0.7 0.5 0.2"/>
        </geometry>
    </collision>
</link>
```

### Step 4: Add a fixed joint

```xml
<joint name="base_joint" type="fixed">
    <origin xyz="0.0 0.0 0.15" rpy="0.0 0.0 0.0"/>
    <parent link="base_footprint"/>
    <child link="base_link"/>
</joint>
```

---

## 🧩 6. Create the Main URDF (bot.urdf.xacro)

In the same `urdf` folder, create a file called `bot.urdf.xacro` and add:

```xml
<?xml version="1.0" ?>
<robot name="bot" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:include filename="$(find bot_description)/urdf/bot_base.xacro" />
</robot>
```

---

## 🚀 7. Create the Launch File

Go to the `launch` folder and create `display.launch.py`:

```python
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bot_description_dir = get_package_share_directory("bot_description")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        bot_description_dir, "urdf", "bot.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file")
    
    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_sim:=True",
        ]),
        value_type=str
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(bot_description_dir, "rviz", "display.rviz")],
    )

    return LaunchDescription([
        model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
```

---

🎯 RViz Setup
8.A. Create RViz Configuration File

File: rviz/display.rviz

```bash
Panels:
  - Class: rviz_common/Displays
    Help Height: 0
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /RobotModel1/Description Topic1
      Splitter Ratio: 0.5833333134651184
    Tree Height: 685
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 255; 255
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 0
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: LaserScan
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.029999999329447746
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /scan
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Description File: ""
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
        back_castor_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        base_footprint:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        base_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        front_castor_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        laser_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        left_wheel_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        right_wheel_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
      Mass Properties:
        Inertia: false
        Mass: false
      Name: RobotModel
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
        back_castor_link:
          Value: true
        base_footprint:
          Value: true
        base_link:
          Value: true
        front_castor_link:
          Value: true
        laser_link:
          Value: true
        left_wheel_link:
          Value: true
        odom:
          Value: true
        right_wheel_link:
          Value: true
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: false
      Tree:
        base_footprint:
          base_link:
            back_castor_link:
              {}
            front_castor_link:
              {}
            laser_link:
              {}
            left_wheel_link:
              {}
            right_wheel_link:
              {}
        odom:
          {}
      Update Interval: 0
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_footprint
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Covariance x: 0.25
      Covariance y: 0.25
      Covariance yaw: 0.06853891909122467
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
    - Class: nav2_rviz_plugins/GoalTool
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 3.625094413757324
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: -0.07663324475288391
        Y: 0.07754191756248474
        Z: -0.014682665467262268
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.6752017736434937
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz_default_plugins)
      Yaw: 2.0423831939697266
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 836
  Hide Left Dock: false
  Hide Right Dock: true
  QMainWindow State: 000000ff00000000fd00000004000000000000018b000002eafc020000000cfb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000002ea000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261fb00000018004e0061007600690067006100740069006f006e0020003201000001ee000001390000000000000000fb0000001e005200650061006c00730065006e0073006500430061006d006500720061000000028f000000980000000000000000fb0000000c00430061006d0065007200610000000267000000c00000000000000000fb0000000c00430061006d0065007200610100000267000000c00000000000000000000000010000010f000002eafc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073000000003d000002ea000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004420000000000000000fb0000000800540069006d00650100000000000004500000000000000000000004af000002ea00000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: true
  Width: 1600
  X: 642
  Y: 102

```

---

Absolutely! Here's how you can write the next step (📦 **Step 22**) for your GitHub `README.md`, showing how to set up the `CMakeLists.txt` and `package.xml` for the `bot_description` package — with clear explanations and emojis:

---

## 📦 8.B.: Setup `CMakeLists.txt` and `package.xml` for `bot_description`

This step sets up the core metadata and build configuration files for the `bot_description` package.

---

### 🧱 `CMakeLists.txt`

Create or update your `CMakeLists.txt` inside the `bot_description` folder with the following content:

```cmake
cmake_minimum_required(VERSION 3.8)
project(bot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install important directories like URDF, launch, rviz, etc.
install(
  DIRECTORY launch models urdf rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

# Optional testing support
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

✅ **What this does**:

* Sets the CMake version and project name.
* Enables strict compile warnings.
* Finds the `ament_cmake` build system.
* Installs your `launch`, `urdf`, `rviz`, `models`, and `worlds` folders.
* Optionally sets up testing and linting tools.

---

### 📄 `package.xml`

Add the following `package.xml` file in the same `bot_description` folder:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>bot_description</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="kutkarsh706@gmail.com">utk</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Runtime dependencies -->
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>urdf</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>ros2launch</exec_depend>
  <exec_depend>ros_gz_sim</exec_depend>

  <!-- Test dependencies -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

🧾 **What this includes**:

* Metadata like package name, version, maintainer, and license.
* Declares **runtime dependencies** like `xacro`, `robot_state_publisher`, `rviz2`, and `ros_gz_sim`.
* Adds **testing tools** under test dependencies.
* Uses `ament_cmake` as the build tool.

---

### 🧪 Build & Verify

Once these files are added, **build the workspace** again:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

✅ Now your robot description package is fully structured and ready to be used in both RViz and Gazebo!

---

🧭 **Next steps?**

* Add `ros_gz` launch files.
* Spawn your robot in a Gazebo world.
* Add controller plugins via `ros2_control`.

Let me know if you'd like help writing those steps too!


## 🛠️ 9. Build the Workspace

Go to your workspace root and build the package:

```bash
cd ~/bot_ws
colcon build
```

---

## 🌐 10. Source and Launch

After a successful build:

```bash
source ~/.bashrc
ros2 launch bot_description display.launch.py
```

Your robot should now appear in **RViz** with the correct URDF structure!

---

Sure! Here's a **cleaner rewrite of Steps 11 to 13**, with each component (left/right wheels, front/rear castors, sensors) broken into **separate code snippets** for clarity in your `README.md`.

---

## **Step 11: Add Left and Right Wheels**

### 📄 Edit `bot_base.xacro`

Open the file:

```bash
cd ~/bot_ws/src/bot_description/urdf
nano bot_base.xacro
```

### ➕ Add Right Wheel

```xml
<!-- Right Wheel -->
<link name="right_wheel_link">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="1.0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>
  <visual>
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <geometry>
      <cylinder radius="0.15" length="0.05"/>
    </geometry>
    <material name="blue">
      <color rgba="0.0 0.0 1.0 1.0"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <geometry>
      <cylinder radius="0.15" length="0.05"/>
    </geometry>
  </collision>
</link>

<joint name="right_wheel_joint" type="continuous">
  <origin xyz="0.0 -0.275 0.0" rpy="1.57 0.0 0.0"/>
  <parent link="base_link"/>
  <child link="right_wheel_link"/>
  <axis xyz="0.0 0.0 -1.0"/>
</joint>
```

### ➕ Add Left Wheel

```xml
<!-- Left Wheel -->
<link name="left_wheel_link">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="1.0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>
  <visual>
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <geometry>
      <cylinder radius="0.15" length="0.05"/>
    </geometry>
    <material name="blue">
      <color rgba="0.0 0.0 1.0 1.0"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <geometry>
      <cylinder radius="0.15" length="0.05"/>
    </geometry>
  </collision>
</link>

<joint name="left_wheel_joint" type="continuous">
  <origin xyz="0.0 0.275 0.0" rpy="1.57 0.0 0.0"/>
  <parent link="base_link"/>
  <child link="left_wheel_link"/>
  <axis xyz="0.0 0.0 -1.0"/>
</joint>
```

### 🔧 Build and Launch RViz

```bash
cd ~/bot_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch bot_description display.launch.py
```

✅ Check if **both wheels** are correctly attached in RViz.

---

## **Step 12: Add Castor Wheels**

### ➕ Add Front Castor

```xml
<!-- Front Castor -->
<link name="front_castor_link">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="0.5"/>
    <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
  </inertial>
  <visual>
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <geometry>
      <sphere radius="0.045"/>
    </geometry>
    <material name="black">
      <color rgba="0.0 0.0 0.0 1.0"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <geometry>
      <sphere radius="0.045"/>
    </geometry>
  </collision>
</link>

<joint name="front_castor_joint" type="fixed">
  <origin xyz="0.25 0.0 -0.1" rpy="0.0 0.0 0.0"/>
  <parent link="base_link"/>
  <child link="front_castor_link"/>
</joint>
```

### ➕ Add Rear Castor

```xml
<!-- Rear Castor -->
<link name="back_castor_link">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="0.5"/>
    <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
  </inertial>
  <visual>
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <geometry>
      <sphere radius="0.045"/>
    </geometry>
    <material name="black">
      <color rgba="0.0 0.0 0.0 1.0"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <geometry>
      <sphere radius="0.045"/>
    </geometry>
  </collision>
</link>

<joint name="back_castor_joint" type="fixed">
  <origin xyz="-0.25 0.0 -0.1" rpy="0.0 0.0 0.0"/>
  <parent link="base_link"/>
  <child link="back_castor_link"/>
</joint>
```

### 🔧 Rebuild and Launch RViz

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch bot_description display.launch.py
```

✅ Check if **both castors** are added and positioned correctly.

---

## **Step 13: Add Sensor Using a Separate Xacro File**

### 🆕 Create `bot_sensors.xacro`

```bash
cd ~/bot_ws/src/bot_description/urdf
nano bot_sensors.xacro
```

### 🧩 Add the Sensor Link and Joint

```xml
<?xml version="1.0" ?>
<robot name="bot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <link name="laser_link">
    <inertial>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <mass value="0.3"/>
      <inertia ixx="0.0002" ixy="0.0" ixz="0.0" iyy="0.0004" iyz="0.0" izz="0.0005"/>
    </inertial>
    <visual>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <geometry>
        <box size="0.05 0.2 0.1"/>
      </geometry>
      <material name="green">
        <color rgba="0.0 1.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <geometry>
        <box size="0.05 0.2 0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="laser_joint" type="fixed">
    <origin xyz="0.35 0.0 0.0" rpy="0.0 0.0 0.0"/>
    <parent link="base_link"/>
    <child link="laser_link"/>
  </joint>

</robot>
```

### 🧩 Include in `bot.urdf.xacro`

Open `bot.urdf.xacro`:

```bash
nano bot.urdf.xacro
```

Add this line **at the top**, before `<robot>` begins:

```xml
<xacro:include filename="$(find bot_description)/urdf/bot_sensors.xacro" />
```

### ✅ Why Use Xacro?

Using `.xacro` provides the following benefits:

* ✅ Reuse macros like functions to avoid repeating the same XML blocks.
* ✅ Parameterize sizes, colors, and positions using variables.
* ✅ Split URDF into smaller, manageable pieces.
* ✅ Easier maintenance and debugging.

---

### 🔁 Final Build and Visual Test

```bash
cd ~/bot_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch bot_description display.launch.py
```

✅ You should now see the full robot in RViz with:

* Base link
* Left and right wheels
* Front and back castors
* A front-mounted laser sensor

---


