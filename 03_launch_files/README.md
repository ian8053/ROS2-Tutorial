# 03 - Launch Files

## 概念

Launch files 用來同時啟動多個 nodes，並設定參數、remap topics 等。

ROS2 支援三種格式：**XML**、**YAML**、**Python**。這裡以 XML 為主。

## XML 格式 (推薦)

### 基本結構

```xml
<?xml version="1.0"?>
<launch>
  <node pkg="package_name" exec="executable_name" name="node_name" output="screen"/>
</launch>
```

### 完整範例 (my_robot.launch.xml)

```xml
<?xml version="1.0"?>
<launch>
  <!-- 宣告參數 -->
  <arg name="use_sim_time" default="false" description="Use simulation time"/>

  <!-- 啟動 turtlesim -->
  <node pkg="turtlesim" exec="turtlesim_node" name="sim" output="screen">
    <param name="background_r" value="100"/>
    <param name="background_g" value="100"/>
    <param name="background_b" value="200"/>
  </node>

  <!-- 啟動鍵盤控制 -->
  <node pkg="turtlesim" exec="turtle_teleop_key" name="teleop" output="screen"/>
</launch>
```

## 常用功能

### Remapping Topics

```xml
<node pkg="my_package" exec="my_node" name="my_node">
  <remap from="/input" to="/camera/image_raw"/>
  <remap from="/output" to="/processed_image"/>
</node>
```

### 設定參數

```xml
<node pkg="my_package" exec="my_node" name="my_node">
  <param name="rate" value="10.0"/>
  <param name="topic_name" value="my_topic"/>
</node>
```

### 從 YAML 讀取參數

```xml
<node pkg="my_package" exec="my_node" name="my_node">
  <param from="$(find-pkg-share my_package)/config/params.yaml"/>
</node>
```

### 條件啟動

```xml
<arg name="use_rviz" default="true"/>

<node pkg="rviz2" exec="rviz2" name="rviz2" if="$(var use_rviz)"/>
```

### Include 其他 launch file

```xml
<include file="$(find-pkg-share other_package)/launch/other.launch.xml">
  <arg name="param1" value="value1"/>
</include>
```

## C++ 專案結構

```
my_package/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── my_node.cpp
├── launch/
│   └── my_robot.launch.xml
└── config/
    └── params.yaml
```

### CMakeLists.txt 安裝 launch files

```cmake
# 安裝 launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安裝 config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
```

## 執行 Launch File

```bash
# 方法 1: 從 package 執行
ros2 launch package_name my_robot.launch.xml

# 方法 2: 帶參數
ros2 launch package_name my_robot.launch.xml use_sim_time:=true

# 方法 3: 直接執行檔案
ros2 launch /path/to/my_robot.launch.xml
```

## YAML 格式 (替代方案)

```yaml
# my_robot.launch.yaml
launch:
  - node:
      pkg: turtlesim
      exec: turtlesim_node
      name: sim
      output: screen

  - node:
      pkg: turtlesim
      exec: turtle_teleop_key
      name: teleop
      output: screen
```

## 練習

1. 建立一個 launch file 同時啟動 publisher 和 subscriber
2. 使用參數控制發布頻率
3. 練習 topic remapping
