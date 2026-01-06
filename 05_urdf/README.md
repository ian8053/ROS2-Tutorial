# 05 - URDF 機器人模型

## 概念

URDF (Unified Robot Description Format) 是用 XML 描述機器人的格式。

定義：
- **Links**: 機器人的剛體部件（車身、輪子、手臂）
- **Joints**: 連接 links 的關節（固定、旋轉、滑動）

## 基本結構

```xml
<?xml version="1.0"?>
<robot name="my_robot">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Wheel Link -->
  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
    </visual>
  </link>

  <!-- Joint -->
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0.2 0.15 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

</robot>
```

## 幾何形狀

```xml
<!-- 方塊 -->
<box size="長 寬 高"/>

<!-- 圓柱 -->
<cylinder radius="半徑" length="長度"/>

<!-- 球 -->
<sphere radius="半徑"/>

<!-- 載入 mesh -->
<mesh filename="package://my_pkg/meshes/model.stl"/>
```

## Joint 類型

| 類型 | 說明 |
|------|------|
| `fixed` | 固定，不能動 |
| `continuous` | 無限旋轉（輪子） |
| `revolute` | 有限旋轉（有角度限制） |
| `prismatic` | 滑動關節 |

## 使用 Xacro

Xacro 是 URDF 的擴展，支援變數和巨集。

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

  <!-- 定義變數 -->
  <xacro:property name="wheel_radius" value="0.05"/>
  <xacro:property name="wheel_width" value="0.02"/>

  <!-- 定義巨集 -->
  <xacro:macro name="wheel" params="name x y">
    <link name="${name}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </visual>
    </link>
    <joint name="${name}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${name}_wheel"/>
      <origin xyz="${x} ${y} 0" rpy="1.5708 0 0"/>
    </joint>
  </xacro:macro>

  <!-- 使用巨集 -->
  <xacro:wheel name="front_left" x="0.2" y="0.15"/>
  <xacro:wheel name="front_right" x="0.2" y="-0.15"/>

</robot>
```

## 轉換 Xacro 到 URDF

```bash
xacro my_robot.urdf.xacro > my_robot.urdf
```

## 在 RViz 中顯示

### Launch file

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'my_robot.urdf'
    )

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),
    ])
```

## 練習

1. 建立一個簡單的兩輪機器人 URDF
2. 用 RViz 顯示你的機器人
3. 用 Xacro 重構，使用巨集產生左右輪
