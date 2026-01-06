# 03 - Launch Files

## 概念

Launch files 用來同時啟動多個 nodes，並設定參數、remap topics 等。

ROS2 使用 Python 撰寫 launch files。

## 基本結構

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='package_name',
            executable='node_name',
            name='custom_node_name',
            output='screen'
        ),
        # 可以加更多 nodes...
    ])
```

## 完整範例

### my_launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 宣告參數
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # 啟動 turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen'
        ),

        # 啟動鍵盤控制
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop',
            output='screen',
            prefix='xterm -e'  # 開新終端
        ),
    ])
```

## 常用功能

### Remapping Topics

```python
Node(
    package='my_package',
    executable='my_node',
    remappings=[
        ('/input', '/camera/image_raw'),
        ('/output', '/processed_image'),
    ]
)
```

### 設定參數

```python
Node(
    package='my_package',
    executable='my_node',
    parameters=[
        {'param_name': 'param_value'},
        {'rate': 10.0},
    ]
)
```

### 從 YAML 讀取參數

```python
Node(
    package='my_package',
    executable='my_node',
    parameters=['/path/to/params.yaml']
)
```

## 執行 Launch File

```bash
# 方法 1: 從 package 執行
ros2 launch package_name launch_file.py

# 方法 2: 直接執行檔案
ros2 launch /path/to/launch_file.py
```

## 練習

1. 建立一個 launch file 同時啟動 publisher 和 subscriber
2. 使用參數控制發布頻率
3. 練習 topic remapping
