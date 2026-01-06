# 04 - TF2 座標轉換

## 概念

TF2 是 ROS2 的座標轉換系統，用來追蹤多個座標系 (frames) 之間的關係。

### 為什麼重要？

機器人系統有很多座標系：
- `map` - 地圖原點
- `odom` - 里程計原點
- `base_link` - 機器人中心
- `camera_link` - 相機位置
- `lidar_link` - LiDAR 位置

TF2 幫你管理這些座標系之間的轉換。

## 常用指令

```bash
# 查看 TF tree
ros2 run tf2_tools view_frames

# 即時查看兩個 frame 之間的轉換
ros2 run tf2_ros tf2_echo frame1 frame2

# 監控 TF
ros2 run tf2_ros tf2_monitor
```

## Static Transform (靜態轉換)

固定不變的轉換，例如：相機固定在機器人上的位置

### 指令發布

```bash
ros2 run tf2_ros static_transform_publisher \
    --x 0.1 --y 0 --z 0.2 \
    --roll 0 --pitch 0 --yaw 0 \
    --frame-id base_link \
    --child-frame-id camera_link
```

### Launch file 發布

```python
from launch_ros.actions import Node

Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['--x', '0.1', '--y', '0', '--z', '0.2',
               '--roll', '0', '--pitch', '0', '--yaw', '0',
               '--frame-id', 'base_link',
               '--child-frame-id', 'camera_link']
)
```

## Dynamic Transform (動態轉換)

會隨時間變化的轉換，例如：機器人在地圖中的位置

### Python 範例

```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class DynamicTFBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

    def broadcast_timer_callback(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'robot'

        # 設定位置
        t.transform.translation.x = 1.0
        t.transform.translation.y = 2.0
        t.transform.translation.z = 0.0

        # 設定旋轉 (quaternion)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = DynamicTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()
```

## TF Listener (監聽轉換)

```python
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            self.get_logger().info(f'Transform: {transform}')
        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {e}')
```

## 練習

1. 用 `view_frames` 查看 turtlesim 的 TF tree
2. 發布一個 static transform
3. 寫一個會移動的 dynamic transform broadcaster
