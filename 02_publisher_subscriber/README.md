# 02 - Publisher / Subscriber

## 概念

Publisher 和 Subscriber 是 ROS2 最基本的通訊模式。

- **Publisher**: 發布訊息到 topic
- **Subscriber**: 訂閱 topic 並接收訊息

## 建立 Package

```bash
cd ~/ros2_ws/src

# Python 版本
ros2 pkg create --build-type ament_python my_pub_sub_py --dependencies rclpy std_msgs

# C++ 版本
ros2 pkg create --build-type ament_cmake my_pub_sub_cpp --dependencies rclcpp std_msgs
```

## Python 範例

### Publisher (publisher_node.py)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber (subscriber_node.py)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 編譯與執行

```bash
cd ~/ros2_ws
colcon build --packages-select my_pub_sub_py
source install/setup.bash

# 終端 1
ros2 run my_pub_sub_py publisher_node

# 終端 2
ros2 run my_pub_sub_py subscriber_node
```

## 練習

1. 修改發布頻率
2. 使用不同的訊息類型 (Int32, Float64, etc.)
3. 建立多個 publisher 發布到同一個 topic
