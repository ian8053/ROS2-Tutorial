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

```cpp
// 在 launch file 中使用 (XML 格式)
// my_robot.launch.xml
```

```xml
<launch>
  <node pkg="tf2_ros" exec="static_transform_publisher"
        args="--x 0.1 --y 0 --z 0.2 --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id camera_link"/>
</launch>
```

## Dynamic Transform (動態轉換)

會隨時間變化的轉換，例如：機器人在地圖中的位置

### C++ 範例 - Broadcaster

```cpp
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class DynamicTFBroadcaster : public rclcpp::Node
{
public:
    DynamicTFBroadcaster() : Node("dynamic_tf_broadcaster")
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&DynamicTFBroadcaster::broadcast_timer_callback, this));
    }

private:
    void broadcast_timer_callback()
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = "robot";

        // 設定位置
        t.transform.translation.x = 1.0;
        t.transform.translation.y = 2.0;
        t.transform.translation.z = 0.0;

        // 設定旋轉 (quaternion)
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(t);
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicTFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
```

## TF Listener (監聽轉換)

```cpp
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class TFListener : public rclcpp::Node
{
public:
    TFListener() : Node("tf_listener")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = this->create_wall_timer(
            1s, std::bind(&TFListener::timer_callback, this));
    }

private:
    void timer_callback()
    {
        try {
            auto transform = tf_buffer_->lookupTransform(
                "map", "base_link", tf2::TimePointZero);

            RCLCPP_INFO(this->get_logger(),
                "Transform: x=%.2f, y=%.2f, z=%.2f",
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        }
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFListener>());
    rclcpp::shutdown();
    return 0;
}
```

### CMakeLists.txt

```cmake
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
ament_target_dependencies(tf_broadcaster rclcpp tf2_ros geometry_msgs)

add_executable(tf_listener src/tf_listener.cpp)
ament_target_dependencies(tf_listener rclcpp tf2_ros geometry_msgs)

install(TARGETS
  tf_broadcaster
  tf_listener
  DESTINATION lib/${PROJECT_NAME})
```

## 練習

1. 用 `view_frames` 查看 turtlesim 的 TF tree
2. 發布一個 static transform
3. 寫一個會移動的 dynamic transform broadcaster
