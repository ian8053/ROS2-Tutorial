# 02 - Publisher / Subscriber

## 概念

Publisher 和 Subscriber 是 ROS2 最基本的通訊模式。

- **Publisher**: 發布訊息到 Topic
- **Subscriber**: 訂閱 Topic 並接收訊息

### 架構圖

```
Publisher Node                              Subscriber Node
(MinimalPublisher)                          (MinimalSubscriber)
      │                                            ▲
      │ 發布訊息                                   │ 收到訊息
      ▼                                            │
┌──────────────────────────────────────────────────────────┐
│                    Topic "/topic"                        │
│                   （邏輯上的頻道名稱）                    │
├──────────────────────────────────────────────────────────┤
│                        DDS                               │
│              （底層傳輸，自動處理）                       │
└──────────────────────────────────────────────────────────┘
```

### Node、Topic、DDS 的關係

| 層級 | 說明 | 你需要管嗎 |
|------|------|-----------|
| **Node** | 程式（Publisher Node、Subscriber Node 是兩個獨立程式） | ✅ 你要寫 |
| **Topic** | 頻道名稱（`/topic`），Node 之間溝通的管道 | ✅ 你要指定 |
| **DDS** | 底層通訊協定，負責發現節點、傳輸資料 | ❌ 通常不用管 |

### 類比

```
Topic 像是「電視頻道名稱」
    ├── /camera/image     ← 攝影機頻道
    ├── /odom             ← 里程計頻道
    └── /cmd_vel          ← 速度指令頻道

DDS 像是「有線電視系統」
    → 負責把訊號從發送端傳到接收端
    → 你不用管它怎麼傳，只要知道頻道名稱
```

### 簡化版

```
Publisher Node ──→ Topic ──→ Subscriber Node
                    │
               DDS（底層默默工作）
```

**重點：Publisher 和 Subscriber 是兩個不同的 Node（獨立程式），透過 Topic 溝通。**

---

## 建立 Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_pub_sub --dependencies rclcpp std_msgs
```

---

## C++ 範例

### Publisher (publisher_node.cpp)

```cpp
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher() : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello World: " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
```

#### 語法解釋

##### 1. 類別繼承

```cpp
class MinimalPublisher : public rclcpp::Node
```

| 部分 | 說明 |
|------|------|
| `MinimalPublisher` | 你自己定義的類別 |
| `:` | 繼承符號 |
| `public rclcpp::Node` | 繼承 ROS2 的 Node 類別 |

```
類比：
ROS2 的 Node 就像「汽車底盤」（提供基本功能）
    ↓
你設計「車身、座椅、顏色」
    ↓
組合成「你的車」= MinimalPublisher
```

##### 2. 初始化列表 `:`

```cpp
MinimalPublisher() : Node("minimal_publisher"), count_(0)
//                 ↑
//                 這個冒號 = 初始化列表
```

| 部分 | 說明 |
|------|------|
| `Node("minimal_publisher")` | 呼叫父類別建構函數，設定節點名稱 |
| `count_(0)` | 初始化成員變數 count_ 為 0 |

**為什麼用 `:` 而不是在 `{}` 裡賦值？**

```cpp
// 方法 A：初始化列表（推薦）
MinimalPublisher() : count_(0) { }

// 方法 B：在函數裡賦值（效率較低）
MinimalPublisher() {
    count_ = 0;  // 先建立再賦值，多一步
}
```

父類別建構函數**只能用 `:`**，不能寫在 `{}` 裡。

##### 3. `this` 指標

```cpp
publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
//           ↑
//           this = 指向自己的指標
```

**範例**：

```cpp
class Dog {
public:
    string name;
    void bark() {
        cout << this->name << " 汪汪叫" << endl;
    }
};

Dog 小白; 小白.name = "小白";
Dog 小黑; 小黑.name = "小黑";

小白.bark();  // this = &小白 → 印出「小白 汪汪叫」
小黑.bark();  // this = &小黑 → 印出「小黑 汪汪叫」
```

**`this` 就是「我自己」的意思。**

##### 4. 建立 Publisher

```cpp
publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
```

| 部分 | 說明 |
|------|------|
| `this->` | 我自己這個物件 |
| `create_publisher<>` | Node 提供的函數 |
| `std_msgs::msg::String` | 訊息類型 |
| `"topic"` | Topic 名稱 |
| `10` | Queue size（緩衝區大小） |

##### 5. `std::bind` 綁定函數

```cpp
timer_ = this->create_wall_timer(
    500ms,
    std::bind(&MinimalPublisher::timer_callback, this)
);
```

**為什麼需要 bind？**

`timer_callback` 是成員函數，需要知道是**哪個物件**在呼叫。

```
情境：你開了兩間披薩店

PizzaShop 達美樂;
PizzaShop 必勝客;

定時器：「每 30 分鐘呼叫 make_pizza()」
    ↓
問題：呼叫誰的 make_pizza？
    ↓
解法：用 bind 綁定
    std::bind(&PizzaShop::make_pizza, &達美樂)
    → 「呼叫達美樂的 make_pizza」
```

**拆解**：

```cpp
std::bind(&MinimalPublisher::timer_callback, this)
         │                                   │
         │                                   └── 綁定到「我自己」
         └── 函數指標（& 取得成員函數位址）
```

##### 6. 為什麼用 `&`？

```cpp
// ❌ 錯誤
std::bind(timer_callback, this);
// 編譯器：「timer_callback 是什麼？找不到」

// ✅ 正確
std::bind(&MinimalPublisher::timer_callback, this);
// 編譯器：「是 MinimalPublisher 裡的 timer_callback」
```

成員函數要用 `&Class::function` 格式。

##### 7. `size_t` 型別

```cpp
size_t count_;
```

| 型別 | 範圍 | 用途 |
|------|------|------|
| `int` | -21億 ~ +21億 | 一般整數 |
| `size_t` | 0 ~ 42億 | 計數器（不能是負數） |

---

### Subscriber (subscriber_node.cpp)

```cpp
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber() : Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10,
            std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
```

#### 語法解釋

##### 1. 建立 Subscription

```cpp
subscription_ = this->create_subscription<std_msgs::msg::String>(
    "topic", 10,
    std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
```

| 參數 | 說明 |
|------|------|
| `"topic"` | 要訂閱的 Topic 名稱 |
| `10` | Queue size |
| `std::bind(...)` | 收到訊息時要呼叫的函數 |

##### 2. `std::placeholders::_1`

```cpp
std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1)
//                                                  ↑
//                                                  佔位符
```

`topic_callback` 需要一個參數（收到的訊息），`_1` 表示「第一個傳入的參數」。

```
當收到訊息時：
    ROS2 呼叫 callback(msg)
        ↓
    _1 = msg
        ↓
    實際執行 this->topic_callback(msg)
```

##### 3. Callback 函數

```cpp
void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
//                  ↑                                         ↑
//                  參數：收到的訊息                           這個函數不會修改物件
```

| 部分 | 說明 |
|------|------|
| `const ... msg` | 訊息是唯讀的 |
| `SharedPtr` | 智慧指標（自動管理記憶體） |
| `const` (最後) | 這個函數不會修改類別的成員變數 |

---

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_pub_sub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Publisher
add_executable(publisher_node src/publisher_node.cpp)
ament_target_dependencies(publisher_node rclcpp std_msgs)

# Subscriber
add_executable(subscriber_node src/subscriber_node.cpp)
ament_target_dependencies(subscriber_node rclcpp std_msgs)

install(TARGETS
  publisher_node
  subscriber_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

---

## 編譯與執行

```bash
cd ~/ros2_ws
colcon build --packages-select my_pub_sub
source install/setup.bash

# 終端 1：啟動 Publisher
ros2 run my_pub_sub publisher_node

# 終端 2：啟動 Subscriber
ros2 run my_pub_sub subscriber_node
```

**預期輸出**：

```
# Publisher 終端
[INFO] [minimal_publisher]: Publishing: 'Hello World: 0'
[INFO] [minimal_publisher]: Publishing: 'Hello World: 1'
...

# Subscriber 終端
[INFO] [minimal_subscriber]: I heard: 'Hello World: 0'
[INFO] [minimal_subscriber]: I heard: 'Hello World: 1'
...
```

---

## 練習

1. 修改發布頻率（從 500ms 改成 1000ms）
2. 使用不同的訊息類型（`std_msgs::msg::Int32`）
3. 建立多個 Publisher 發布到同一個 Topic
4. 一個 Subscriber 訂閱多個 Topic
