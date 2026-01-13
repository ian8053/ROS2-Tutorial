# 02 - Publisher / Subscriber

## 概念

Publisher 和 Subscriber 是 ROS2 最基本的通訊模式。

- **Publisher**: 發布訊息到 topic
- **Subscriber**: 訂閱 topic 並接收訊息

---

## C++ 語法詳解

### 1. 類別繼承與建構函數

```cpp
class MinimalPublisher : public rclcpp::Node
```

```
MinimalPublisher        ← 你自己定義的類別
    :                   ← 繼承符號
public rclcpp::Node     ← 繼承 ROS2 的 Node 類別
```

| 名稱 | 來源 | 說明 |
|------|------|------|
| `Node` | ROS2 提供 | 所有節點的父類別 |
| `MinimalPublisher` | 你自己寫 | 繼承 Node 的自訂類別 |

**類比**：
```
ROS2 的 Node 就像「汽車底盤」
    ↓
你自己設計「車身、座椅、顏色」
    ↓
組合起來變成「你的車」= MinimalPublisher
```

---

### 2. 初始化列表 `:`

```cpp
MinimalPublisher() : Node("minimal_publisher"), count_(0)
//                 ↑
//                 這個冒號 = 初始化列表
```

```
冒號後面 = 在建構函數本體「之前」執行的初始化

Node("minimal_publisher")  → 呼叫父類別建構函數，設定節點名稱
count_(0)                  → 初始化成員變數 count_ 為 0
```

**為什麼用 `:` 而不是在 `{}` 裡賦值？**

```cpp
// 方法 A：初始化列表（推薦）
MinimalPublisher() : count_(0) { }

// 方法 B：在函數裡賦值（效率較低）
MinimalPublisher() {
    count_ = 0;  // 這是「先建立再賦值」
}
```

父類別建構函數**只能用 `:`**，不能寫在 `{}` 裡。

---

### 3. `size_t` 型別

```cpp
size_t count_;
```

```
size_t = 無符號整數（unsigned integer）
       = 專門用來表示「大小」或「數量」
       = 不能是負數
```

| 型別 | 範圍 | 用途 |
|------|------|------|
| `int` | -21億 ~ +21億 | 一般整數 |
| `size_t` | 0 ~ 42億（或更大） | 陣列索引、計數器 |

```
count_ 是用來計數「發了幾次訊息」
    ↓
不可能發 -5 次訊息
    ↓
所以用 size_t（永遠 ≥ 0）
```

---

### 4. `this` 指標

`this` = **指向自己的指標**

```cpp
publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
//           ↑
//           this = 我自己這個物件
```

**範例**：

```cpp
class Dog {
public:
    string name;

    void bark() {
        cout << this->name << " 汪汪叫" << endl;
        //       ↑
        //       this = 正在執行這個函數的那隻狗
    }
};

int main() {
    Dog 小白;
    小白.name = "小白";

    Dog 小黑;
    小黑.name = "小黑";

    小白.bark();  // this = &小白，印出「小白 汪汪叫」
    小黑.bark();  // this = &小黑，印出「小黑 汪汪叫」
}
```

**`this` 就是「我自己」的意思。**

---

### 5. `std::bind` 綁定函數

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

定時器機器人：「每 30 分鐘呼叫 make_pizza()」
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
         │
         └── 取得「timer_callback」的函數指標
```

---

### 6. 為什麼用 `&`？

`&` 用來取得成員函數的指標：

```cpp
// 錯誤寫法
std::bind(timer_callback, this);
// ❌ 編譯器：「timer_callback 是什麼？」

// 正確寫法
std::bind(&MinimalPublisher::timer_callback, this);
// ✅ 編譯器：「是 MinimalPublisher 裡的 timer_callback」
```

因為 `timer_callback` 是**成員函數**（屬於某個 class），要用 `&Class::function` 格式。

---

### 7. 建立 Publisher

```cpp
publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
```

```
this->                              ← 我自己
create_publisher<                   ← 呼叫 Node 提供的函數
  std_msgs::msg::String             ← 訊息類型
>("topic", 10)                      ← topic 名稱, queue size
```

---

## 完整程式碼流程

```
MinimalPublisher 類別
├── 繼承自 Node
├── 建構時：
│   ├── Node("minimal_publisher") → 設定節點名稱
│   └── count_ = 0
│
├── publisher_ = 建立發布器
│   ├── 訊息類型：String
│   ├── Topic："topic"
│   └── Queue：10
│
└── timer_ = 建立定時器
    ├── 每 500ms 觸發
    └── 觸發時呼叫 this->timer_callback()
```

---

## 建立 Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_pub_sub --dependencies rclcpp std_msgs
```

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

## 編譯與執行

```bash
cd ~/ros2_ws
colcon build --packages-select my_pub_sub
source install/setup.bash

# 終端 1
ros2 run my_pub_sub publisher_node

# 終端 2
ros2 run my_pub_sub subscriber_node
```

## 練習

1. 修改發布頻率
2. 使用不同的訊息類型 (Int32, Float64, etc.)
3. 建立多個 publisher 發布到同一個 topic
