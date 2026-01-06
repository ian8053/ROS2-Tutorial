# C++ 基礎觀念 - ROS2 會用到的

在看懂 ROS2 程式碼之前，需要先理解這些 C++ 觀念。

---

## 1. Class 類別

Class 就是一個「藍圖」，用來建立物件。

```cpp
// 定義一個 Dog 類別
class Dog
{
public:
    // 成員變數（屬於這個 class 的資料）
    std::string name;
    int age;

    // 成員函數（屬於這個 class 的功能）
    void bark()
    {
        std::cout << name << " says: Woof!" << std::endl;
    }
};

// 使用
int main()
{
    Dog myDog;           // 建立一個 Dog 物件
    myDog.name = "Buddy";
    myDog.age = 3;
    myDog.bark();        // 輸出: Buddy says: Woof!
}
```

### 成員變數 vs 區域變數

```cpp
class Dog
{
    std::string name;    // 成員變數：屬於整個 class，所有成員函數都可以用

    void bark()
    {
        int x = 10;      // 區域變數：只在這個函數裡面存在
        std::cout << name << std::endl;  // 可以用 name
    }

    void sleep()
    {
        std::cout << name << std::endl;  // 可以用 name
        // std::cout << x << std::endl;  // 錯誤！x 不存在於這裡
    }
};
```

---

## 2. 建構函數 (Constructor)

建立物件時自動執行的函數。

```cpp
class Dog
{
public:
    std::string name;
    int age;

    // 建構函數：跟 class 同名，沒有回傳型別
    Dog(std::string n, int a)
    {
        name = n;
        age = a;
        std::cout << "Dog created!" << std::endl;
    }
};

int main()
{
    Dog myDog("Buddy", 3);  // 建立時自動執行建構函數
    // 輸出: Dog created!
}
```

---

## 3. 初始化列表 `: xxx()`

建構函數的另一種寫法，**ROS2 大量使用這種寫法**。

```cpp
// 方法 A：在建構函數裡面賦值
Dog(std::string n, int a)
{
    name = n;
    age = a;
}

// 方法 B：初始化列表（效率更好）
Dog(std::string n, int a) : name(n), age(a)
{
    // 建構函數內容（可以空的）
}
```

兩種寫法結果一樣，但初始化列表效率更好。

### ROS2 的例子

```cpp
class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher() : Node("minimal_publisher"), count_(0)
    //                   ↑                          ↑
    //                   初始化父類別 Node           初始化 count_ 為 0
    {
        // 其他初始化...
    }

private:
    size_t count_;
};
```

**為什麼要用初始化列表？**
- 初始化父類別時**必須**用這種寫法
- `Node("minimal_publisher")` = 告訴父類別：這個 Node 的名字叫 "minimal_publisher"

---

## 4. 繼承 (Inheritance)

一個 class 可以繼承另一個 class 的功能。

```cpp
// 父類別（基底類別）
class Animal
{
public:
    std::string name;

    Animal(std::string n) : name(n) {}

    void eat()
    {
        std::cout << name << " is eating" << std::endl;
    }
};

// 子類別：繼承 Animal
class Dog : public Animal
{
public:
    // 呼叫父類別的建構函數
    Dog(std::string n) : Animal(n) {}

    void bark()
    {
        std::cout << name << " says: Woof!" << std::endl;
    }
};

int main()
{
    Dog myDog("Buddy");
    myDog.eat();   // 繼承自 Animal
    myDog.bark();  // Dog 自己的
}
```

### ROS2 的例子

```cpp
// MinimalPublisher 繼承自 rclcpp::Node
class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher() : Node("minimal_publisher")  // 呼叫父類別建構函數
    {
        // Node 提供的函數，我們可以直接用
        publisher_ = this->create_publisher<...>("topic", 10);
        timer_ = this->create_wall_timer(...);
    }
};
```

---

## 5. 指標 (Pointer)

指標 = 儲存記憶體位址的變數。

```cpp
int x = 10;       // 普通變數，值是 10
int* p = &x;      // 指標，儲存 x 的記憶體位址

std::cout << x << std::endl;   // 10（x 的值）
std::cout << &x << std::endl;  // 0x7fff...（x 的位址）
std::cout << p << std::endl;   // 0x7fff...（p 儲存的位址，跟上面一樣）
std::cout << *p << std::endl;  // 10（p 指向的值）
```

### `->` vs `.`

```cpp
Dog myDog("Buddy", 3);
Dog* pDog = &myDog;

// 用物件：用 .
myDog.bark();

// 用指標：用 ->
pDog->bark();
```

**簡單記：指標用 `->`，非指標用 `.`**

---

## 6. 智能指標 (Smart Pointer)

### 普通指標的問題

```cpp
int* p = new int(10);  // 分配記憶體
// ... 做一些事 ...
// 忘記 delete p; → 記憶體洩漏！
```

### 智能指標解決這個問題

```cpp
#include <memory>

std::shared_ptr<int> p = std::make_shared<int>(10);
// ... 做一些事 ...
// 不用 delete！智能指標會自動釋放記憶體
```

### ROS2 的例子

```cpp
// ROS2 大量使用 SharedPtr
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

// 建立智能指標
publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

// 使用（因為是指標，所以用 ->）
publisher_->publish(message);
```

**白話：SharedPtr = 會自動清理的指標，不用擔心記憶體洩漏**

---

## 7. `this` 指標

`this` = 指向「自己這個物件」的指標。

```cpp
class Dog
{
public:
    std::string name;

    void printName()
    {
        // 這兩個一樣
        std::cout << name << std::endl;
        std::cout << this->name << std::endl;
    }

    Dog* getSelf()
    {
        return this;  // 回傳自己的指標
    }
};
```

**為什麼要寫 `this->`？**
- 讓程式碼更清楚
- 某些情況下避免命名衝突
- 純粹是風格問題，可以省略

---

## 8. 命名空間 (Namespace)

避免名稱衝突。

```cpp
// 假設有兩個函式庫都有 print() 函數
namespace libraryA {
    void print() { std::cout << "A" << std::endl; }
}

namespace libraryB {
    void print() { std::cout << "B" << std::endl; }
}

int main()
{
    libraryA::print();  // 輸出: A
    libraryB::print();  // 輸出: B
}
```

### `#include` vs `::`

```cpp
#include <string>      // 載入 string 的程式碼到你的程式
std::string name;      // 使用時要指定命名空間 std::

// 類比：
// #include = 把書搬進圖書館
// std:: = 告訴你這本書在哪個書架
```

**兩個都要寫，一個是載入，一個是指定位置**

---

## 9. `std::bind` 綁定函數

把「成員函數」和「物件」綁在一起。

### 問題

```cpp
class Dog
{
public:
    void bark() { std::cout << "Woof!" << std::endl; }
};

// 我想把 bark 傳給別人執行，但 bark 需要知道是「哪隻狗」的
```

### 解法

```cpp
#include <functional>

Dog myDog;

// 把 bark 和 myDog 綁在一起
auto boundBark = std::bind(&Dog::bark, &myDog);

// 現在可以直接呼叫
boundBark();  // 輸出: Woof!
```

### ROS2 的例子

```cpp
// Timer 需要一個函數，每 500ms 執行一次
// 但 timer_callback 是成員函數，需要知道是「哪個物件」的

timer_ = this->create_wall_timer(
    500ms,
    std::bind(&MinimalPublisher::timer_callback, this)
    //        ↑ 函數                              ↑ 這個物件
);
```

---

## 10. 變數命名慣例：`_` 結尾

```cpp
class MyClass
{
private:
    int count_;        // 成員變數，用 _ 結尾
    std::string name_; // 成員變數，用 _ 結尾

public:
    void setCount(int count)
    {
        count_ = count;  // 這樣就不會跟參數 count 搞混
    }
};
```

**`_` 結尾不是指標的意思！只是命名習慣，表示「這是成員變數」**

```cpp
timer_      // 剛好是指標（SharedPtr）
publisher_  // 剛好是指標（SharedPtr）
count_      // 不是指標！只是一個 size_t 數字
```

---

## 11. ROS2 專用函數

這些都是 `rclcpp` 函式庫提供的：

```cpp
// 初始化 ROS2 系統
rclcpp::init(argc, argv);

// 讓 Node 持續運作（處理 timer、callback 等）
rclcpp::spin(node);

// 關閉 ROS2 系統
rclcpp::shutdown();
```

### `spin` 是什麼？

```cpp
rclcpp::spin(node);
```

白話：**讓 Node 不斷循環，等待事件發生**

- Timer 到時間 → 執行 timer_callback
- 收到訊息 → 執行 subscriber callback
- 沒有 spin，程式會直接結束

### Log 函數

**Log = 日誌 (Logging)，不是對數 (Logarithm)！**

```cpp
RCLCPP_INFO(this->get_logger(), "Hello %s", "World");   // 資訊
RCLCPP_WARN(this->get_logger(), "Warning!");            // 警告
RCLCPP_ERROR(this->get_logger(), "Error!");             // 錯誤
```

就像 `printf`，但會加上時間戳記和 Node 名稱。

---

## 總結

| 觀念 | 白話解釋 |
|------|----------|
| class | 藍圖，用來建立物件 |
| 成員變數 | 屬於 class 的資料 |
| 建構函數 | 建立物件時自動執行 |
| 初始化列表 `: xxx()` | 建構函數初始化的另一種寫法 |
| 繼承 `: public Parent` | 子類別獲得父類別的功能 |
| 指標 `*` | 儲存記憶體位址 |
| `->` | 用指標存取成員 |
| 智能指標 SharedPtr | 會自動清理的指標 |
| `this` | 指向自己的指標 |
| `::` | 命名空間的分隔符號 |
| `std::bind` | 把函數和物件綁在一起 |
| `_` 結尾 | 命名習慣，表示成員變數 |
