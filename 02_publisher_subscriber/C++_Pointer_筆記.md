# C++ 指標 (Pointer) 完整筆記

## 什麼是指標？

指標是用來**儲存記憶體位址**的變數。透過指標可以：
- 配置記憶體
- 存值 / 取值
- 釋放記憶體

---

## 1. 宣告

```cpp
type * name;           // 宣告指標
type * name = address; // 宣告並初始化
```

**關鍵：有 `*` 就是指標**

```cpp
int a;
a = 8;

int *p;      // p 是指標
p = &a;      // p 存放 a 的記憶體位址

*p = 10;     // 透過指標修改 a 的值，現在 a = 10

int * q = &a;  // 空格分開比較清楚
*q = 20;       // a = 20, *p = 20
```

**記憶體示意圖：**
```
變數    值        位址
─────────────────────────
a       20       0x123
p       0x123    (指向 a)
q       0x123    (指向 a)
```

**多個指標宣告：**
```cpp
type * nameA, * nameB;  // 兩個都是指標
```

---

## 2. 指派

### 存取指標本身
```cpp
name = address;  // 把記憶體位址存到指標
```

### 存取指標管理的記憶體
```cpp
*name = value;   // 把值存到指標指向的記憶體
```

---

## 3. 取值

### 取得指標儲存的位址
```cpp
name   // 回傳記憶體位址 (如 0x123)
```

### 取得指標指向的值
```cpp
*name  // 回傳該記憶體位址存的值
```

---

## 4. 取得記憶體位址

### 方法一：& 運算子
```cpp
int a = 5;
int *p = &a;  // 取得 a 的位址

functionA(&a);  // Pass by Pointer：讓函數可以修改 a
```

### 方法二：new 配置
```cpp
int * r = new int;  // 配置一個 int 大小的記憶體
*r = 100;           // 存值
```

### 方法三：從另一個指標
```cpp
int * s = r;  // s 和 r 指向同一個位址
*s = 200;     // *r 也變成 200
```

**函數呼叫範例：**
```cpp
functionA(new int);  // 傳入新配置的記憶體
functionA(r);        // 傳入指標
functionA(&a);       // 傳入變數的位址
```

---

## 5. delete 釋放記憶體

```cpp
int * r = new int;
*r = 100;
delete r;  // 釋放記憶體
// r 還存著位址值，但那個記憶體已經不能用了
```

**重要：用 `new` 配置的記憶體，要用 `delete` 釋放**

---

## 6. const 與指標

### const 在 * 後面：指標本身是常數
```cpp
int * const m = new int;
*m = 100;    // OK：可以改值
*m = 200;    // OK：可以改值
m = new int; // ERROR：m 是 read-only，不能改指向
```

### const 在 * 前面：指向的值是常數
```cpp
const int * n = new int;
*n = 100;    // ERROR：不能透過 n 改值

const int * n = m;  // OK：n 指向 m 指向的位址
// 只能讀取，不能修改
```

### 問題解答：`const int *` vs `int * const` 差異

| 宣告 | 可以改指標指向？ | 可以改指向的值？ |
|------|-----------------|-----------------|
| `int * const m` | ❌ 不行 | ✅ 可以 |
| `const int * n` | ✅ 可以 | ❌ 不行 |
| `const int * const p` | ❌ 不行 | ❌ 不行 |

**記憶口訣：const 靠近誰，誰就不能改**
- `int * const m`：const 靠近 m → m（指標）不能改
- `const int * n`：const 靠近 int → int（值）不能改

---

## 7. 指標參數 (Pass by Pointer)

```cpp
void modify(int *p) {
    *p = 999;  // 修改原本的變數
}

int main() {
    int a = 10;
    modify(&a);  // 傳入 a 的位址
    // 現在 a = 999
}
```

**比較三種傳遞方式：**
| 方式 | 語法 | 能改原值？ |
|------|------|-----------|
| Pass by Value | `func(int x)` | ❌ |
| Pass by Pointer | `func(int *x)` | ✅ |
| Pass by Reference | `func(int &x)` | ✅ |

---

## 8. 回傳指標

```cpp
int* createInt() {
    int* p = new int;
    *p = 42;
    return p;  // OK：回傳 new 配置的記憶體
}

// 錯誤示範
int* badFunc() {
    int local = 10;
    return &local;  // ERROR：區域變數離開函數就消失了
}
```

**可以宣告為 const：**
```cpp
const int* getData();  // 回傳的指標指向的值不能改
```

---

## 9. 動態陣列

```cpp
int size = 5;
int* arr = new int[size];  // 配置陣列

arr[0] = 10;
arr[1] = 20;
// ...

delete[] arr;  // 用 delete[] 釋放陣列
```

---

## 10. 陣列參數

**兩種寫法等價：**
```cpp
void func(int arr[]);   // 傳統寫法
void func(int *arr);    // 指標寫法
```

---

## 11. 指標的指標

```cpp
int a = 5;
int *p = &a;     // p 指向 a
int **pp = &p;   // pp 指向 p

**pp = 100;      // a = 100
```

**記憶體示意：**
```
a   = 5      (位址 0x100)
p   = 0x100  (位址 0x200)
pp  = 0x200  (位址 0x300)
```

---

## 12. void 指標

```cpp
void *vp;       // 可以指向任何型別
int a = 10;
vp = &a;        // OK

// 使用時要 casting
int *ip = (int*)vp;
cout << *ip;    // 10
```

---

## 13. 指標陣列

```cpp
int a = 1, b = 2, c = 3;
int* arr[3];     // 宣告：3 個指標的陣列

arr[0] = &a;
arr[1] = &b;
arr[2] = &c;

cout << *arr[0]; // 1
cout << *arr[1]; // 2
```

---

## 總結：指標操作一覽

| 操作 | 語法 | 說明 |
|------|------|------|
| 宣告 | `int *p;` | 宣告指標 |
| 取位址 | `&a` | 取得變數的位址 |
| 解參考 | `*p` | 取得指標指向的值 |
| 配置 | `new int` | 動態配置記憶體 |
| 釋放 | `delete p` | 釋放記憶體 |
| 陣列配置 | `new int[n]` | 配置陣列 |
| 陣列釋放 | `delete[] p` | 釋放陣列 |

---

## ROS2 中的指標應用

在 ROS2 中，指標常用於：

1. **智慧指標** (推薦)：
```cpp
std::shared_ptr<rclcpp::Node> node =
    std::make_shared<rclcpp::Node>("my_node");
```

2. **訊息指標**：
```cpp
void callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
}
```

ROS2 大量使用 `shared_ptr` 和 `unique_ptr`，比原始指標更安全，會自動管理記憶體。
