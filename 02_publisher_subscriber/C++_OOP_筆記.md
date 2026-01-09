# C++ OOP (物件導向) 完整筆記

## OOP 三大核心概念

### 1. 封裝 (Encapsulation)

隱藏內部細節，只暴露需要的介面。

```cpp
class Temperature {
private:
    float secretValue;     // 私有：只有自己能用

protected:
    float calibration;     // 保護：自己 + 子類別能用

public:
    virtual float getTemp() = 0;  // 公開：誰都能用
};
```

**存取權限表：**

| 權限 | 自己 | 子類別 | 外部 |
|------|------|--------|------|
| `private` | ✅ | ❌ | ❌ |
| `protected` | ✅ | ✅ | ❌ |
| `public` | ✅ | ✅ | ✅ |

**白話：**
- private：我的秘密，只有我能看
- protected：家族秘密，子孫可以看
- public：公開資訊，誰都能看

---

### 2. 繼承 (Inheritance)

子類別繼承父類別的功能和規定。

```cpp
// 父類別：定義規定
class Temperature {
public:
    virtual float getTemp() = 0;  // 純虛函數 = 規定
};

// 子類別：繼承並實作
class AnalogSensor : public Temperature {
public:
    float getTemp() override {
        return 25.5;  // 實作方式由子類別決定
    }
};
```

**繼承語法：**
```cpp
class 子類別 : public 父類別 {
    // ...
};
```

---

### 3. 多型 (Polymorphism)

同一個介面，不同實作。

```cpp
// 不同感測器，同一個 getTemp() 介面
class AnalogSensor : public Temperature {
    float getTemp() override { return readAnalogPin(); }
};

class DigitalSensor : public Temperature {
    float getTemp() override { return readI2C(); }
};

class NetworkSensor : public Temperature {
    float getTemp() override { return fetchFromServer(); }
};

// 使用時：不管哪種，都可以用同一個介面
Temperature* sensor = new AnalogSensor();
float temp = sensor->getTemp();  // 自動呼叫 AnalogSensor 的版本
```

**多型的威力：**
```cpp
// 管理多種感測器
Temperature* sensors[3];
sensors[0] = new AnalogSensor();
sensors[1] = new DigitalSensor();
sensors[2] = new NetworkSensor();

// 同一個迴圈處理所有類型
for (int i = 0; i < 3; i++) {
    float temp = sensors[i]->getTemp();  // 自動呼叫對應版本
}
```

---

## 抽象類別 (Abstract Class)

含有 `= 0` 的純虛函數，不能直接建立物件。

```cpp
class Temperature {
public:
    virtual float getTemp() = 0;  // = 0 表示沒有實作
};

// ❌ 錯誤：不能 new 抽象類別
Temperature* t = new Temperature();

// ✅ 正確：要 new 子類別
Temperature* t = new AnalogSensor();
```

**用途：定義介面（規定），強制子類別實作。**

---

## 建構子 (Constructor)

物件「出生」時自動執行的程式碼。

```cpp
class Sensor {
    int pin;

public:
    // 建構子：跟 class 同名，沒有回傳值
    Sensor(int p) {
        pin = p;
        cout << "Sensor 出生，pin = " << pin << endl;
    }
};

// 使用
Sensor s(5);           // 呼叫建構子，印「Sensor 出生，pin = 5」
Sensor s = Sensor(5);  // 另一種寫法，同樣意思
```

**用途：初始化變數、開啟檔案、連接硬體**

---

## 解構子 (Destructor)

物件「死掉」時自動執行的程式碼。

```cpp
class Sensor {
    int* data;

public:
    Sensor() {
        data = new int[100];  // 建構時配置記憶體
        cout << "出生" << endl;
    }

    ~Sensor() {               // 解構子：~ + class 名稱
        delete[] data;        // 解構時釋放記憶體
        cout << "死掉" << endl;
    }
};

void test() {
    Sensor s;  // 印「出生」
}              // 函數結束，s 死掉，印「死掉」
```

**用途：釋放記憶體、關閉檔案、斷開連接**

**重要規則：**

| 情況 | 需要寫解構子？ |
|------|---------------|
| 用了 `new` | ✅ 要自己寫 `delete` |
| 開了檔案 | ✅ 要自己寫 `close` |
| 普通變數 | ❌ 不用寫 |

---

## this 指標

指向「自己這個物件」的指標。

```cpp
class Sensor {
    int value;  // 成員變數

public:
    void setValue(int value) {  // 參數也叫 value
        // 問題：兩個 value 怎麼分辨？

        this->value = value;
        // this->value = 成員變數（我自己的 value）
        // value = 參數（外面傳進來的）
    }
};
```

**白話：`this->value` = 「我自己的 value」**

---

## 完整範例

```cpp
#include <iostream>
using namespace std;

// 父類別：抽象類別，定義介面
class Temperature {
public:
    virtual float getTemp() = 0;  // 純虛函數
    virtual ~Temperature() {}     // 虛解構子（多型時需要）
};

// 子類別：實作介面
class AnalogSensor : public Temperature {
    int pin;
    float* buffer;

public:
    // 建構子
    AnalogSensor(int p) {
        pin = p;
        buffer = new float[100];
        cout << "AnalogSensor 建立，pin = " << pin << endl;
    }

    // 解構子
    ~AnalogSensor() {
        delete[] buffer;
        cout << "AnalogSensor 銷毀，pin = " << pin << endl;
    }

    // 覆寫 getTemp
    float getTemp() override {
        return 25.5;  // 模擬讀取溫度
    }
};

int main() {
    // 使用多型
    Temperature* sensor = new AnalogSensor(5);
    cout << "溫度: " << sensor->getTemp() << endl;
    delete sensor;  // 會呼叫解構子

    return 0;
}
```

輸出：
```
AnalogSensor 建立，pin = 5
溫度: 25.5
AnalogSensor 銷毀，pin = 5
```

---

## 總結

| 概念 | 說明 |
|------|------|
| 封裝 | private/protected/public 控制存取 |
| 繼承 | 子類別繼承父類別的規定和功能 |
| 多型 | 同一個介面，不同實作 |
| 抽象類別 | `= 0` 純虛函數，只能被繼承 |
| 建構子 | 物件出生時執行，初始化用 |
| 解構子 | 物件死掉時執行，清理用 |
| this | 指向自己的指標 |

---

## 參考資源

- [C++ OOP 教學](https://www.runoob.com/cplusplus/cpp-classes-objects.html)
- [Virtual Functions](https://www.geeksforgeeks.org/virtual-function-cpp/)
