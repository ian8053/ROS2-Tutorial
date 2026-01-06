# 01 - Nodes, Topics, Services

## 概念

### Node (節點)
- ROS2 的基本執行單元
- 一個程式可以包含多個 nodes
- 每個 node 負責特定功能

### Topic (話題)
- 發布/訂閱模式 (Pub/Sub)
- 非同步、一對多通訊
- 適合：感測器資料、持續性資料流

### Service (服務)
- 請求/回應模式 (Request/Response)
- 同步、一對一通訊
- 適合：觸發特定動作、查詢狀態

## 常用指令

```bash
# 列出所有節點
ros2 node list

# 查看節點資訊
ros2 node info /node_name

# 列出所有話題
ros2 topic list

# 查看話題內容
ros2 topic echo /topic_name

# 查看話題資訊
ros2 topic info /topic_name

# 列出所有服務
ros2 service list

# 呼叫服務
ros2 service call /service_name service_type "{request_data}"
```

## 練習

1. 啟動 turtlesim 並觀察 nodes 和 topics
2. 用 `ros2 topic pub` 發布訊息控制烏龜
3. 用 `ros2 service call` 重置烏龜位置

```bash
# 啟動 turtlesim
ros2 run turtlesim turtlesim_node

# 另一個終端：控制烏龜
ros2 run turtlesim turtle_teleop_key
```
