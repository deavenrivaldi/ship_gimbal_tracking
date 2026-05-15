# 拋射體模擬系統 (Projectile Simulation System)

本專案提供一個 ROS 2 系統，用於在 Gazebo 中生成並模擬拋射體。目前系統僅針對拋射體本身進行建模；若您認為有需要，我可以新增一個「虛擬大砲 (virtual canon)」來進行拋射體的生成。

## 運作原理

當系統接收到 `/fire` 服務請求時，會執行以下動作：
1. **生成 (Spawn)**：在 Gazebo 的指定位置生成一個球體模型。
2. **發射 (Launch)**：在指定的方向施加脈衝力，以模擬初始速度。

## 編譯
```bash
rm -rf build/projectile_bringup
colcon build --symlink-install --packages-select projectile_bringup
source install/setup.bash
```

### 啟動系統
請執行以下指令來啟動模擬環境：

```bash
ros2 launch projectile_bringup projectile.launch.py
```

### 依賴項目
請確保在您的 `package.xml` 中加入以下依賴：
```xml
<depend>projectile_msgs</depend>
```

---

## 使用方法

若要發射拋射體，請從您的 ROS 2 節點呼叫 `/fire` 服務。

### 範例程式碼 (Python)

```python
from projectile_msgs.srv import Fire

# 建立服務客戶端
client = self.create_client(Fire, '/fire')

# 準備請求內容
request = Fire.Request()
request.position.x = 0.0  # 生成位置
request.position.y = 0.0
request.position.z = 5.0
request.direction.x = 1.0  # 方向向量
request.direction.y = 0.0
request.direction.z = 0.0
request.velocity = 15.5    # 初始速度 (m/s)

# 非同步呼叫服務
future = client.call_async(request)
```

### 服務回傳值
* `success` (bool)：若拋射體成功生成則為 `True`。
* `projectile_name` (string)：已生成拋射體的唯一識別碼。

---

## 設定檔

模擬參數位於：
`projectile_bringup/config/projectile_params.yaml`

### YAML 設定範例
```yaml
gz_interface:
  ros__parameters:
    world_name: world_test  # Gazebo 世界名稱
    world_file: world_test.sdf
    force: 1000.0           # 脈衝力大小
```

*注意：您可以調整 `force`（力）的參數，但我尚不確定這對您的專案是否有實際幫助。*

# 系統合並名稱對應
launch -> launch
add .yaml to connect gimabl / shoot with py
world -> world ✅
models -> projectile_sphere.sdf ✅
control -> gz_interface.py
add msgs to define msg

# 移植改動
1. 新依賴套件
```
#在虛擬環境中安裝
pip install empy lark catkin_pkg setuptools pyyaml jinja2 typeguard lxml
```

# 發射指令
```
ros2 service call /fire ship_msgs/srv/Fire "{position: {x: 0.0, y: 0.0, z: 1.0}, direction: {x: 20.0, y: 0.0, z: 2.0}, force: 150.0}"
```