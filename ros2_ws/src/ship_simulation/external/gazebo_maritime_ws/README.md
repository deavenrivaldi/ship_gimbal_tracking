## 編譯 plguin
```
# 0. 進入資料夾 (從 ship_gimbal_tracking 開啟)
cd ros2_ws/src/ship_simulation/external/gazebo_maritime_ws/src/gazebo_maritime

# 1. 建立並進入編譯資料夾
mkdir build && cd build

# 2. 執行 CMake 配置 (這會檢查你是否安裝了 gz-sim8 等依賴)
cmake ..

# 3. 開始編譯 (使用所有 CPU 核心)
make -j$(nproc)

```
```
# 4. 搬遷 .so 檔 (編譯完成的 build 資料夾中)
cp libPublisherPlugin.so ../lib/libPublisherPlugin.so
cp libSurface.so ../lib/libSurface.so
cp libWaves.so ../lib/libWaves.so
cp libWaveVisual.so ../lib/libWaveVisual.so

```

## 說明
- build : 已編譯完成插件 (.so)
- src : 未編譯插件 (.c/.h)
> 此資料夾純存放檔案，build 與 src 無直接關聯

#### 可能優化
    1. 加入資料夾 CMake
    2. 撰寫 ros2 CMakeLists.txt ，在 clone 時編譯插件 (路徑會改，須修正 xxx_lunch.py)