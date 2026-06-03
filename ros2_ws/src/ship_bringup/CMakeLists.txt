cmake_minimum_required(VERSION 3.8)
project(ship_bringup)

# 檢查編譯器環境
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 尋找必要的套件
find_package(ament_cmake REQUIRED)

# --- 安裝規則 (這正是解決您問題的關鍵) ---

# 1. 安裝 launch 檔案到 share 目錄
install(DIRECTORY
  ship_bringup
  DESTINATION share/${PROJECT_NAME}
)

# 2. 安裝 config 資料夾到 share 目錄
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# 結束配置
ament_package()