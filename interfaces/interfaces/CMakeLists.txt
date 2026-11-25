cmake_minimum_required(VERSION 3.8)
project(interfaces)

# 接口包本身不编译 C/C++ 代码，这里统一定义 C++ 标准，供下游包参考
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ============================================================================
# 标准消息包依赖
# 新增自定义接口时：若用到了新的标准消息包（如 diagnostic_msgs、std_srvs、
# trajectory_msgs、action_msgs ...），在这里加 find_package，并同步在
# package.xml 里加 <depend>，同时把包名追加到下面的 INTERFACE_DEPENDENCIES。
# ============================================================================
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ============================================================================
# ★★★ 自定义接口文件登记处 ★★★
# 新增 msg / srv / action 时，只改下面这一处（文件路径用相对本包的写法）：
#   set(INTERFACE_FILES
#     "msg/Example.msg"
#     "srv/Example.srv"
#     "action/Example.action"
#   )
# 漏登记 = 文件存在但不会被生成，下游 ros2 interface show 一定找不到。
# 详见仓库根目录 readme.md「新增自定义话题/接口的标准流程」。
# ============================================================================
set(INTERFACE_FILES
  # 目前为空：全部通信都使用 ROS2 自带消息，见 readme.md「当前策略」
)

# rosidl 生成消息时需要的依赖包
set(INTERFACE_DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  builtin_interfaces
)

if(INTERFACE_FILES)
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${INTERFACE_FILES}
    DEPENDENCIES ${INTERFACE_DEPENDENCIES}
  )
else()
  message(STATUS "interfaces: 暂无自定义 msg/srv/action（当前只用 ROS2 自带消息，本包为接口层占位）")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

ament_package()
