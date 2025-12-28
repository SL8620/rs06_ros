# rs06_ros 仓库说明

## 仓库概述

`rs06_ros` 是一个基于 Linux SocketCAN 和 ROS1/catkin 的 RS06 电机测试与驱动示例工程，主要实现：

- 一个通用的 SocketCanDriver 封装类（收发线程 + 回调）。
- 一个 RS06 电机协议封装类（打包/解包 RS06 专用 CAN 报文）。
- 两个简单的测试程序：
  - test_socketcan：验证 CAN 通讯收发是否正常。
  - test_motor：交互式控制 RS06 电机（使能/失能/位置模式/位置命令）。

本仓库既可以独立使用（直接 cmake + make），也可以作为 ROS1 工作空间中的一个 catkin 包集成。

## 目录结构

- [CMakeLists.txt](CMakeLists.txt)：catkin 风格的构建配置，生成测试可执行文件。
- [package.xml](package.xml)：ROS1 catkin 包描述文件。
- [include/rs06_ros/rs06.hpp](include/rs06_ros/rs06.hpp)：RS06 电机协议封装类头文件。
- [include/rs06_ros/socketcan_driver.hpp](include/rs06_ros/socketcan_driver.hpp)：SocketCanDriver 头文件。
- [src/rs06.cpp](src/rs06.cpp)：RS06 类实现。
- [src/socketcan_driver.cpp](src/socketcan_driver.cpp)：SocketCanDriver 实现。
- [src/test_motor.cpp](src/test_motor.cpp)：电机交互测试程序。
- [src/test_socketcan.cpp](src/test_socketcan.cpp)：SocketCAN 收发测试程序。

> 说明：根目录下的 build/ 为本地构建目录，可直接通过 CMake 使用；在 catkin 工作空间中通常由 catkin_make 自动生成。

## 模块与类说明

### SocketCanDriver

位置：
- 头文件：[include/rs06_ros/socketcan_driver.hpp](include/rs06_ros/socketcan_driver.hpp)
- 实现文件：[src/socketcan_driver.cpp](src/socketcan_driver.cpp)

功能：
- 封装 Linux 原生 SocketCAN 接口（PF_CAN / SOCK_RAW / CAN_RAW）。
- 管理收发线程：
  - 接收线程：poll + 非阻塞 read，收到帧后调用用户注册的回调函数。
  - 发送线程：内部队列 + 条件变量，异步 send CAN 帧。

关键接口：
- `SocketCanDriver(const std::string& interface_name, CanFrameCallback recv_callback)`
  - interface_name：如 `"can0"`、`"can1"`。
  - recv_callback：`void(const struct can_frame&)` 类型的回调，用于上层处理收到的帧。
- `bool start()` / `void stop()`
  - 打开/关闭 CAN 设备，启动/停止收发线程。
- `bool send(const struct can_frame& frame)`
  - 将帧压入发送队列，由发送线程实际写入 socket。

### RS06 电机类

位置：
- 头文件：[include/rs06_ros/rs06.hpp](include/rs06_ros/rs06.hpp)
- 实现文件：[src/rs06.cpp](src/rs06.cpp)

功能：
- 针对单个 RS06 电机的 CAN 协议封装：
  - 发送控制指令（使能、失能、设零点、切换到位置模式、设置目标速度/位置）。
  - 发送位置查询报文，解析 RS06 返回的位置反馈。
- 内部管理电机状态：
  - `enabled_`：是否已发送使能。
  - `position_mode_enabled_`：是否已成功切换到位置模式。
  - 安全位置/速度范围：`p_safe_min_/max_`、`v_safe_min_/max_`。
  - 最近一次反馈位置：`last_position_rad_`、`has_position_`。

核心接口（对上层使用者）：
- 构造函数：
  - `RS06(SocketCanDriver& can_driver, uint8_t id, float p_safe_min, float p_safe_max, float v_safe_min, float v_safe_max)`
  - 通过引用注入一个已创建的 SocketCanDriver，以及电机 CAN ID 和可选的安全限位。
- 控制函数：
  - `bool enable()`：发送使能指令，成功后内部标记 enabled_。 
  - `bool disable()`：发送失能指令，成功后内部清除 enabled_。析构时会自动调用一次。
  - `bool setZero()`：设置当前角度为零点（若已使能则拒绝操作，提示需先失能）。
  - `bool setPositionMode()`：发送位置模式配置报文，成功后标记 position_mode_enabled_。
  - `bool setPosition(double position_rad, double velocity_rad_s)`：
    - 要求已先调用 `setPositionMode()`；
    - 参数位置单位 rad，速度单位 rad/s；
    - 内部会先根据物理范围 (0, 50) 和安全范围 [v_safe_min_, v_safe_max_] 对速度限幅；
    - 位置按 [p_safe_min_, p_safe_max_] 进行限幅；
    - 按协议发送两条 index 写入报文（最大速度 0x7017、目标位置 0x7016）。
  - `bool requestPosition()`：发送位置查询报文（index=0x7019），等待 RS06 返回位置反馈。
- 反馈处理与查询：
  - `void handleCanFrame(const struct can_frame& frame)`：
    - 在 SocketCanDriver 的回调中调用；
    - 根据电机 ID 和 index 判断是否为本电机的位置反馈，并解析存储。
  - `bool hasPosition() const`：是否已收到过有效位置反馈。
  - `double position() const`：返回最近一次解析到的位置（rad）。

内部打包/解包函数：
- `packEnable / packDisable / packSetZero`
  - 按 RS06 扩展帧协议构造 CAN ID 和数据段，DLC 固定 8 字节，未用数据补 0。
- `packIndexedCommand`
  - 通用 index+data 写入报文打包：ID mode=0x12，Byte0~1 为 index 小端，Byte4~7 为数据（最多 4 字节）。
- `packSetPositionMode / packRequestPosition`
  - 分别封装位置模式指令、位置查询指令。
- `unpackFeedback`
  - 对 index=0x7019 的位置反馈进行解析，数据区 float4 → double 存储。

## 测试程序说明

### test_socketcan

位置：
- [src/test_socketcan.cpp](src/test_socketcan.cpp)

功能：
- 验证 SocketCanDriver 收发是否正常：
  - 创建 SocketCanDriver，绑定到指定 CAN 接口（默认 `can1`）。
  - 注册回调函数，打印所有收到的 CAN 帧（ID、DLC、Data 字节）。
  - 循环发送一定数量的测试帧（ID 变化，数据递增），观察总线是否有回环或对端回应。
  - 支持 Ctrl+C 优雅退出。

运行示例：

1. 确保 CAN 口已 up（按需要修改接口和波特率）：
   - `sudo ip link set can1 up type can bitrate 1000000`

2. 在仓库 build 目录中运行：
   - `cd /home/nvidia/codeSpace/rs06_ros/build`
   - `./devel/lib/rs06_ros/test_socketcan`

### test_motor

位置：
- [src/test_motor.cpp](src/test_motor.cpp)

功能：
- 与一台 RS06 电机进行交互式测试（示例使用电机 ID=101）：
  - 后台线程周期性发送位置查询报文，并由 RS06::handleCanFrame 解包；
  - 主线程循环读取键盘命令：
    - `e`：使能电机；
    - `d`：失能电机；
    - `s`：切换到位置模式后，提示输入目标速度和目标位置并发送位置指令；
    - `q`：退出程序；
  - 每次等待输入前，打印一次当前最新位置 `Current position: X rad`。

典型使用流程：

1. 确保 CAN 口 up，并与 RS06 正常连接：
   - `sudo ip link set can1 up type can bitrate 1000000`

2. 在 build 目录运行：
   - `cd /home/nvidia/codeSpace/rs06_ros/build`
   - `./devel/lib/rs06_ros/test_motor`

3. 交互示例：
   - 程序启动后显示当前位置与命令菜单。
   - 输入 `e` 回车：发送使能；
   - 输入 `s` 回车：切换到位置模式，并按提示输入：
     - 目标速度（rad/s，建议 0~5 之间）；
     - 目标位置（rad，在安全范围 -3.14 ~ 3.14 内）；
   - 随时按 `d` 失能，`q` 退出；
   - 也可用 Ctrl+C 中断，程序会触发信号处理并优雅退出。

## 编译与使用

### 作为独立工程使用

仓库已经自带 CMakeLists.txt，可直接使用 CMake 构建（无需 catkin 工作空间）：

```bash
cd /home/nvidia/codeSpace/rs06_ros
mkdir -p build
cd build
cmake ..
make -j4
```

生成的可执行文件位于：

- `build/devel/lib/rs06_ros/test_socketcan`
- `build/devel/lib/rs06_ros/test_motor`

### 集成到 ROS1 catkin 工作空间（可选）

1. 将整个仓库放入 catkin 工作空间 src 目录，例如：

```bash
mkdir -p ~/catkin_ws/src
cp -r /home/nvidia/codeSpace/rs06_ros ~/catkin_ws/src/
```

2. 在 catkin 工作空间根目录编译：

```bash
cd ~/catkin_ws
catkin_make
```

3. 配置环境并通过 rosrun 运行：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun rs06_ros test_socketcan
rosrun rs06_ros test_motor
```

## 后续扩展建议

- 在 RS06 类上增加更多 index 读写封装，如电流、状态字、错误码等；
- 将 RS06 控制封装成 ROS 节点（订阅目标位置/速度话题，发布当前状态话题）；
- 根据实际工程需要，增加参数配置（例如在 ROS 参数服务器中配置 CAN 接口名、电机 ID、安全范围等）。
