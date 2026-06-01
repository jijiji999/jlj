# dm_motor 使用说明

## 1. 包简介

`dm_motor` 是一个基于 `SocketCAN + CAN FD` 的 ROS 2 电机驱动包，面向达妙电机的人形机器人底层控制场景。

当前包内已经具备以下能力：

- 单电机使能、失能、清错、保存零点
- MIT 模式控制单个电机
- MIT 模式批量控制多个电机
- 29 电机配置管理
- ROS 2 状态发布节点
- ROS 2 MIT 测试节点
- 本地 Web 可视化调试页面

当前默认配置文件：

- [config/dm_motor_29.yaml](/home/sliouzhou04/jlj_ws/src/dm_motor/config/dm_motor_29.yaml)

## 2. 目录说明

- [include/dm_motor/types.hpp](/home/sliouzhou04/jlj_ws/src/dm_motor/include/dm_motor/types.hpp)
  定义电机配置、MIT 命令、电机状态等基础数据结构

- [include/dm_motor/damiao_protocol.hpp](/home/sliouzhou04/jlj_ws/src/dm_motor/include/dm_motor/damiao_protocol.hpp)
  达妙协议打包 / 解包接口

- [include/dm_motor/socketcan_transport.hpp](/home/sliouzhou04/jlj_ws/src/dm_motor/include/dm_motor/socketcan_transport.hpp)
  SocketCAN FD 底层收发接口

- [include/dm_motor/dm_motor_driver.hpp](/home/sliouzhou04/jlj_ws/src/dm_motor/include/dm_motor/dm_motor_driver.hpp)
  单电机驱动类接口

- [include/dm_motor/dm_motor_manager.hpp](/home/sliouzhou04/jlj_ws/src/dm_motor/include/dm_motor/dm_motor_manager.hpp)
  多电机管理类接口

- [src/dm_motor_node.cpp](/home/sliouzhou04/jlj_ws/src/dm_motor/src/dm_motor_node.cpp)
  正式状态发布节点

- [src/dm_motor_test_node.cpp](/home/sliouzhou04/jlj_ws/src/dm_motor/src/dm_motor_test_node.cpp)
  MIT 测试节点

- [src/dm_motor_web_node.cpp](/home/sliouzhou04/jlj_ws/src/dm_motor/src/dm_motor_web_node.cpp)
  Web 可视化后端节点

- [web/index.html](/home/sliouzhou04/jlj_ws/src/dm_motor/web/index.html)
  可视化页面前端

## 3. 编译与环境加载

在工作空间根目录执行：

```bash
cd /home/sliouzhou04/jlj_ws
colcon build --packages-select dm_motor
source install/setup.bash
```

如果后续每次开新终端，都需要重新执行：

```bash
source /home/sliouzhou04/jlj_ws/install/setup.bash
```

## 4. CAN 桥接启动

如果你仍然沿用 `vcan + cannelloni + 以太网转 CANFD 模块` 方案，可以先启动桥接脚本：

```bash
bash /home/sliouzhou04/jlj_ws/install/dm_motor/share/dm_motor/scripts/start_can_bridge.sh 192.168.1.157
```

默认作用：

- 创建 `vcan0 ~ vcan3`
- 启动 `cannelloni`
- 将 4 路虚拟 CAN 映射到远端模块

关闭桥接：

```bash
pkill cannelloni
```

## 5. 电机配置说明

当前 29 电机配置文件：

- [dm_motor_29.yaml](/home/sliouzhou04/jlj_ws/src/dm_motor/config/dm_motor_29.yaml)

当前配置规则：

- 每条 `vcan` 总线内部独立编号
- 每条总线上的 `can_id` 都保持在 `0x01 ~ 0x0F` 以内
- `master_id = can_id + 0x10`
- MIT 参数范围按电机型号自动加载
- 每个关节还额外带有图片整理出的：
  `upper_position_limit`
  `lower_position_limit`
  `max_output_torque`
  `max_output_speed`

当前已经内建的型号 MIT 范围：

- `4310`
  `P:[-12.5, 12.5]`
  `V:[-30.0, 30.0]`
  `T:[-10.0, 10.0]`

- `4340`
  `P:[-12.5, 12.5]`
  `V:[-10.0, 10.0]`
  `T:[-28.0, 28.0]`

- `4340P`
  `P:[-12.5, 12.5]`
  `V:[-10.0, 10.0]`
  `T:[-28.0, 28.0]`

- `8009P`
  `P:[-12.5, 12.5]`
  `V:[-25.0, 25.0]`
  `T:[-30.0, 30.0]`

- `10010`
  `P:[-12.5, 12.5]`
  `V:[-12.0, 12.0]`
  `T:[-120.0, 120.0]`

统一增益范围：

- `Kp:[0.0, 500.0]`
- `Kd:[0.0, 5.0]`

当前默认 PD 参数：

- `4310`
  `Kp = 10`
  `Kd = 1`

- `4340`
  `Kp = 10`
  `Kd = 1`

- `4340P`
  `Kp = 10`
  `Kd = 1`

- `8009P`
  `Kp = 27`
  `Kd = 3.7`

- `10010`
  `Kp = 27`
  `Kd = 3.7`

总线分配如下：

- `vcan0`：`waist_yaw` + 左臂全部
- `vcan1`：`waist_parallel_1`、`waist_parallel_2` + 右臂全部
- `vcan2`：左腿全部
- `vcan3`：右腿全部

说明：

- 同一条 CAN 总线上，低 8 位 ID 不能重复
- 该电机反馈帧只带低 4 位 ID，因此同一条总线上低 4 位也不能重复
- 达妙说明书里明确建议 `CAN_ID < 16`
- 因为你有 4 条独立总线，所以不同总线之间可以复用同样的 `can_id`
- 测试节点和底层 MIT 发送前都会根据这些限制做裁剪，避免直接发出越界位置、超速或超扭矩命令

## 6. 正式开发可用节点

### 6.1 `dm_motor_node`

用途：

- 轮询电机反馈
- 发布 `joint_states`
- 适合接入正式系统、上层状态估计、可视化或控制框架

启动命令：

```bash
ros2 run dm_motor dm_motor_node --ros-args \
  -p config_path:=/home/sliouzhou04/jlj_ws/src/dm_motor/config/dm_motor_29.yaml \
  -p state_publish_hz:=100.0 \
  -p rx_poll_timeout_ms:=0
```

参数说明：

- `config_path`
  电机配置文件路径

- `state_publish_hz`
  `joint_states` 发布频率

- `rx_poll_timeout_ms`
  每次轮询反馈的超时时间

发布话题：

- `joint_states`

输出内容：

- `name`
- `position`
- `velocity`
- `effort`

## 7. 测试节点

### 7.1 `dm_motor_test_node`

用途：

- 快速验证 MIT 控制协议是否打通
- 单关节测试
- 多关节同参数测试

核心行为：

- 启动时可自动使能
- 定时发送 MIT 指令
- 退出时可自动失能

启动单电机测试：

```bash
ros2 run dm_motor dm_motor_test_node --ros-args \
  -p config_path:=/home/sliouzhou04/jlj_ws/src/dm_motor/config/dm_motor_29.yaml \
  -p target_motor:=left_elbow \
  -p command_rate_hz:=50.0 \
  -p command_timeout_ms:=20 \
  -p mit_position:=0.0 \
  -p mit_velocity:=1.0 \
  -p mit_kp:=-1.0 \
  -p mit_kd:=-1.0 \
  -p mit_torque:=0.0
```

启动全电机同参数测试：

```bash
ros2 run dm_motor dm_motor_test_node --ros-args \
  -p config_path:=/home/sliouzhou04/jlj_ws/src/dm_motor/config/dm_motor_29.yaml \
  -p target_motor:=all \
  -p command_rate_hz:=20.0 \
  -p mit_position:=0.0 \
  -p mit_velocity:=0.3 \
  -p mit_kp:=-1.0 \
  -p mit_kd:=-1.0 \
  -p mit_torque:=0.0
```

参数说明：

- `target_motor`
  具体关节名，或者 `all`

- `auto_enable_on_start`
  启动时是否自动使能

- `auto_disable_on_shutdown`
  退出时是否自动失能

- `command_rate_hz`
  MIT 指令发送频率

- `command_timeout_ms`
  每次命令等待反馈的超时时间

- `mit_position`
  MIT 目标位置

- `mit_velocity`
  MIT 目标速度

- `mit_kp`
  MIT 位置增益，设成负值时自动使用该电机型号默认值

- `mit_kd`
  MIT 微分增益，设成负值时自动使用该电机型号默认值

- `mit_torque`
  MIT 前馈力矩

适合的调试顺序：

1. 先启动 CAN 桥接
2. 先只测单电机
3. 先小速度、小力矩
4. 确认使能成功后，再逐步调大参数

## 8. 可视化调试页面

### 8.1 `dm_motor_web_node`

用途：

- 提供本地 HTTP 服务
- 提供可视化页面
- 对每个电机做单独的使能 / 失能 / MIT 滑块测试

启动命令：

```bash
ros2 run dm_motor dm_motor_web_node --ros-args \
  -p config_path:=/home/sliouzhou04/jlj_ws/src/dm_motor/config/dm_motor_29.yaml \
  -p http_bind_host:=127.0.0.1 \
  -p http_port:=8080 \
  -p command_timeout_ms:=20 \
  -p poll_interval_ms:=20
```

浏览器打开：

```text
http://127.0.0.1:8080
```

页面功能：

- 单电机 `Enable`
- 单电机 `Disable`
- `Clear Error`
- `Save Zero`
- MIT 五个参数滑块调节
- `Single Send`
  每次拖动参数后，点击一次只发送一次当前 MIT 指令
- `Continuous On/Off`
  开启后按照固定频率连续发送当前 MIT 指令，拖动滑块会实时影响后续发送内容
- `Reset Zero`
  将当前电机卡片上的五个 MIT 参数全部恢复到 0
- 页面顶部 `Continuous Send Rate`
  设置连续发送频率
- 页面顶部 `Stop All Continuous`
  一键停止所有电机的连续发送
- 实时查看状态、位置、速度、扭矩、温度

页面参数范围说明：

- 位置滑条优先使用每个关节的 `upper_position_limit / lower_position_limit`
- 速度滑条使用 `-max_output_speed ~ +max_output_speed`
- 扭矩滑条使用 `-max_output_torque ~ +max_output_torque`
- `kp / kd` 仍使用电机 MIT 模型范围

页面默认 PD 行为：

- `kp / kd` 默认直接显示对应型号的默认值
- 默认不可拖动
- 每个 `kp / kd` 条右侧都有一个小方框
- 灰色表示锁定，绿色表示已解锁可调
- 点击一次解锁后才可以拖动
- 再点击一次会恢复默认参数值，并重新锁定

页面刷新说明：

- 首次加载时会读取完整电机配置
- 之后页面默认每 `100ms` 高频刷新一次状态
- 如果仍感觉慢，可以优先检查底层反馈频率和 `poll_interval_ms`

建议：

- 第一次先单电机测试
- 页面默认本机访问，不建议直接开放到外网

## 9. 主要 C++ 类与函数

### 9.1 `DmMotorDriver`

文件：

- [dm_motor_driver.hpp](/home/sliouzhou04/jlj_ws/src/dm_motor/include/dm_motor/dm_motor_driver.hpp)

用途：

- 控制单个电机

常用函数：

- `enable()`
  使能单个电机

- `disable()`
  失能单个电机

- `clear_error()`
  清除错误

- `save_zero_position()`
  将当前位置保存为零点

- `send_mit_command(const MitCommand & command)`
  发送 MIT 控制命令

- `rotate_velocity(float velocity, float kd, float torque_ff)`
  用 MIT 方式做速度控制

- `read_register_float(uint8_t register_id)`
  读取浮点寄存器

- `read_register_u32(uint8_t register_id)`
  读取整型寄存器

- `write_register_float(uint8_t register_id, float value)`
  写浮点寄存器

- `write_register_u32(uint8_t register_id, uint32_t value)`
  写整型寄存器

- `switch_mode(MotorMode mode)`
  切换控制模式

- `last_state()`
  获取最近一次反馈状态

### 9.2 `DmMotorManager`

文件：

- [dm_motor_manager.hpp](/home/sliouzhou04/jlj_ws/src/dm_motor/include/dm_motor/dm_motor_manager.hpp)

用途：

- 多电机统一管理
- 批量控制
- 配置加载

常用函数：

- `enable_motor(name)`
  使能指定电机

- `disable_motor(name)`
  失能指定电机

- `clear_motor_error(name)`
  清错

- `save_motor_zero(name)`
  保存零点

- `command_motor_mit(name, command)`
  控制单个电机

- `enable_all()`
  全部电机使能

- `disable_all()`
  全部电机失能

- `command_group_mit(commands)`
  批量 MIT 控制

- `snapshot_states()`
  获取所有已收到反馈的电机状态快照

- `poll_once()`
  主动轮询一次底层反馈

- `motor_names()`
  获取所有电机名称

- `motor_configs()`
  获取全部电机配置

- `motor_config(name)`
  获取某个电机配置

### 9.3 `SocketCanTransport`

文件：

- [socketcan_transport.hpp](/home/sliouzhou04/jlj_ws/src/dm_motor/include/dm_motor/socketcan_transport.hpp)

用途：

- 底层 CAN FD 收发

常用函数：

- `send(const CanFrame & frame)`
- `receive(timeout)`
- `receive_available(timeout, max_frames)`

### 9.4 协议层

文件：

- [damiao_protocol.hpp](/home/sliouzhou04/jlj_ws/src/dm_motor/include/dm_motor/damiao_protocol.hpp)

用途：

- MIT 控制帧打包
- 反馈帧解包
- 寄存器读写帧打包

常用函数：

- `build_enable_frame()`
- `build_disable_frame()`
- `build_save_zero_frame()`
- `build_clear_error_frame()`
- `build_mit_frame()`
- `build_register_read_frame()`
- `build_register_write_u32_frame()`
- `build_register_write_float_frame()`
- `decode_feedback_frame()`

## 10. 典型二次开发方式

### 10.1 在 C++ 程序里直接调用管理器

```cpp
#include "dm_motor/dm_motor_manager.hpp"

int main()
{
  dm_motor::DmMotorManager manager(
    "/home/sliouzhou04/jlj_ws/src/dm_motor/config/dm_motor_29.yaml");

  manager.enable_motor("left_elbow");

  dm_motor::MitCommand command;
  command.position = 0.0f;
  command.velocity = 1.0f;
  command.kp = 0.0f;
  command.kd = 1.0f;
  command.torque = 0.0f;

  manager.command_motor_mit("left_elbow", command);
  manager.disable_motor("left_elbow");
  return 0;
}
```

### 10.2 在正式控制程序里常见用法

- 后台一直运行 `dm_motor_node` 发布 `joint_states`
- 上层控制器内部持有 `DmMotorManager`
- 控制周期内批量调用 `command_group_mit()`
- 机器人下电或急停时调用 `disable_all()`

## 11. 常用调试指令汇总

构建：

```bash
cd /home/sliouzhou04/jlj_ws
colcon build --packages-select dm_motor
```

加载环境：

```bash
source /home/sliouzhou04/jlj_ws/install/setup.bash
```

启动 CAN 桥：

```bash
bash /home/sliouzhou04/jlj_ws/install/dm_motor/share/dm_motor/scripts/start_can_bridge.sh 192.168.1.157
```

启动正式状态节点：

```bash
ros2 run dm_motor dm_motor_node --ros-args \
  -p config_path:=/home/sliouzhou04/jlj_ws/src/dm_motor/config/dm_motor_29.yaml
```

启动 MIT 测试节点：

```bash
ros2 run dm_motor dm_motor_test_node --ros-args \
  -p config_path:=/home/sliouzhou04/jlj_ws/src/dm_motor/config/dm_motor_29.yaml \
  -p target_motor:=left_elbow \
  -p mit_velocity:=1.0 \
  -p mit_kd:=1.0
```

启动 Web 可视化页面：

```bash
ros2 run dm_motor dm_motor_web_node --ros-args \
  -p config_path:=/home/sliouzhou04/jlj_ws/src/dm_motor/config/dm_motor_29.yaml \
  -p http_port:=8080
```

查看话题：

```bash
ros2 topic list
ros2 topic echo /joint_states
```

停止桥接：

```bash
pkill cannelloni
```

## 12. 安全建议

- 第一次只测一个关节
- MIT 参数先小后大
- 优先从 `velocity + kd` 的柔和控制开始
- 大关节尤其是 `10010` 电机，首次动作前确保机械结构固定可靠
- Web 页面和测试节点都只是调试工具，不建议直接当正式控制器

## 13. 推荐使用方式

如果是联调阶段，推荐顺序：

1. 启动 `start_can_bridge.sh`
2. 启动 `dm_motor_web_node`
3. 页面里先测试单电机使能 / 失能
4. 页面里小幅拖动 MIT 参数
5. 确认单电机正常后，再测试多关节

如果是正式开发阶段，推荐顺序：

1. 用 [dm_motor_29.yaml](/home/sliouzhou04/jlj_ws/src/dm_motor/config/dm_motor_29.yaml) 维护配置
2. 用 `dm_motor_node` 负责状态发布
3. 在你自己的控制程序里调用 `DmMotorManager`
4. 用 `command_group_mit()` 做周期控制
