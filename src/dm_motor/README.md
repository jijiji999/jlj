# dm_motor

`dm_motor` 是一个面向达妙电机 CAN FD 控制的 ROS 2 底层工具包，参考了你已经验证通过的：

- `/home/sliouzhou04/enet/test/start_can.sh`
- `/home/sliouzhou04/enet/test/testcanfd.py`
- `DM-J4340-2EC V1.1` 说明书中的 MIT / 反馈 / 参数读写协议

当前包内提供了三层能力：

1. `SocketCAN FD` 传输层：直接走 Linux `socketcan` 原生接口，不依赖 `python-can`
2. 单电机驱动类：使能、失能、清错、保存零点、MIT 控制、寄存器读写
3. 多电机管理器：支持按名称控制多个电机，并保留 29 电机的人形机器人配置模板

## 目录说明

- `include/dm_motor/`
  - `socketcan_transport.hpp`：底层 CAN FD 收发
  - `damiao_protocol.hpp`：协议打包 / 解包
  - `dm_motor_driver.hpp`：单电机驱动接口
  - `dm_motor_manager.hpp`：多电机管理接口
  - `dm_motor_node.hpp`：ROS 2 节点封装
- `config/dm_motor_29.yaml`
  - 29 电机模板配置，按 4 路 `vcan` 分组
- `scripts/start_can_bridge.sh`
  - 基于你原来的 `start_can.sh` 做了参数化版本

## 使用前注意

说明书里有两个非常关键的协议特性，后续实机时一定要保留：

1. 电机接收控制帧时，只校验 CAN ID 的低 8 位
2. 反馈帧 `D[0]` 里只保留 ID 的低 4 位，其高 4 位被状态码占用

这意味着同一条 CAN 总线上：

- 低 8 位不能重复
- 低 4 位也最好不要重复
- 官方也建议 `CAN_ID < 16`

包内配置加载时已经按这个约束做了校验。

## MIT 映射参数

MIT 模式的 `position / velocity / torque` 打包需要和电机里实际生效的 `PMAX / VMAX / TMAX` 一致。

当前 29 电机配置中，`master_id` 已按 `can_id + 0x10` 完整填写。实机前如果驱动侧参数有改动，建议再做一次确认：

- `0x15` -> `PMAX`
- `0x16` -> `VMAX`
- `0x17` -> `TMAX`

## 典型 C++ 用法

```cpp
#include "dm_motor/dm_motor_manager.hpp"

int main()
{
  dm_motor::DmMotorManager manager("/absolute/path/to/dm_motor_29.yaml");

  manager.enable_motor("joint_01");

  dm_motor::MitCommand cmd;
  cmd.position = 0.0f;
  cmd.velocity = 2.0f;
  cmd.kp = 0.0f;
  cmd.kd = 1.0f;
  cmd.torque = 0.0f;

  manager.command_motor_mit("joint_01", cmd);
  manager.disable_motor("joint_01");
  return 0;
}
```

## ROS 2 节点

构建后可运行：

```bash
colcon build --packages-select dm_motor
source install/setup.bash
ros2 run dm_motor dm_motor_node --ros-args -p config_path:=/absolute/path/to/dm_motor_29.yaml
```

节点会周期性发布 `joint_states`，位置 / 速度 / 力矩来自最近一次收到的反馈帧。

## 测试节点

包内额外提供了一个 `dm_motor_test_node`，用于先做 MIT 模式的联调测试。

它支持两种方式：

- 单关节测试：设置 `target_motor:=left_elbow` 之类的具体关节名
- 全关节测试：设置 `target_motor:=all`

默认所有电机都使用同一组 MIT 参数，直接由 ROS 参数控制：

- `mit_position`
- `mit_velocity`
- `mit_kp`
- `mit_kd`
- `mit_torque`

单关节示例：

```bash
ros2 run dm_motor dm_motor_test_node --ros-args \
  -p config_path:=/absolute/path/to/dm_motor_29.yaml \
  -p target_motor:=left_elbow \
  -p mit_position:=0.0 \
  -p mit_velocity:=1.0 \
  -p mit_kp:=0.0 \
  -p mit_kd:=1.0 \
  -p mit_torque:=0.0
```

29 电机同参数测试示例：

```bash
ros2 run dm_motor dm_motor_test_node --ros-args \
  -p config_path:=/absolute/path/to/dm_motor_29.yaml \
  -p target_motor:=all \
  -p mit_position:=0.0 \
  -p mit_velocity:=0.5 \
  -p mit_kp:=0.0 \
  -p mit_kd:=1.0 \
  -p mit_torque:=0.0
```

安全建议：

- 首次联调建议从单关节开始
- `mit_position`、`mit_velocity`、`mit_torque` 先给很小
- 先确认 `enable` 成功再逐步加大参数
- 停止节点时会自动对目标电机发送 `disable`

## 可视化页面

包内提供了一个本地 Web 调试面板后端 `dm_motor_web_node`，配合静态页面可以直观测试：

- 单电机使能 / 失能
- 清错
- 保存零点
- MIT 五个参数滑块拖动发送
- 电机反馈状态、位置、速度、扭矩、温度查看

启动方式：

```bash
source /home/sliouzhou04/jlj_ws/install/setup.bash
ros2 run dm_motor dm_motor_web_node --ros-args \
  -p config_path:=/home/sliouzhou04/jlj_ws/src/dm_motor/config/dm_motor_29.yaml \
  -p http_bind_host:=127.0.0.1 \
  -p http_port:=8080
```

然后在本机浏览器打开：

```text
http://127.0.0.1:8080
```

说明：

- 页面默认每 500ms 刷新一次电机状态
- 拖动滑块时如果打开了 `Auto send MIT while dragging`，会自动发送 MIT 指令
- 首次联调建议先只操作单电机，再逐步测试多个关节
- Web 面板默认只监听 `127.0.0.1`，避免调试控制误暴露

## 启动 SocketCAN / Cannelloni

如果你仍然沿用现在这套以太网转 CAN FD 模块链路，可以直接参考：

```bash
bash install/dm_motor/share/dm_motor/scripts/start_can_bridge.sh 192.168.1.157
```
