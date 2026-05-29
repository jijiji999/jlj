# dm_humanoid 使用说明

## 1. 包定位

`dm_humanoid` 是上层人形机器人控制包，和 `dm_motor` 并列。

推荐职责划分：

- `dm_motor`
  负责 CANFD、电机驱动、29 电机配置、底层状态与 MIT 指令发送

- `dm_humanoid`
  负责状态机、手柄切换、IMU 接入、RL 观测拼接、模式控制和后续舞蹈模式扩展

## 2. 当前节点

- `dm_humanoid_control_node`

## 3. 当前模式

- `fixed`
- `passive`
- `loco`
- `dance`

其中：

- `dance` 目前是占位入口
- `loco` 已完成观测构造和动作下发框架
- 真实 ONNX 推理后端仍需后续接入 `onnxruntime`

## 4. 配置文件

上层控制配置：

- [dm_humanoid_29.yaml](/home/sliouzhou04/jlj_ws/src/dm_humanoid/config/dm_humanoid_29.yaml)

底层电机配置：

- [dm_motor_29.yaml](/home/sliouzhou04/jlj_ws/src/dm_motor/config/dm_motor_29.yaml)

## 5. 启动方式

先启动：

- `dm_joy_node`
- `dm_imu_node`
- `CAN bridge`

再启动：

```bash
ros2 run dm_humanoid dm_humanoid_control_node --ros-args \
  -p config_path:=/home/sliouzhou04/jlj_ws/src/dm_humanoid/config/dm_humanoid_29.yaml
```

如果要显式指定底层电机配置：

```bash
ros2 run dm_humanoid dm_humanoid_control_node --ros-args \
  -p config_path:=/home/sliouzhou04/jlj_ws/src/dm_humanoid/config/dm_humanoid_29.yaml \
  -p motor_config_path:=/home/sliouzhou04/jlj_ws/src/dm_motor/config/dm_motor_29.yaml
```
