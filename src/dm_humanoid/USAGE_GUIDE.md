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
- `ankle_parallel_web_node`

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

现在推荐这样修改 `fixed / passive` 的参数：

- 改全局默认：`controller.fixed_mode`、`controller.passive_mode`
- 改同类关节的一组参数：`controller.gain_profiles`
- 改单个关节：`joints[].fixed_kp / fixed_kd / passive_kp / passive_kd`

其中：

- `gain_profiles` 是更便于维护的分组增益配置
- `joints[].gain_profile` 用来把关节挂到某个分组
- `fixed_mode.transition_duration_sec` 控制切到 `fixed` 时多快到达目标位姿

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

## 6. fixed / passive 切换测试

如果你现在只想验证状态机的 `fixed` 和 `passive` 切换，可以直接启动一个限制模式测试：

```bash
ros2 launch dm_humanoid fixed_passive_test.launch.py
```

这个 launch 现在默认会同时启动：

- `dm_joy_node`
- `dm_joy_monitor_node`
- `dm_humanoid_control_node`

并默认做两件事：

- 启动模式为 `passive`
- 只允许 `fixed` 和 `passive`

也就是说：

- 即使没有 IMU，也可以先做这个测试
- `loco` 和 `dance` 的请求会被忽略
- 手柄链路也会一并启动，不需要再单独开一个 `dm_joy` 终端

如果你只想起 `dm_humanoid`，不想让这个 launch 自动起手柄节点：

```bash
ros2 launch dm_humanoid fixed_passive_test.launch.py start_joy:=false
```

如果你想关闭手柄监视器输出：

```bash
ros2 launch dm_humanoid fixed_passive_test.launch.py use_joy_monitor:=false
```

如果你要显式覆盖手柄路径：

```bash
ros2 launch dm_humanoid fixed_passive_test.launch.py \
  joy_device_path:=/dev/input/by-id/usb-S_TGZ_Controller_3E529690-joystick
```

### 6.1 用手柄切换

当前已经改成组合键确认模式：

- `A + L1` -> `fixed`
- `A + L2` -> `passive`
- `A + R1` -> `loco`
- `A + R2` -> `dance`

其中：

- `A` 是确认键
- `L2 / R2` 来自手柄扳机轴，不是普通按钮
- 当前 `dance` 仍然只是预留模式入口

对应配置在：

- [dm_humanoid_29.yaml](/home/sliouzhou04/jlj_ws/src/dm_humanoid/config/dm_humanoid_29.yaml:15)

如果后面发现 `L2 / R2` 触发方向和当前手柄不一致，可以调整：

- `passive_modifier_axis_sign`
- `dance_modifier_axis_sign`
- `trigger_activation_threshold`

### 6.2 用命令行切换

如果你想不依赖手柄，直接从终端切换：

切到 `fixed`：

```bash
ros2 topic pub --once /humanoid_control/mode_command std_msgs/msg/String "{data: fixed}"
```

切到 `passive`：

```bash
ros2 topic pub --once /humanoid_control/mode_command std_msgs/msg/String "{data: passive}"
```

### 6.3 怎么判断是否切换成功

可以从三个地方看：

1. 节点日志会打印：
   `Switched humanoid mode to fixed`
   或
   `Switched humanoid mode to passive`
2. 话题：

```bash
ros2 topic echo /humanoid_control/mode
```

3. 实机表现：

- `fixed`
  关节会回到设定初始位姿，并明显变硬
  切换到 `fixed` 时会按 `fixed_mode.transition_duration_sec` 设定的时间平滑靠近目标

- `passive`
  关节保持很软，可以手动掰动

补充：

- 在 `fixed / passive` 模式下，三个并联关节不会走并联解算
- 在 `loco / dance` 模式下，才会重新启用并联关节的逻辑关节解算与命令映射

### 6.4 常用启动参数

如果你要显式指定底层电机配置并从 `fixed` 启动：

```bash
ros2 launch dm_humanoid fixed_passive_test.launch.py \
  motor_config_path:=/home/sliouzhou04/jlj_ws/src/dm_motor/config/dm_motor_29.yaml \
  startup_mode:=fixed
```

## 7. 双踝并联关节可视化测试

用于实机验证左右腿踝关节并联机构解算。页面中左右各有一组控制卡片：

- 左侧：`left_ankle_pitch` / `left_ankle_roll`
- 右侧：`right_ankle_pitch` / `right_ankle_roll`

后端会分别把逻辑角度解算成：

- `left_ankle_parallel_1`
- `left_ankle_parallel_2`
- `right_ankle_parallel_1`
- `right_ankle_parallel_2`

先启动 CAN bridge，例如：

```bash
bash /home/sliouzhou04/jlj_ws/install/dm_motor/share/dm_motor/scripts/start_can_bridge.sh 192.168.1.157
```

再启动测试节点：

```bash
ros2 run dm_humanoid ankle_parallel_web_node --ros-args \
  -p motor_config_path:=/home/sliouzhou04/jlj_ws/src/dm_motor/config/dm_motor_29.yaml \
  -p left_solver_config_path:=/home/sliouzhou04/jlj_ws/src/dm_humanoid/config/parallel/left_ankle.yaml \
  -p right_solver_config_path:=/home/sliouzhou04/jlj_ws/src/dm_humanoid/config/parallel/right_ankle.yaml \
  -p http_bind_host:=0.0.0.0 \
  -p http_port:=8091
```

浏览器打开：

```text
http://<你的主机IP>:8091
```

实机验证建议顺序：

1. 先点击 `Enable Pair`
2. 每一侧先在页面里二选一：
   - `逆解模式`
   - `正解模式`
3. 逆解模式下：
   - 手动拖动 `pitch / roll`
   - 点一次 `解算`
   - 页面会算出对应电机角度
   - 确认结果后再点 `发送`
4. 正解模式下：
   - 手动拖动 `Motor 1 Angle / Motor 2 Angle`
   - 点一次 `解算`
   - 页面会算出对应虚拟关节 `pitch / roll`
   - 此时 `pitch / roll` 条会跟着变化，但不允许手动拖动
   - 确认结果后再点 `发送`
5. 对照页面解算结果和实机表现，判断正解/逆解是否正确
6. 测试结束后点击 `Disable Pair`

补充说明：

- 如果之前经常出现“解算失败”，主要原因通常不是页面，而是正解数值迭代不稳定
- 当前版本已经把正解内部改成了更稳的数值差分 Jacobian 迭代
- 建议仍然从零位附近开始小步验证，再逐渐拉大范围
