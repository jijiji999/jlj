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

## 6. 双踝并联关节可视化测试

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
