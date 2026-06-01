# dm_test

`dm_test` 是一个面向机器人分部位轨迹跟踪测试的 ROS 2 包。

当前第一版已经提供：

- 左臂测试
- 右臂测试
- 左腿 / 右腿 / 腰部 / 整机分组预留
- 基于当前姿态的小幅相对轨迹跟踪
- 外部 CSV 轨迹跟踪
- 可调轨迹播放速度
- 参考轨迹、实际反馈、跟踪误差发布

## 1. 编译

```bash
cd /home/sliouzhou04/jlj_ws
colcon build --packages-select dm_test
source install/setup.bash
```

## 2. 启动左臂测试

```bash
ros2 launch dm_test arm_trajectory_test.launch.py trajectory_name:=left_arm_small_range
```

## 3. 启动右臂测试

```bash
ros2 launch dm_test arm_trajectory_test.launch.py trajectory_name:=right_arm_small_range
```

## 4. 关键话题

- `dm_test/reference_joint_states`
  参考轨迹
- `dm_test/actual_joint_states`
  实际反馈
- `dm_test/error_joint_states`
  跟踪误差

## 5. 用 CSV 跑左臂轨迹

如果你已经有一个左臂 CSV 文件，可以直接传给 launch：

```bash
export ROS_LOG_DIR=/tmp/roslog_dm_test
ros2 launch dm_test arm_trajectory_test.launch.py \
  trajectory_name:=left_arm_small_range \
  trajectory_csv_path:=/你的/csv/left_arm_swing.csv \
  playback_speed:=0.1 \
  command_rate_hz:=100.0
```

说明：

- `trajectory_csv_path`
  你的轨迹 CSV 文件路径
- `playback_speed`
  轨迹播放速度比例，`1.0` 是原始速度，`0.1` 是十分之一速度，适合一开始慢速跟随
- `command_rate_hz`
  机器人控制指令发送频率，建议先保持较高，比如 `50` 或 `100`

推荐一开始先这样理解：

- `command_rate_hz` 控制“下发命令有多快”
- `playback_speed` 控制“轨迹本身播放有多慢”

也就是说，你完全可以让机器人用 `100 Hz` 稳定收命令，同时把轨迹按 `0.1x` 慢慢播放。

如果你的 CSV 没有时间列，还可以加：

```bash
csv_row_rate_hz:=30.0
```

这表示 CSV 每一行相当于原始轨迹的 `1/30` 秒。

## 6. CSV 格式要求

默认测试配置文件：

- `config/dm_test_trajectories.yaml`

当前左臂 CSV 至少要包含这些列：

- `time`
- `left_shoulder_pitch`
- `left_shoulder_roll`
- `left_shoulder_yaw`
- `left_elbow`
- `left_wrist_roll`
- `left_wrist_pitch`
- `left_wrist_yaw`

这些值当前按“绝对关节位置（弧度）”处理。

如果时间列名字不是 `time`，可以传：

```bash
time_column_name:=timestamp
```

## 7. 轨迹配置说明

配置思路：

- `groups`
  定义测试对象，例如 `left_arm`、`right_arm`
- `trajectories`
  定义具体轨迹
- `offsets`
  表示相对于“启动时当前姿态”的小幅偏移量，单位是弧度

- `motor_config_path`
  支持 `package://dm_motor/config/dm_motor_29.yaml` 这种包内路径写法，安装后启动也能正常找到配置

- `csv_columns`
  可以给每个关节配置 CSV 列名映射，方便接不同来源的轨迹文件

后续如果你要扩展左腿、右腿、腰部和整机测试，优先直接在这个 YAML 里追加轨迹即可。
