# dm_test

`dm_test` 是一个面向机器人分部位轨迹跟踪测试的 ROS 2 包。

当前第一版已经提供：

- 左臂测试
- 右臂测试
- 左腿 / 右腿 / 腰部 / 整机分组预留
- 基于当前姿态的小幅相对轨迹跟踪
- 外部 CSV 轨迹跟踪
- 全身逻辑关节轨迹到三组并联机构电机命令的自动解算
- 可调轨迹播放速度
- 参考轨迹、实际反馈、跟踪误差发布

## 0. 外部资源位置

当前机器人模型和动作文件统一放在：

- MJCF: `/home/sliouzhou04/jlj/JLJBot_mjcf/JLJBot/JLJBot.xml`
- MJCF scene: `/home/sliouzhou04/jlj/JLJBot_mjcf/JLJBot/scene_29dof.xml`
- URDF: `/home/sliouzhou04/jlj/JLJBot/JLJBot/JLJBot.urdf`
- 动作 CSV/PKL: `/home/sliouzhou04/jlj/pkl`

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

## 4. 启动基于 MJCF 的右臂摆臂测试

```bash
export ROS_LOG_DIR=/tmp/roslog_dm_test
ros2 launch dm_test arm_trajectory_test.launch.py \
  trajectory_name:=right_arm_swing_from_mjcf \
  enable_pre_positioning:=true \
  pre_position_duration_sec:=8.0 \
  playback_speed:=0.1 \
  command_rate_hz:=100.0
```

## 5. 启动基于 MJCF 的左臂摆臂测试

这条轨迹来自：

- `/home/sliouzhou04/jlj/JLJBot_mjcf/JLJBot/JLJBot.xml`

对应 CSV：

- `config/left_arm_swing_from_mjcf.csv`

建议一开始先慢速跟随：

```bash
export ROS_LOG_DIR=/tmp/roslog_dm_test
ros2 launch dm_test arm_trajectory_test.launch.py \
  trajectory_name:=left_arm_swing_from_mjcf \
  enable_pre_positioning:=true \
  pre_position_duration_sec:=8.0 \
  playback_speed:=0.1 \
  command_rate_hz:=100.0
```

这里会先把左臂从“当前实际位置”缓慢移动到轨迹起始位，然后再开始正式摆臂。

## 6. 启动双臂交替摆动测试

```bash
export ROS_LOG_DIR=/tmp/roslog_dm_test
ros2 launch dm_test arm_trajectory_test.launch.py \
  trajectory_name:=both_arms_alternating_swing \
  enable_pre_positioning:=true \
  pre_position_duration_sec:=10.0 \
  playback_speed:=0.08 \
  command_rate_hz:=100.0
```

这条轨迹会让双臂做交替摆动，左右相位相反。第一次测试建议保持更慢一点，比如 `0.08` 或 `0.05`。

## 7. 关键话题

- `dm_test/reference_joint_states`
  参考轨迹
- `dm_test/actual_joint_states`
  实际反馈
- `dm_test/error_joint_states`
  跟踪误差

## 8. 启动全身走路轨迹测试

当前已经把外部 `zoulu.csv` 转成包内测试轨迹：

- `config/whole_body_zoulu.csv`

源文件位置：

- `/home/sliouzhou04/jlj/pkl/zoulu.csv`

这条轨迹来自原始文件的 29 个逻辑关节角，原始帧率是 `30 FPS`，根节点位姿没有下发给电机。

全身轨迹里的并联机构关节使用逻辑关节名：

- `waist_roll` / `waist_pitch`
- `left_ankle_pitch` / `left_ankle_roll`
- `right_ankle_pitch` / `right_ankle_roll`

`dm_trajectory_test_node` 会读取 `dm_humanoid/config/dm_humanoid_29.yaml` 里的三组 `parallel_mechanisms` 配置，发送前自动把这三组 pitch/roll 目标逆解成：

- `waist_parallel_1` / `waist_parallel_2`
- `left_ankle_parallel_1` / `left_ankle_parallel_2`
- `right_ankle_parallel_1` / `right_ankle_parallel_2`

同时，`dm_test/reference_joint_states`、`dm_test/actual_joint_states` 和 `dm_test/error_joint_states` 发布的仍然是逻辑关节空间，方便直接看 `waist_pitch/roll` 和 `ankle_pitch/roll` 的跟踪误差。

如果 `/home/sliouzhou04/jlj/pkl/zoulu.csv` 更新了，可以重新生成包内测试 CSV：

```bash
python3 /home/sliouzhou04/jlj_ws/src/dm_test/scripts/convert_whole_body_csv.py \
  --source /home/sliouzhou04/jlj/pkl/zoulu.csv \
  --output /home/sliouzhou04/jlj_ws/src/dm_test/config/whole_body_zoulu.csv \
  --fps 30
```

第一次测试建议非常慢：

```bash
export ROS_LOG_DIR=/tmp/roslog_dm_test
ros2 launch dm_test whole_body_trajectory_test.launch.py \
  playback_speed:=0.05 \
  pre_position_duration_sec:=15.0 \
  command_rate_hz:=100.0
```

如果确认动作方向和幅度都正常，再逐步调大：

```bash
playback_speed:=0.1
```

`playback_speed` 控制轨迹播放速度，`0.05` 表示原始速度的二十分之一。控制节点会先把全身关节从当前反馈位置缓慢移动到轨迹第一帧，再开始正式跟踪。

## 9. 轨迹效果评估

建议录制这些话题：

```bash
ros2 bag record \
  /dm_test/reference_joint_states \
  /dm_test/actual_joint_states \
  /dm_test/error_joint_states
```

当前包里新增了一个离线评估工具：

```bash
python3 /home/sliouzhou04/jlj_ws/src/dm_test/scripts/evaluate_tracking.py \
  --error-csv /你的/error_joint_states.csv
```

它会输出左臂各关节的：

- `MAE`
- `RMSE`
- `MaxAbsError`

如果你没有直接导出误差 CSV，也可以用参考和实际两个 CSV 来算：

```bash
python3 /home/sliouzhou04/jlj_ws/src/dm_test/scripts/evaluate_tracking.py \
  --reference-csv /你的/reference_joint_states.csv \
  --actual-csv /你的/actual_joint_states.csv
```

示例 CSV 在：

- `scripts/example_error_joint_states.csv`

## 10. 实时画轨迹曲线

现在也可以实时画每个左臂电机的：

- 参考轨迹
- 实际反馈
- 位置误差

实时绘图脚本在：

- `scripts/realtime_plot_tracking.py`

运行方式：

```bash
source /home/sliouzhou04/jlj_ws/install/setup.bash
python3 /home/sliouzhou04/jlj_ws/src/dm_test/scripts/realtime_plot_tracking.py
```

或者用安装后的脚本：

```bash
ros2 run dm_test realtime_plot_tracking.py
```

默认会订阅：

- `/dm_test/reference_joint_states`
- `/dm_test/actual_joint_states`
- `/dm_test/error_joint_states`

默认显示左臂 7 个关节，滚动时间窗是 15 秒。

如果你想改时间窗和刷新频率，可以这样：

```bash
python3 /home/sliouzhou04/jlj_ws/src/dm_test/scripts/realtime_plot_tracking.py \
  --window-sec 20 \
  --update-hz 15
```

说明：

- 这需要本机 Python 环境里有 `matplotlib`
- 最适合一边跑 `dm_test`，一边开一个新终端实时看曲线

## 11. 用 CSV 跑左臂轨迹

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
- `enable_pre_positioning`
  是否先把当前关节缓慢移动到轨迹起始位
- `pre_position_duration_sec`
  预对位持续时间，越大越慢，测试初期建议 `5~10` 秒

推荐一开始先这样理解：

- `command_rate_hz` 控制“下发命令有多快”
- `playback_speed` 控制“轨迹本身播放有多慢”

也就是说，你完全可以让机器人用 `100 Hz` 稳定收命令，同时把轨迹按 `0.1x` 慢慢播放。

如果机器人开机时左臂不在轨迹起点，建议保持：

```bash
enable_pre_positioning:=true pre_position_duration_sec:=8.0
```

如果你的 CSV 没有时间列，还可以加：

```bash
csv_row_rate_hz:=30.0
```

这表示 CSV 每一行相当于原始轨迹的 `1/30` 秒。

## 12. CSV 格式要求

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

## 13. 轨迹配置说明

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
