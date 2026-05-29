# dm_test

`dm_test` 是一个面向机器人分部位轨迹跟踪测试的 ROS 2 包。

当前第一版已经提供：

- 左臂测试
- 右臂测试
- 左腿 / 右腿 / 腰部 / 整机分组预留
- 基于当前姿态的小幅相对轨迹跟踪
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

## 5. 轨迹配置

默认测试配置文件：

- `config/dm_test_trajectories.yaml`

配置思路：

- `groups`
  定义测试对象，例如 `left_arm`、`right_arm`
- `trajectories`
  定义具体轨迹
- `offsets`
  表示相对于“启动时当前姿态”的小幅偏移量，单位是弧度

- `motor_config_path`
  支持 `package://dm_motor/config/dm_motor_29.yaml` 这种包内路径写法，安装后启动也能正常找到配置

后续如果你要扩展左腿、右腿、腰部和整机测试，优先直接在这个 YAML 里追加轨迹即可。
