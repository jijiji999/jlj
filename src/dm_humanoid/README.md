# dm_humanoid

`dm_humanoid` 是并列于 `dm_motor` 的上层人形机器人控制包。

职责：

- 组织 `fixed / passive / loco / dance` 模式状态机
- 订阅 `joy`、`imu/data`
- 调用 `dm_motor::DmMotorManager` 下发 29 关节 MIT 指令
- 为强化学习策略部署提供观测拼接与动作下发框架

默认控制配置：

- [config/dm_humanoid_29.yaml](/home/sliouzhou04/jlj_ws/src/dm_humanoid/config/dm_humanoid_29.yaml)

底层电机配置仍由：

- [../dm_motor/config/dm_motor_29.yaml](/home/sliouzhou04/jlj_ws/src/dm_motor/config/dm_motor_29.yaml)

提供。
