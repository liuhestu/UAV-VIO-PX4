# 阶段二执行报告：PX4 + OpenVINS 无动力 EKF2 与控制器数据流

执行时间：2026-09-13（Asia/Shanghai）

## 结论

```text
Phase2-A EKF2 EV processing：PASS
Phase2-B mc_pos_control runtime dataflow：PASS（仅 DISARM/landed 安全态）
回滚与重启验收：PASS
```

本次证明了 OpenVINS 经 Adapter/MAVROS 形成的 external-vision 水平位置测量能被 PX4 v1.17 EKF2 实际融合，并且融合后的 `vehicle_local_position` 所在数据链会触发 `mc_pos_control` 的安全态计算。它没有证明真实飞机的位置闭环、控制误差收敛或可飞性。

## 安全边界

- 飞控为裸板、未连接电机；全过程 `armed=false`。
- 未执行 ARM、actuator test、模式切换或固件刷写。
- 未发送任何 setpoint，未修改 RC、arming、failsafe 或 innovation/noise 门限。
- 唯一临时修改的 PX4 参数为 `EKF2_EV_CTRL: 0 → 1 → 0`。
- bit 0 之外的 EV velocity、vertical position、yaw fusion 均未开启。

## Gate 0：连接与参数基线

- USB by-id：`/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00`。
- PX4：v1.17.0，commit `d6f12ad1c4000000`。
- 20 秒内连续 24 条 `/mavros/state` 均为 `connected=true`、`armed=false`、`AUTO.LOITER`，无 heartbeat loss。
- `EKF2_EV_CTRL=0`；完整参数基线见 `parameters_before.txt`。
- `EKF2_MULTI_IMU=2`，实际有两个健康 EKF 实例；固件未导出 `EKF2_MULTI_MAG`。

## Gate 1：Adapter 离线验证

- phase2 配置默认 `output_enabled=false`、`send_velocity=true`、30 Hz、`odom → base_link`。
- 单元/集成测试汇总：7 项，0 error，0 failure，0 skipped。
- 新增速度 covariance 有限性、对称性、非负对角线检查，以及 global-to-body 旋转测试。
- EuRoC 离线输出稳定 30.000 Hz；position、quaternion、body velocity 与 velocity covariance 均有限。
- stale input 会停止输出；本轮 bag 播放结束后日志也再次验证了该保护。

## Gate 2：PX4 收包、融合保持关闭

OpenVINS 初始化后动态启用 Adapter，ROS 输出为 30.000–30.002 Hz。PX4 NSH 连续五条 `vehicle_visual_odometry` 显示：

- `pose_frame=2`；
- `velocity_frame=3`；
- position、q、velocity、position/orientation/velocity variance 均有限；
- `timestamp_sample` 连续推进，样本间隔约 30–67 ms；
- 此时 MAVROS 与 NSH 均读回 `EKF2_EV_CTRL=0`，飞控仍 `armed=false`。

因此阶段一的 `velocity_frame=0` 阻断条件已经消除，且 Gate 2 没有提前开启融合。

## Gate 3：EKF2 external-vision 水平位置处理

临时设置并双重读回：

```text
EKF2_EV_CTRL=1
```

关键证据：

- `estimator_selector_status`：`primary_instance=0`、`instances_available=2`，两实例均 healthy，采样期间主实例未切换。
- 两个实例的 `estimator_status_flags` 均为 `cs_ev_pos=true`、`cs_ev_vel=false`，且 horizontal position/velocity reject flags 为 false。
- uORB aid-source topic 的 instance 与消息内 estimator 编号交叉映射：topic instance 1 对应 `estimator_instance=0`（主实例），topic instance 0 对应 `estimator_instance=1`。
- 主实例连续样本均 `fused=true`、`innovation_rejected=false`，`time_last_fuse` 持续推进。
- 主实例样例 test ratio：X 约 0.013–0.015，Y 约 0.006–0.007，显著小于 1。
- 另一实例连续五条也均 `fused=true`、`innovation_rejected=false`，test ratio 约 0–0.013。
- `vehicle_local_position` 连续更新，`xy_valid=true`、`v_xy_valid=true`、`dead_reckoning=false`；观测到 `xy_reset_counter=8`、`delta_xy=[0.91790, -0.08857]`。由于未保存事件发生前的同字段样本，不能把该累计 reset 次数全部归因于本轮 EV。
- `ekf2 status`：两个实例均 healthy、local position valid，主实例保持 0。

`estimator_event_flags` 在监听窗口内没有产生可打印的新样本，因此没有把“某一 reset event”作为 PASS 依据；PASS 基于更直接的连续 `fused=true` 和 `time_last_fuse` 推进。

## Gate 4：mc_pos_control 数据流

源码链路核对：

```text
vehicle_local_position subscription
→ set_vehicle_states(...)
→ PositionControl::setState(...)
→ PositionControl::update(...)
→ vehicle_local_position_setpoint
→ vehicle_attitude_setpoint
```

本机 PX4 源码位置包括：

- `MulticopterPositionControl.hpp:102,241`
- `MulticopterPositionControl.cpp:394,427,569,574,603,609,612`
- `PositionControl.cpp:91,252,266`

运行时证据：

- `flag_multicopter_position_control_enabled=true`、position/velocity/altitude/attitude control flags 为 true；
- `flag_armed=false`、`flag_control_offboard_enabled=false`；
- `trajectory_setpoint` 持续刷新，但各 position/velocity/acceleration/yaw 字段为 NaN；
- `vehicle_local_position_setpoint` 持续刷新，位置/速度为 NaN，thrust 为 `[0, 0, -0.001]`；
- `vehicle_attitude_setpoint` 持续刷新，roll/pitch 为 0，thrust 为 `[0, 0, -0.001]`。

这与 DISARM/landed 安全覆盖逻辑一致：控制模块在运行，但没有有效飞行 setpoint，也不产生实际控制推力。因此 Phase2-B 的 PASS 仅表示运行时数据流成立。

## Gate 5：停止、回滚与清除状态

执行结果：

1. Adapter `output_enabled=false`，3 秒监听窗口无 `/mavros/odometry/out` 新消息。
2. `EKF2_EV_CTRL` 恢复为 0，MAVROS 与 NSH 均读回 0。
3. bag、Adapter、OpenVINS 全部停止；全过程仍 `armed=false`。
4. 执行 PX4 reboot 清除测试形成的 EKF 状态。
5. 重连后再次确认 `connected=true`、`armed=false`、`AUTO.LOITER`、`EKF2_EV_CTRL=0`。
6. 重启后两个 EKF 实例均 `cs_ev_pos=false`、`cs_ev_vel=false`。
7. 最终无 MAVROS、MAVLink shell、OpenVINS、Adapter 或 bag 进程残留。

回滚快照见 `parameters_after_reboot.txt`。

## 执行中发现的非阻断问题

- 三个运行脚本原先在 source ROS 环境前启用 `set -u`，会因 Humble setup 读取未定义变量而立即退出；已改为完成 setup 后再启用 `set -u`。
- OpenVINS 在 SIGINT 退出阶段发生 exit code -11；数据处理期间正常。这与阶段一已知退出问题一致，需要后续单独修复，但不改变本轮运行时证据。
- PX4 reboot 导致 USB EOF 时，当前 MAVROS 进程以 `Resource deadlock avoided` 退出；重新启动 MAVROS 后连接和回滚检查正常。后续自动化脚本应把“飞控重启后重建 MAVROS 进程”作为固定步骤。
- MAVROS 对 PX4 Events 的文本解码不完整，日志出现 `UNK EVENT`；本轮判据使用 PX4 uORB 原始字段，不依赖这些文本。

## 未验证事项与下一步

尚未验证：真实传感器时间同步、相机/IMU/机体系外参、真实延迟、yaw 对齐、持续掉线恢复、动态 plant 一致性、控制误差收敛和任何带动力行为。

下一步应先做阶段三 SITL/HITL 一致动力学闭环，再做真实传感器手持方向/尺度/延迟测试；在这些通过前，不进入位置控制飞行。
