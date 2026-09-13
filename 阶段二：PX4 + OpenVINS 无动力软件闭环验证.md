# 阶段二：PX4 + OpenVINS 无动力 EKF2 与控制器数据流验证

> 状态：**已执行完成：Phase2-A PASS；Phase2-B PASS（仅 DISARM/landed 安全态）；已回滚并重启验收**  
> 审查时间：2026-09-13（Asia/Shanghai）  
> 安全定位：裸板、DISARM、无电机，只验证 EKF2 外部视觉处理和位置控制器软件数据流。  
> 重要边界：本阶段不是物理闭环、不是飞行验证，也不能证明位置控制可用。

## 当前执行结果（2026-09-13）

- 已新增 phase2 Adapter 配置和 launch：默认 `output_enabled=false`、`send_velocity=true`。
- 已增加速度 covariance 合法性与旋转测试；测试汇总 7 项、0 失败。
- EuRoC 离线实测输出稳定 30.000 Hz，位姿、body velocity 和 velocity covariance 有限。
- Pixhawk 首次重新连接时确认 `connected=true`、`armed=false`、模式 `AUTO.LOITER`。
- 已读取的参数原值：`EKF2_EV_CTRL=0`、`EKF2_EV_DELAY=0 ms`、`EKF2_EV_NOISE_MD=0`、`EKF2_EV_QMIN=0`、`EKF2_EVP_GATE=5`、`EKF2_EVP_NOISE≈0.1`、`EKF2_EV_POS_X/Y/Z=0`、`EKF2_MULTI_IMU=2`；该固件未导出 `EKF2_MULTI_MAG`。
- 物理重连后完成 20 秒稳定心跳检查；全过程保持 `armed=false`，未执行模式切换、ARM、actuator test 或 setpoint 发送。
- PX4 连续接收约 30 Hz 数据，实测 `pose_frame=2`、`velocity_frame=3`，全部目标字段有限。
- 临时设置 `EKF2_EV_CTRL=1` 后，两个 EKF 实例均 `cs_ev_pos=true`、`cs_ev_vel=false`；主实例连续 `fused=true`、`innovation_rejected=false`。
- 当前 `AUTO.LOITER` 已启用位置控制器；运行时观察到 DISARM/landed 安全态的 local-position/attitude setpoint 输出。
- 已恢复 `EKF2_EV_CTRL=0`、停止全部视觉进程并重启飞控；重连后两个 EKF 实例均 `cs_ev_pos=false`，无测试进程残留。
- 完整证据、结论边界和已知异常见 `logs/phase2/phase2_ekf_control_dataflow_report.md`。

## 0. 审查结论

总体方向合理，但原计划不能直接执行，必须修正以下问题：

1. **阶段一的 NaN 速度会阻断 PX4 v1.17 的 EV position 处理。** 阶段一 PX4 实测为 `velocity_frame=UNKNOWN(0)`。本机源码的 `controlExternalVisionFusion()` 会先按 velocity frame 分支，遇到 UNKNOWN 会在调用 `controlEvPosFusion()` 前返回。阶段二必须令 Adapter 输出坐标语义正确的有限速度，使 PX4 得到合法 velocity frame；但 `EKF2_EV_CTRL` 仍只打开 bit 0，不融合速度。
2. **EuRoC 运动与静止 Pixhawk IMU 不一致不等于一定被拒绝。** 如果没有其他水平位置源，PX4 v1.17 启动 EV position fusion 时会直接把水平位置状态 reset 到视觉测量；之后才可能发生 innovation rejection 或再次 reset。必须观测 reset 事件，不能只观察 reject。
3. **关闭 EV fusion 不会自动恢复测试前的 EKF 状态。** 回滚参数后需要重启飞控/EKF，并重新检查基线；仅把 `EKF2_EV_CTRL` 改回原值不算恢复完成。
4. **`vehicle_local_position` 持续发布本身不是 EV 融合证据。** 必须同时使用 `estimator_status_flags`、`estimator_aid_src_ev_pos` 和 `estimator_event_flags`。
5. **DISARM 时控制器输出不是物理闭环。** PX4 的 `mc_pos_control` 在 landed/not-taken-off 条件下会覆盖 setpoint 以避免产生推力。本阶段最多证明模块执行和数据依赖成立，不能通过控制输出评价 VIO 或控制律效果。
6. **不应为了运行控制器绕过模式安全检查。** 若当前模式已使 position-control flag 生效，则只读观察；若未生效，任何模式切换都作为可选 Gate，执行前需要用户再次确认。禁止修改 RC、arming 或 failsafe 参数来强行进入模式。

修正后的阶段二分为两个独立结论：

```text
A. 实机裸板：证明 OpenVINS measurement 被 EKF2 处理，并明确 fused/rejected/reset 状态
B. 源码 + 可选实机只读观测：证明 vehicle_local_position 是 mc_pos_control 的输入，模块产生安全态输出
```

只有带一致 IMU/视觉/动力学的 SITL/HITL 才能验证真正的软件反馈闭环；不把 EuRoC + 静止 Pixhawk 包装成闭环验证。

---

## 1. 已确认的本机基线

| 项目 | 实测/源码结果 | 阶段二含义 |
| --- | --- | --- |
| PX4 飞控固件 | v1.17.0，commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` | 只按该版本字段验收 |
| PX4 本地源码 | `PX4-Autopilot/`，同一 commit | 只读核对，不编译、不刷写 |
| OpenVINS | commit `69488123ed9362dd44b6f28e7f4680abbff1442b` | 阶段一已编译通过 |
| MAVROS2 | 2.14.0 | odometry plugin 已在阶段一实机验证 |
| Adapter | `ros2_ws/src/estimator_adapter/` | 阶段一 5 项测试通过 |
| 阶段一 PX4 输入 | `pose_frame=2`、`velocity_frame=0` | position frame 可用，velocity frame 阻断阶段二 |
| 阶段一参数 | `EKF2_EV_CTRL=0` | 未融合 |
| 阶段一安全状态 | `armed=false`，用户确认裸板无电机 | 本阶段仍需重新确认 |
| EuRoC bag | `/home/he/datasets/euroc/V1_01_easy_db` | 只单次播放三个传感器 topic |
| 飞控当前连接 | 审查时 USB by-id 不存在 | 执行前必须重新连接并解析设备 |

阶段一完整证据见：

```text
logs/phase1_execution_report.md
```

---

## 2. 阶段二目标与非目标

### 2.1 必须完成的目标

```text
EuRoC → OpenVINS → Adapter → MAVROS → vehicle_visual_odometry
                                      ↓
                              EKF2 EV processing
                                      ↓
                estimator_aid_src_ev_pos / status / events
                                      ↓
                           vehicle_local_position
```

证明以下事实：

- EV position measurement 确实到达 EKF2 aid-source 处理链。
- `EKF2_EV_CTRL=1` 只请求水平位置融合。
- 每条样本的 `fused`、`innovation_rejected`、innovation 和 test ratio 可观测。
- 是否发生 `reset_pos_to_vision`、`starting_vision_pos_fusion` 有明确证据。
- `vehicle_local_position` 的 reset counter、有效性和数值变化可与 EV 事件关联。
- 源码证明 `mc_pos_control` 订阅 `vehicle_local_position`；若当前控制模式已激活该模块，则补充安全态运行证据。

### 2.2 本阶段不做

- 不 ARM，不做 actuator test，不发送执行器命令。
- 不刷写或修改 PX4 固件。
- 不打开 EV vertical position、velocity 或 yaw fusion。
- 不修改 innovation gate/noise 来强行接受 EuRoC。
- 不修改 RC、arming、failsafe、land detector 或 preflight-check 参数。
- 不发送 Offboard/Position setpoint。
- 不评价 VIO 精度、控制稳定性或可飞行性。
- 不把源码调用链等同于运行时闭环。

---

## 3. 版本特性与临时坐标假设

### 3.1 EKF2_EV_CTRL 位定义

本机 PX4 v1.17 源码 `params_external_vision.yaml` 定义：

| Bit | 值 | 含义 | 本阶段 |
| ---: | ---: | --- | --- |
| 0 | 1 | Horizontal position | 开启 |
| 1 | 2 | Vertical position | 关闭 |
| 2 | 4 | 3D velocity | 关闭 |
| 3 | 8 | Yaw | 关闭 |

因此测试值只能是：

```text
EKF2_EV_CTRL = 1
```

不能在旧值上直接做按位 OR；执行前必须确认测试期间目标值精确为 1，结束后恢复原值。

### 3.2 测试用外参

EuRoC IMU frame 临时视为 aircraft body：

```text
R_BI = I
t_BI = 0
EKF2_EV_POS_X/Y/Z = 0
```

这只是为了证明软件处理链，不代表真实 RealSense/OpenVINS 安装关系。全程标记：

```text
TEST_ONLY / NOT_CALIBRATED / NOT_FOR_FLIGHT
```

执行时先读取三项 `EKF2_EV_POS_*`。若任一原值非 0，停止并报告，不在本阶段静默覆盖；只有三项原值本来就是 0 时才继续。

不得额外发布伪造的 ENU↔NED 或 FLU↔FRD identity TF。标准轴变换继续由阶段一已验证的 MAVROS 变换完成。

### 3.3 阶段二速度策略

阶段一的 `send_velocity=false` 不能沿用。阶段二必须：

```text
send_velocity = true
EKF2_EV_CTRL = 1
```

两者不矛盾：

- 有限速度让 PX4 建立合法 `velocity_frame`，避免 v1.17 在 position processing 前提前返回。
- Adapter 把 OpenVINS global velocity 旋转到 ROS child/body frame；MAVROS 再做 FLU→BODY_FRD。
- bit 2 保持关闭，所以 EKF2 不融合该速度。

进入参数修改 Gate 前必须在 `vehicle_visual_odometry` 验证：

```text
pose_frame = 2                 # POSE_FRAME_FRD
velocity_frame = 3             # VELOCITY_FRAME_BODY_FRD
position / q / velocity finite
velocity_variance finite
```

任一条件不满足，停止，不设置 `EKF2_EV_CTRL=1`。

---

## 4. Gate 0：安全、连接和参数快照

### 4.1 物理与进程条件

执行前逐项确认：

```text
[ ] 用户再次确认飞控为无电机裸板，或动力系统物理断开
[ ] /mavros/state: connected=true, armed=false
[ ] 没有 ARM、mode、setpoint、actuator 脚本运行
[ ] QGroundControl 不占用同一串口
[ ] by-id 串口重新出现并解析到实际 tty
[ ] 准备好 MAVLink Shell，用于 uORB 与 PX4 参数双重核验
```

若任何时刻出现 `armed=true`，立即关闭 Adapter 输出并终止实验。

### 4.2 参数和状态快照

至少记录以下原值，不先假设默认值：

```text
EKF2_EV_CTRL
EKF2_EV_DELAY
EKF2_EV_NOISE_MD
EKF2_EV_QMIN
EKF2_EVP_GATE
EKF2_EVP_NOISE
EKF2_EV_POS_X
EKF2_EV_POS_Y
EKF2_EV_POS_Z
EKF2_MULTI_IMU
EKF2_MULTI_MAG
```

同时记录：

```text
/mavros/state 的 mode/armed
listener estimator_status_flags 1
listener estimator_selector_status 1
listener estimator_aid_src_ev_pos 1
listener estimator_event_flags 1
listener vehicle_local_position 1
listener vehicle_control_mode 1
ekf2 status
```

解释前提：

- `EKF2_EV_QMIN` 必须 `<= 0` 才能接受当前 MAVLink ODOMETRY 的 `quality=0`。若原值大于 0，本阶段停止并单独评估，不能偷偷降低质量门限。
- 记录 `cs_gnss_pos` 或其他水平 aiding。无其他水平位置源时，EV 启动通常会 reset 水平位置；有 GNSS 时更可能通过 EV position bias 对齐，结果解释不同。
- 若存在多个 EKF2 instance，以 `estimator_selector_status.primary_instance` 为主结论，同时记录每个 `estimator_aid_src_ev_pos.estimator_instance`，不能把非主实例的融合状态误认为飞控正在采用的状态。
- `EKF2_EV_DELAY` 是 reboot-required 参数。本阶段只读取，不修改，不做延迟标定。

参数快照保存到：

```text
logs/phase2/parameters_before.txt
```

---

## 5. Gate 1：阶段二 Adapter 离线验证

先在不连接 MAVROS/PX4 的条件下，为 `send_velocity=true` 做离线验证：

1. 新增独立的 `phase2.yaml`/`phase2.launch.py`，默认 `output_enabled=false`、`send_velocity=true`；不得直接复用当前把 `send_velocity` 硬编码为 false 的 `phase1.launch.py`。
2. 运行现有 Adapter 单元测试。
3. 增加/确认 global velocity → child/body velocity 的基向量和非单位姿态测试。
4. 检查速度 covariance 旋转结果有限、对称、对角线非负。
5. 单次播放 EuRoC，只播放 `/imu0`、`/cam0/image_raw`、`/cam1/image_raw`。
6. 验证 Adapter 仍为 30 Hz、系统实时时间戳严格单调、stale input 会停止输出。
7. 保存一条完整 Odometry 和频率证据。

注意：不使用 bag `--loop`。阶段一已经发现回卷会给 OpenVINS 制造时间倒退；需要重跑时重新启动 OpenVINS 后单次播放。

Gate 1 通过条件：输出 position、quaternion、body velocity 和对应 covariance 全部有限，且没有把 global velocity 原样冒充 body velocity。

---

## 6. Gate 2：恢复链路，但保持融合关闭

启动顺序：

```text
1. MAVROS，经 by-id 串口连接
2. OpenVINS
3. Adapter，output_enabled=false、send_velocity=true
4. EuRoC 单次 bag
5. 确认 OpenVINS 初始化
6. 动态启用 Adapter 输出
```

此时保持：

```text
EKF2_EV_CTRL = 阶段二开始前原值（预期 0）
```

实机检查：

```text
listener vehicle_visual_odometry -n 5
```

必须满足：

- 5 条连续样本约 30 Hz，而非缓存值。
- `pose_frame=2`。
- `velocity_frame=3`。
- position、q、velocity、position/velocity variance 有限。
- `timestamp_sample` 新鲜且单调。
- `/mavros/state` 仍为 `armed=false`。

若 velocity frame 仍为 0，Gate 2 失败；不能继续修改 EKF 参数。

---

## 7. Gate 3：最小 EV horizontal-position processing

这是本阶段唯一允许的 PX4 参数修改：

```text
EKF2_EV_CTRL: 原值 → 1
```

预期使用本机已验证的 MAVROS 参数节点：

```bash
ros2 param set /mavros/param EKF2_EV_CTRL 1
ros2 param get /mavros/param EKF2_EV_CTRL
```

要求：

1. 通过 MAVROS 参数接口修改。
2. 立即通过 MAVROS 参数读回。
3. 再通过 PX4 NSH `param show EKF2_EV_CTRL` 交叉核验。
4. 不保存其他参数，不执行批量 param load/reset。

参数生效后连续采集：

```text
listener estimator_status_flags -n 5
listener estimator_selector_status -n 5
listener estimator_aid_src_ev_pos -n 10
listener estimator_event_flags -n 5
listener vehicle_local_position -n 10
ekf2 status
```

### 7.1 证据解释

| 证据 | 可以证明 | 不能单独证明 |
| --- | --- | --- |
| `estimator_aid_src_ev_pos.timestamp_sample` 更新 | EV position 已进入 aid-source processing | 样本已融合 |
| `cs_ev_pos=true` | EKF2 正处于 EV position fusion intended 状态 | 每条样本均通过 gate |
| `fused=true` | 对应样本实际执行了融合 | 融合结果物理正确 |
| `innovation_rejected=true` | 对应样本被 innovation gate 拒绝 | 数据未进入 EKF2 |
| `reset_pos_to_vision=true` | 状态被重置到视觉位置 | 稳态融合健康 |
| `starting_vision_pos_fusion=true` | 启动了视觉位置融合 | VIO 可用于飞行 |

还需保存：

- observation、innovation、innovation variance。
- X/Y test ratio 和 filtered test ratio。
- `time_last_fuse` 是否持续推进。
- `vehicle_local_position.xy_valid`、`v_xy_valid`、`xy_reset_counter`、`delta_xy`。
- 主 EKF instance 是否变化；若 selector 切换实例，分别保存切换前后证据。
- EV 打开前后 `x/y/vx/vy/eph/evh` 的变化。
- 是否有 `vision data stopped`、fusion failing、reset 或 stop 事件。

### 7.2 通过条件

最低通过条件是：

```text
estimator_aid_src_ev_pos 持续更新
+
至少一种明确处理结果：fused / innovation_rejected / reset event
+
vehicle_local_position 的相关变化与事件时间一致
```

不要求所有样本 accepted。若 EKF 因 EuRoC 与静止 Pixhawk IMU 不一致而 reject/reset，记录真实结果，不扩大 gate、不改 noise、不重复重置来追求“好看”。

---

## 8. Gate 4：mc_pos_control 数据流验证

### 8.1 必做：源码证据

本机 v1.17.0 源码已经定位：

```text
MulticopterPositionControl::_local_pos_sub
  → MulticopterPositionControl::Run()
  → set_vehicle_states(vehicle_local_position, dt)
  → PositionControl::setState(states)
  → PositionControl::update(dt)
  → vehicle_local_position_setpoint
  → vehicle_attitude_setpoint
```

实际文件：

```text
src/modules/mc_pos_control/MulticopterPositionControl.hpp
src/modules/mc_pos_control/MulticopterPositionControl.cpp
src/modules/mc_pos_control/PositionControl/PositionControl.cpp
```

控制计算仅在：

```text
vehicle_control_mode.flag_multicopter_position_control_enabled = true
```

且 setpoint 时间条件满足时进入。DISARM/landed/not-taken-off 状态下，源码会覆盖 setpoint、重置积分并生成不产生推力的安全态输出。

### 8.2 可选：实机运行时证据

先只读当前状态：

```text
listener vehicle_control_mode -n 5
listener trajectory_setpoint -n 5
```

如果当前模式已经使 `flag_multicopter_position_control_enabled=true`，无需切换模式，直接观察：

```text
listener vehicle_local_position_setpoint -n 5
listener vehicle_attitude_setpoint -n 5
```

记录输入 `trajectory_setpoint` 与两个输出 topic 是否持续更新、时间戳是否与 `vehicle_local_position` 更新对应，以及输出是否处于 landed/DISARM 安全状态。

如果 flag 为 false：

- 本轮不自动切换模式。
- 停止在 Gate 3，向用户报告。
- 只有用户再次明确确认后，才可尝试一次 DISARM 模式切换。
- 如果 Commander 因 local position/manual control 等要求拒绝，记录拒绝；禁止修改 `COM_RC_IN_MODE`、arming checks、failsafe 或输入假数据绕过检查。
- 不使用 Offboard，因为它需要持续 setpoint，超出本阶段授权范围。

### 8.3 Gate 4 的结论边界

即使观测到 `vehicle_attitude_setpoint`，本阶段也只能得出：

```text
mc_pos_control 在该控制模式下消费 vehicle_local_position 并运行安全态计算
```

不能得出“OpenVINS 已闭环控制飞机”。真正的动态软件闭环应在后续 SITL/HITL 中，让 plant IMU 与视觉轨迹一致，并验证控制误差收敛。

最终报告必须分别给出两个 verdict，不得合并：

```text
Phase2-A EKF2 EV processing：PASS / FAIL
Phase2-B mc_pos_control runtime dataflow：PASS / NOT_RUN / BLOCKED / FAIL
```

源码调用链核对通过时，若没有运行时输出证据，Phase2-B 只能是 `NOT_RUN` 或 `BLOCKED`，不能标记为 PASS。

---

## 9. Gate 5：停止、回滚与清除 EKF 测试状态

停止顺序：

```text
1. output_enabled=false
2. 确认 /mavros/odometry/out 停止更新
3. EKF2_EV_CTRL 恢复阶段二开始前原值
4. MAVROS + NSH 双重读回参数
5. 停止 bag
6. 停止 Adapter
7. 停止 OpenVINS
8. 若曾切换模式，恢复原模式
9. 确认 armed=false
10. 停止 MAVROS
```

由于 EV position 可能 reset EKF 水平状态，还必须：

```text
11. 重启飞控，清除本次不一致数据造成的 EKF 状态
12. 重连后再次确认 armed=false
13. 再次读取 EKF2_EV_CTRL 和全部快照参数
14. 确认 cs_ev_pos=false，且没有 Adapter/Bag/OpenVINS 进程残留
```

回滚时使用快照中的实际整数原值替换 `<原值>`：

```bash
ros2 param set /mavros/param EKF2_EV_CTRL <原值>
ros2 param get /mavros/param EKF2_EV_CTRL
```

若无法完成飞控重启，则阶段二不得标记为“已恢复”。

---

## 10. 验收清单

```text
[x] Gate 0 安全条件、连接和原值快照完成
[x] Adapter send_velocity=true 的坐标和 covariance 测试通过
[x] PX4 vehicle_visual_odometry: pose_frame=2、velocity_frame=3
[x] EKF2_EV_CTRL 测试值精确为 1，bit 1/2/3 未开启
[x] estimator_aid_src_ev_pos 持续更新
[x] fused/rejected/reset 至少一种处理结果有明确证据（连续 fused=true）
[~] estimator_status_flags 已保存；estimator_event_flags 监听窗口无新样本
[x] estimator_selector_status 和主 EKF instance 已确认
[x] vehicle_local_position reset/validity/数值变化已保存
[x] mc_pos_control 源码数据流已核对
[x] Phase2-A EKF2 EV processing 已单独判定：PASS
[x] Phase2-B 有 trajectory/state/output 运行时证据：PASS（仅安全态）
[x] 未 ARM、未 actuator test、未发送 setpoint、未绕过安全检查
[x] 参数恢复原值，并通过 MAVROS/NSH 双重读回
[x] 飞控重启后 cs_ev_pos=false，基线恢复
[x] 报告完整记录结论边界和未验证事项
```

阶段二报告保存到：

```text
logs/phase2/phase2_ekf_control_dataflow_report.md
```

---

## 11. 立即停止条件

- `armed=true` 或出现任何执行器/电机输出迹象。
- 裸板/动力物理断开条件无法再次确认。
- `velocity_frame != 3`，或 position/q/velocity/covariance 非有限。
- 时间戳倒退、陈旧或输出频率失控。
- `EKF2_EV_CTRL` 读回不是精确的 1。
- 发现 EV velocity、vertical position 或 yaw fusion 被开启。
- EKF2 出现持续 fault、模块重启或数据停止。
- 参数原值不完整、无法双重读回或无法可靠回滚。
- 需要放宽 innovation/noise/quality、安全或模式门限才能继续。
- 飞控 USB 掉线，或 QGroundControl/MAVROS 串口争用。

---

## 12. 后续阶段建议

阶段二通过后仍不能直接接位置环飞行。推荐顺序：

```text
阶段三 A：真实 RealSense/IMU 时间同步、内外参和 body-frame 标定
阶段三 B：SITL/HITL 一致动力学下的 EKF2 + mc_pos_control 动态闭环
阶段三 C：真实传感器手持方向、尺度、延迟、reset 和掉线测试
阶段四：拆桨整机测试与受控首飞评审
```

在真实传感器外参、延迟、yaw、速度语义和故障恢复通过前，不进入位置控制飞行。
