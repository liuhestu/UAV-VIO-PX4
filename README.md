OpenVINS 输出 RealSense IMU 的 3D 位姿，Adapter 用一个固定外参 TPVT_{PV} 把它转换成 Pixhawk IMU / PX4 控制所使用的位姿，再送给 PX4。

```bash
OpenVINS：Odom 6 DOF
  ↓
estimator_adapter：外参变换，发布数据
  ↓
MAVROS
  ↓
PX4 EKF2（EKF2_EV_CTRL = 3，XYZ融合）
  ↓
vehicle_local_position
  ↓
位置环
  ↓
悬停
```