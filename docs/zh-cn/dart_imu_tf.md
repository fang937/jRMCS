# 飞镖 IMU 与 TF

`dart_launcher.yaml` 中的 `DartLauncherHardware` 直接完成 IMU 采集、EKF 解算和 IMU/TF 发布，
复用 C 板 BMI088 回调及 `Bmi088Ekf`，无需额外的 broadcaster 组件或 IMU 节点。
本实现假设 C 板刚性固定在底座；如果 C 板随偏航机构运动，需修改 IMU 所属坐标系及变换关系。

## 数据与坐标系

- `/dart/imu/data`：ROS `sensor_msgs/msg/Imu`，SensorDataQoS，默认最大发布频率 100 Hz。
- `imu_link`：物理 BMI088 传感器坐标系，姿态为 sensor 到 odom_imu 的旋转。
- 角速度单位 rad/s，加速度单位 m/s²，包含重力；加速度为最新样本，并非插值后的同步样本。
- 消息时间戳为主机发布时刻；板载时间戳仅用于 EKF 积分和判断新样本，未映射为 ROS 时间。
- 协方差全零表示未知。无新 IMU 样本时不重复发布 IMU 和 IMU 姿态 TF。

TF 树：

```text
odom_imu
└── base_link                 IMU 动态姿态，平移固定为 0
    ├── imu_link              静态安装外参
    └── yaw_link              编码器实际角度，绕 +Z 旋转
        └── muzzle_link       静态发射口外参，+X 为发射方向
```

只有收到有效 IMU 样本后才发布 `odom_imu -> base_link`。
只有四台电机都收到过反馈后才发布 `base_link -> yaw_link`。
静态边通过 `/tf_static` 发布，动态边通过 `/tf` 发布。
不同时加载其他发布相同子坐标系的 TF 发布器，避免同一子坐标系有多个发布源。

本实现只估计姿态，不估计底座位置。没有磁力计，航向只有相对参考并可能漂移；
`odom_imu` 不表示外部定位提供的绝对世界坐标。
距离电机和左右扳机没有已知的机构几何映射，因此没有虚构其 TF 关节。

## 安装标定

参数位于 `dart_launcher.yaml` 的 `dart_launcher_hardware.ros__parameters`：

- `imu_xyz`：base_link 下传感器位置，单位 m。
- `imu_rpy`：传感器坐标到 base_link 的旋转，单位 rad，组合顺序 `Rz(yaw) * Ry(pitch) * Rx(roll)`。
- `yaw_xyz`：base_link 下偏航轴的位置，单位 m。
- `yaw_angle_scale` / `yaw_angle_offset`：偏航角为 `motor_angle * scale + offset`，单位 rad。
  scale 包含机构额外传动比和旋向；电机反馈本身已包含 DJI 减速箱减速比。
- `muzzle_xyz` / `muzzle_rpy`：yaw_link 下发射口位置和姿态。

当前几何外参均为零值占位，scale 为 1，必须按实际安装标定。
IMU 安装旋转通过 `q_world_base = q_world_sensor * inverse(q_base_sensor)` 补偿。
这些参数仅用于观测，不改变电机控制或软件限位。

## 运行检查

使用现有启动入口加载 `dart_launcher.yaml` 后：

```bash
ros2 topic echo /dart/imu/data --qos-reliability best_effort
ros2 run tf2_ros tf2_echo odom_imu imu_link
ros2 run tf2_ros tf2_echo base_link yaw_link
```

静止时角速度应接近零，加速度模长应接近重力加速度；转动 C 板时检查姿态方向。
在 RViz 中可将 Fixed Frame 设为 `odom_imu` 并添加 TF 显示。
