# rmcs_tongji_auto_aim

基于同济 alliance_auto_aim 算法库的 RMCS 自瞄组件

## 概述

本包是 RMCS 框架的自瞄组件,使用了来自 [alliance_auto_aim](https://github.com/Alliance-Algorithm/alliance_auto_aim) 的先进算法,包括:

- **v1::Identifier**: 装甲板识别 (OpenVINO 神经网络)
- **tongji::Solver**: PnP 位姿求解
- **tongji::CarPredictorManager**: 11 维 EKF 整车状态预测
- **tongji::StateMachine**: 5 状态跟踪状态机
- **tongji::FireController**: 迭代弹道求解与射击控制

## 架构特点

### 双线程设计
- **主线程**: 弹道计算,输出控制信号 (@executor频率)
- **视觉线程**: 图像处理,识别,跟踪,火控 (@100fps)

### 双缓冲同步
- 使用 atomic + double buffer 实现 lock-free 线程通信
- 避免锁竞争,保证实时性

### ROS2 参数统一
- 所有配置参数通过 ROS2 参数系统管理
- 支持 launch 文件配置和运行时查询
- 角度参数使用用户友好的度数(内部自动转换为弧度)

## 依赖

### 内部依赖
- rmcs_executor
- rmcs_description
- rmcs_msgs
- fast_tf
- hikcamera
- **alliance_auto_aim** (作为 submodule)

### 外部依赖
- OpenCV >= 4.5
- Eigen3
- yaml-cpp
- OpenVINO (用于模型推理)

## 构建

```bash
cd /path/to/RMCS/rmcs_ws

# 确保 alliance_auto_aim submodule 已初始化
cd src
git submodule update --init --recursive alliance_auto_aim

# 构建
cd ..
colcon build --packages-select rmcs_tongji_auto_aim
```

## 使用

### 1. 在 launch 文件中加载组件

```yaml
components:
  - rmcs_tongji_auto_aim::TongjiAutoAimInitializer -> tongji_auto_aim_initializer
  - rmcs_tongji_auto_aim::TongjiAutoAimController -> tongji_auto_aim_controller
```

### 2. 配置参数

参考 `config/default_params.yaml` 创建自己的配置文件,主要参数:

```yaml
tongji_auto_aim_controller:
  ros__parameters:
    # 相机内参
    fx: 1234.5
    fy: 1234.5
    cx: 720.0
    cy: 540.0

    # 模型路径
    model_path: "models/szu_model.onnx"

    # 弹道参数
    bullet_speed: 26.0
    shoot_velocity: 26.0

    # 火控参数 (度数)
    yaw_offset_deg: 1.5
    pitch_offset_deg: -0.5
```

### 3. 接口说明

**输入 (Input)**:
- `/predefined/update_count`: 更新计数
- `/auto_aim/target_color`: 目标颜色
- `/auto_aim/whitelist`: 目标白名单
- `/tf`: TF 树

**输出 (Output)**:
- `/gimbal/auto_aim/control_direction`: 云台瞄准方向 (Eigen::Vector3d)
- `/gimbal/auto_aim/fire_control`: 射击许可 (bool)
- `/debug/target_omega`: 调试-目标角速度

## 注意事项

### 参数修改需重启

由于 tongji 算法模块在构造时加载参数,**运行时修改参数需要重启节点才能生效**:

```bash
ros2 lifecycle set /tongji_auto_aim_controller shutdown
# 然后重新启动
```

### 坐标系要求

确保 TF 树包含以下坐标系:
- `odom_imu`
- `camera_link`
- `yaw_link`
- `pitch_link`
- `muzzle_link`

### 模型文件

需要提供 SZU 识别模型 (.onnx 格式),放置在配置的 `model_path` 位置。

## 与旧版对比

| 特性 | rmcs_auto_aim | rmcs_tongji_auto_aim |
|------|---------------|----------------------|
| 识别算法 | OpenVINO MLP | OpenVINO (SZU模型) |
| PnP求解 | 融合求解器 (LightBar+IPPE) | IPPE + 重投影优化 |
| 跟踪策略 | 双层跟踪 (Armor+Car) | 整车11维EKF |
| 状态机 | 4状态 | 5状态 (含Switching) |
| 火控 | 双模式控制器 | 迭代弹道求解 |

## 许可

MIT License

## 贡献

欢迎提交 Issue 和 Pull Request!

## 参考

- [alliance_auto_aim](https://github.com/Alliance-Algorithm/alliance_auto_aim)
- [RMCS Framework](https://github.com/Alliance-Algorithm/RMCS)
