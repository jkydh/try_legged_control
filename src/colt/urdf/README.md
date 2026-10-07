# COLT 模型适配

`robot.xacro` 是入口，`colt_model.xacro` 保存 SolidWorks 模型，
`gazebo.xacro` 定义传动和仿真插件，`colt.urdf` 是生成结果。
适配依据是本仓库记录的 legged_control 子模块提交
`a7f381c0367e98e31c01336e678eef47e304d40d`。

## 生成 URDF

在项目根目录、已加载 ROS 环境的终端执行：

```bash
rosrun xacro xacro src/colt/urdf/robot.xacro > /tmp/colt.generated.urdf
check_urdf /tmp/colt.generated.urdf
```

默认带 Gazebo 扩展。`gazebo:=false` 可生成不含 Gazebo 插件的模型，
传动定义仍保留。后续修改模型时编辑 Xacro 文件，再重新生成 `colt.urdf`。
生成文件顶部含 Xacro 的自动生成标记。

## 名称映射

12 个驱动关节名称保持不变：`LF/LH/RF/RH` 加 `_HAA/_HFE/_KFE`。

| 原 link | 适配后 link |
| --- | --- |
| `base_link` | `base` |
| `imu_link` | `base_imu` |
| `C_<腿>_HAA` | `<腿>_hip` |
| `C_<腿>_HFE` | `<腿>_thigh` |
| `C_<腿>_KFE` | `<腿>_calf` |
| `C_<腿>_FOOT` | `<腿>_FOOT` |

足端固定关节命名为 `<腿>_foot_fixed`，IMU 固定关节命名为
`base_imu_joint`，并在 Gazebo 中禁止这五个固定关节合并。
网格文件名和 `package://colt/meshes/…` 路径保持不变。

CAD 导出的关节位置、轴方向、零位、限位、质量、惯性、视觉和碰撞几何均保留。
各 visual 的空 material 名称改为对应 link 的唯一名称。
固定关节中无作用的零向量 axis 已移除。

## 接入启动文件时的要求

这次只修改 `urdf/`，原有 `launch/` 和控制器配置没有修改。
因此模型适配完成不代表原来的 `gazebo.launch` 已能启动控制器：

- 将模型加载到 ROS 参数 `legged_robot_description`，插件使用此参数。
  `robot_state_publisher` 使用的 `robot_description` 也应加载同一模型。
- 加载 `legged_gazebo/config/default.yaml`，该文件的 IMU `base_imu`
  和四个足端名称已与适配后的模型一致。
- 更新使用 `base_link` 的 TF 配置为 `base`，设置合适的生成高度和初始关节角。
  当前 CAD 零位下足底相对 base 的 z 约为 −0.190 m，不能在 z=0 直接生成。
- 为 COLT 设置独立的 task/reference/gait 配置，并生成控制器 `urdfFile` 指向的 URDF。
  关节坐标顺序是 `LF, LH, RF, RH`，每腿依次 `HAA, HFE, KFE`；
  OCS2 默认接触顺序是 `LF_FOOT, RF_FOOT, LH_FOOT, RH_FOOT`。
- 关节轴和 CAD 零位与 A1 不同，需要重新确定 COLT 站姿。
  当前所有关节的 ±6.28 rad、100 N·m、100 rad/s 是导出值，仍需确认实际限制。

Gazebo 复用 `legged_gazebo/LeggedHWSim`。真实硬件的 `LeggedHW::read()/write()`
属于独立的通信接口开发，URDF 不实现这部分逻辑。

本次验证覆盖 Xacro 展开、ROS Python URDF 解析、模型树、名称、网格、惯性、
CAD 参数保留及仿真接口静态检查；未运行 Gazebo 或 MPC 控制器。
Python URDF 解析器会对 actuator 内的 `hardwareInterface` 发出未知标签提示；
这是与上游一致的 ros_control 扩展，传动定义中的这些标签已单独检查。
