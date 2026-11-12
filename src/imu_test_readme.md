# H30 mini IMU 接入 legged_control

## 接入结果与范围

H30 mini 通过 USB 串口以 **460800、8N1、无流控**读取，C++ 节点发布标准 `sensor_msgs/Imu`，控制器可选择该话题作为状态估计输入。
节点不发送设备配置、校准或保存命令。

```text
H30 mini / CH9102 / cdc_acm
    → H30Serial：非阻塞串口读取
    → H30Parser：YESENSE 帧校验和解码
    → h30_imu_node：单位、安装方向转换
    → /h30/imu
    → LeggedController：回调 → RealtimeBuffer → 新鲜度/有效性检查
    → StateEstimateBase::updateImu()
    → KalmanFilterEstimate → MPC / WBC
```

当前 Colt 工程的关节和接触来自 Gazebo；本次增加的是 IMU 驱动和控制器输入通路，**没有新增实体电机、编码器或足端接触硬件驱动**。
控制器仍要求原有的关节、接触和 `base_imu` 硬件接口存在。外部话题模式会用 H30 数据替代读取该 IMU 句柄的数值。

默认 `use_h30_imu:=false`，原有 Gazebo 仿真流程继续使用仿真 IMU。
启用 H30 时默认选择 `cheater:=false`；cheater 估计器会使用 Gazebo 真值覆盖姿态，因此代码禁止把它与外部 IMU 话题模式混用。

## 新增、修改了哪些代码

以下路径相对于工作空间根目录 `/home/ydh/Desktop/try_legged_control`。

| 文件 | 新增/修改 | 作用 |
| --- | --- | --- |
| `src/legged_control/legged_hw/include/legged_hw/H30Imu.h` | 新增 | 定义 IMU 样本、协议解析器及串口类 |
| `src/legged_control/legged_hw/src/H30Imu.cpp` | 新增 | 串口打开/独占/恢复、非阻塞读取、帧同步、校验、TLV 解码 |
| `src/legged_control/legged_hw/src/H30ImuNode.cpp` | 新增 | 发布 ROS IMU；安装方向、世界航向偏置和超时检查 |
| `src/legged_control/legged_hw/config/h30_imu.yaml` | 新增 | 安装旋转、超时和协方差参数 |
| `src/legged_control/legged_hw/launch/h30_imu.launch` | 新增 | 单独启动 H30 节点 |
| `src/legged_control/legged_hw/CMakeLists.txt`、`package.xml` | 修改 | 编译和安装 `h30_imu` 库、`h30_imu_node`；声明 `sensor_msgs`、`tf2` 和测试依赖 |
| `src/legged_control/legged_controllers/include/legged_controllers/TopicImu.h` | 新增 | 固定大小的 IMU 缓存数据、消息有效性及新鲜度检查 |
| `src/legged_control/legged_controllers/include/legged_controllers/LeggedController.h` | 修改 | 增加订阅、实时缓存、输入参数和 `imu_used` 发布器 |
| `src/legged_control/legged_controllers/src/LeggedController.cpp` | 修改 | 在状态估计前选择硬件/话题输入；缺失或超时走现有停止通路；发布实际交给估计器的输入 |
| `src/legged_control/legged_controllers/CMakeLists.txt`、`package.xml` | 修改 | 声明 `sensor_msgs`、`realtime_tools`；加入输入检查测试 |
| `src/colt/launch/mpc.launch` | 修改 | 增加 H30 开关和串口参数，启动节点并设置控制器输入来源 |
| `src/colt/package.xml` | 修改 | 增加 `legged_hw` 运行依赖及集成测试所用消息依赖 |
| `src/legged_control/legged_hw/test/H30ImuTest.cpp` | 新增 | 协议、单位换算、伪串口和退出恢复测试 |
| `src/legged_control/legged_hw/test/h30_ros.test`、`h30_ros_test.py` | 新增 | 伪串口到真实 C++ ROS 节点的端到端测试 |
| `src/legged_control/legged_controllers/test/TopicImuTest.cpp` | 新增 | 消息无效、超时、时钟异常等检查测试 |
| `src/colt/test/h30_integration.py` | 新增 | 隔离的 Gazebo 测试，验证真实控制器消费 H30 输入并在断流后停止 |

`StateEstimateBase`、Kalman 滤波器和 MPC/WBC 算法没有为本次 IMU 接入改写；H30 数据进入它们已有的 `updateImu()` 通路。
原来的 `Imu_test/imu_test.py` 仍可独立使用，保留 CSV 保存功能。它与 ROS 节点不能同时占用同一串口。

## 数据格式、单位与坐标系

协议与已读取成功的 H30 相同：`59 53` 帧头、2 字节小端 TID、1 字节负载长度、TLV 数据、2 字节累加校验。
参考[H30 官方资料仓库的 YESENSE 示例](https://github.com/Seeed-Projects/WHEELTEC-H30-IMU/blob/main/6.Routines%20related%20to%20the%20inertial%20navigation%20module/3.Python%20routine/Yesense_Decode.py)。

- `0x10` 加速度：有符号小端整数乘 `1e-6`，输出 **m/s²**。
- `0x20` 角速度：整数乘 `1e-6` 得到 °/s，再乘 `π/180`，输出 **rad/s**。
- `0x41` 四元数：设备的 **w,x,y,z** 转成 ROS/硬件接口的 **x,y,z,w**，检查模长后归一化。
- 必须在**同一有效帧**里同时有加速度、角速度和四元数，才发布控制用 IMU。不会将不同帧的旧字段拼接成新样本。
- 帧头噪声、坏校验、TLV 越界、已知字段长度错误和无效四元数均被拒绝；未知 TLV 按长度跳过。
- 加速度保留重力项，不在驱动里扣除重力；现有 Kalman 估计器负责其状态估计计算。
- `header.stamp` 是主机读取后的 ROS 时间，不是设备内部采样时间；同批数据可能具有相同时间戳。

`sensor_to_base_rpy` 定义 **R_BS**：将传感器坐标下的向量转到输出 `base_imu` 坐标，三个值依次为 roll、pitch、yaw，单位 rad。

```text
angular_B = R_BS × angular_S
accel_B   = R_BS × accel_S
R_WB      = R_WS × transpose(R_BS)
```

`world_yaw_offset` 再对姿态左乘一个世界 Z 轴旋转，用于对齐航向；它不会改变机体坐标下的加速度和角速度。
默认安装旋转及航向偏置均为 0，假定传感器轴与 `base_imu` 一致，设备输出采用与估计器一致的 Z 向上世界姿态。
当前 URDF 中 `base → base_imu` 的旋转为 0；实际安装前仍需核对设备轴向和世界坐标约定。
驱动不会凭设备名称自动推断 NED/ENU，也不会自动归零航向。

协方差配置是**输出坐标系**下的对角项，单位分别为 rad²、(rad/s)²、(m/s²)²。
默认全部为 0，按 ROS IMU 约定表示未知，不是已标定为无噪声。填写测得的数值时要使用上述单位。

## 参数与话题

H30 节点的私有参数：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `~port` | 本次设备的 `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B31016849-if00` | 稳定串口路径；换设备需更新 |
| `~baud` | `460800` | 也支持 115200、230400、921600 |
| `~topic` | `/h30/imu` | 输出 `sensor_msgs/Imu` |
| `~frame_id` | `base_imu` | 输出数据轴对应的 frame |
| `~sensor_to_base_rpy` | `[0,0,0]` | 安装旋转，rad；通过 YAML 配置 |
| `~world_yaw_offset` | `0` | 世界航向偏置，rad |
| `~startup_timeout` | `5.0` | 首个完整样本等待时间，墙上秒数 |
| `~data_timeout` | `0.1` | 后续完整样本断流超时，墙上秒数 |
| `~*_covariance_diagonal` | `[0,0,0]` | 三种协方差对角项，见 YAML |

控制器参数位于 `/controllers/legged_controller/`：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `imu_source` | `hardware` | `hardware` 读取原句柄；`topic` 使用 ROS IMU |
| `imu_topic` | `/h30/imu` | 订阅话题 |
| `imu_frame_id` | `base_imu` | 严格匹配输入 frame；控制器不执行额外 TF 变换 |
| `imu_timeout` | `0.1` | 接收时间与 ROS 消息时间都要满足新鲜度要求 |

参数在控制器加载时读取，运行中修改参数不自动切换来源。
话题模式还发布 `/controllers/legged_controller/imu_used`：这是**已经交给 `updateImu()` 的输入**，只有控制器执行状态估计时才有消息。
它按控制循环而非设备采样率发布，可能重复同一个传感器样本；不要将其频率当成 H30 采样率。

串口读取运行在独立节点。控制回调把数据写入 `RealtimeBuffer`，控制循环只取缓存，不等待 USB 或 ROS 消息。
消息必须有匹配的 frame、非零时间戳、有限数值、有效四元数和可用的协方差标识。
新鲜度同时检查单调时钟接收间隔及 ROS 时间戳；最多允许 20 ms 的未来时间偏差。
数据失效时停止控制并清零已有控制命令，不回退成仿真 IMU，不自动恢复起立；恢复数据后需重新请求启动控制器。

## 1. 编译

```bash
cd /home/ydh/Desktop/try_legged_control
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin build legged_hw legged_controllers colt --no-deps -j2 -p1
source devel/setup.bash
rospack find legged_hw
```

`--no-deps` 适用于本工作空间已构建原依赖的情况，新工作空间需先构建依赖。
不需要安装 `pyserial`，也不需要安装额外的 CH9102 厂商驱动；本机已经由 `cdc_acm` 识别。

## 2. 真机 IMU 独立读取测试

先退出之前的 Python 串口测试和上位机。独立测试使用正常 ROS 时间；若上轮 Gazebo 留下 `/use_sim_time=true` 却没有 `/clock`，先结束旧 ROS 会话，在干净会话测试。

终端 A：

```bash
cd /home/ydh/Desktop/try_legged_control
source /opt/ros/noetic/setup.bash
source devel/setup.bash
ls -l /dev/serial/by-id/
python3 -B Imu_test/imu_test.py --list
roslaunch legged_hw h30_imu.launch
```

如果设备路径不同，指定 `port:=/dev/ttyACM0` 或实际稳定路径。
如果报权限不足，使用 `Imu_test/README.md` 中的临时 ACL 授权和撤销步骤；`sudo` 只用于授权，不以 root 运行整个 ROS 控制栈。

终端 B 也加载上述两个 setup，然后执行：

```bash
rostopic type /h30/imu
rostopic echo -n 1 /h30/imu
rostopic hz /h30/imu
```

验收：类型为 `sensor_msgs/Imu`，frame 为 `base_imu`，时间戳更新，四元数模长约为 1，无 NaN/Inf。
本设备之前独立 CSV 的接收频率约为 200 Hz；ROS 实测以当前配置和主机负载为准。
静置时角速度接近零，加速度模长接近 9.8 m/s²；缓慢绕已知轴转动时核对方向。
注意这里角速度单位是 rad/s，与原 CSV 中的 °/s 不同。

可选保存 ROS 数据，不再启动第二个串口读取程序：

```bash
mkdir -p Imu_test/artifacts
rosbag record -O "Imu_test/artifacts/h30_$(date +%Y%m%d_%H%M%S).bag" /h30/imu
```

录制结束按 `Ctrl+C`。CSV 仍可用原 Python 工具单独采集；ROS 接入本身通过 rosbag 保存。

## 3. 无实体设备的自动化测试

编译测试目标并执行单元测试：

```bash
catkin build legged_hw legged_controllers --no-deps --no-status -j2 -p1 --make-args tests
source devel/setup.bash
devel/lib/legged_hw/h30_imu_test
devel/lib/legged_controllers/topic_imu_test
rostest legged_hw h30_ros.test
```

前两项分别为 5 项和 3 项测试；ROS 测试会创建伪终端，将二进制帧输入真实的 C++ 节点，检查输出单位、四元数、90° 安装变换、断流超时以及串口参数恢复。

继续验证**真实控制器**的接入和停止行为：

```bash
python3 -B src/colt/test/h30_integration.py
```

该脚本使用独立 ROS/Gazebo 端口、临时日志目录和伪串口，启动无界面 Colt 仿真。
它只运行短暂起立过渡以检查 `imu_used`，在 MPC/WBC 接管前停止输入；不连接实体电机，不验证站立精度。
预期依次出现：

```text
PASS: actual LeggedController consumed H30 quaternion and rad/s data
PASS: controller stopped after external IMU timeout
```

结束时自动关闭本轮进程和伪串口，删除临时测试目录。失败时打印本轮启动日志供排查。

## 4. 在 Colt 启动流程中选择 H30：手持 IMU 模拟机身晃动

这一节用于**人为转动、倾斜 H30，观察控制器收到的姿态、估计结果和仿真控制输出**。
这是给估计器注入真实传感器扰动的联调实验：你的手决定 IMU 姿态，Gazebo 的物理引擎决定虚拟机身姿态，两者没有自动同步。
所以可以验证“手动倾斜 → H30 数据变化 → 控制器收到 → 估计与控制输出变化”，但不保证模型一定站稳或抵消手的晃动，也不能据此认定实体机器人抗扰能力已验收。

准备三个终端：**A 启动仿真，B 发送检查/启停命令，C 连续观察姿态**。需要录制时另开 D。
每个终端都先执行：

```bash
cd /home/ydh/Desktop/try_legged_control
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

若你当前已经启动了 H30 + Colt，且此前检查结果是 `/clock` 正常、`imu_source=topic`、控制器为 `initialized`，**直接从 4.3 开始**，不用再启动第二套仿真。
以下步骤只使用 Gazebo 控制进程，不启动 Unitree 等实体电机硬件节点。

### 4.1 终端 A：启动 H30 和可视化仿真

先退出上一轮 Colt，以及单独运行的 H30 节点、Python 串口测试和上位机。
不要同时执行 `roslaunch legged_hw h30_imu.launch`，本命令已经包含该节点。

```bash
roslaunch colt mpc.launch gui:=true use_h30_imu:=true \
  cheater:=false start_control:=false \
  imu_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B31016849-if00
```

`gui:=true` 便于看模型；没有图形环境时用 `gui:=false`，后面的数据检查相同。
串口路径变化时，将 `imu_port` 改成当前 `/dev/serial/by-id/` 中对应的路径。
如果提示权限不足，先按 `Imu_test/README.md` 做临时 ACL 授权，再重新启动。

此时模型会生成、自然趴下，控制器只加载，**不会自动起立**。
等待终端 A 出现：

```text
LeggedController IMU source=topic /h30/imu timeout=0.1
Colt ready and unpowered; start controllers/legged_controller via /controller_manager/switch_controller to stand up
```

首次/当前配置强制重编译 CppAD 时可能等待几十秒，期间先不晃动 IMU、不切换控制器。
默认起立过渡约 5 秒，计时从 4.5 发出启动命令后开始，不从 launch 启动时开始。
起立的 5 秒、10 秒判断使用仿真时间；Gazebo 运行较慢时，实际等待会更久。H30 断流超时使用墙上时间。

### 4.2 终端 B：确认数据来源、时钟和等待状态

```bash
rosparam get /controllers/legged_controller/imu_source
rosparam get /colt_bringup/cheater
rosparam get /colt_bringup/start_control
rostopic echo -n 1 /clock
rostopic echo -n 1 /h30/imu
rosservice call /controller_manager/list_controllers
```

预期：

- 参数依次是 `topic`、`false`、`false`。
- `/clock` 有消息，稍后再查询时数值变大；仿真会话保持 `/use_sim_time=true`。
- `/h30/imu` 的 frame 为 `base_imu`，姿态、角速度、加速度有有效数值。
- `controllers/joint_state_controller` 为 `running`。
- `controllers/legged_controller` 为 `initialized`，正在等待启动；不会选择 cheater 控制器。

检查物理串口接收频率，用墙上时间，避免 Gazebo 实时倍率影响统计：

```bash
rostopic hz --wall-time /h30/imu
```

本设备之前实测约 200 Hz。观察几秒后按 `Ctrl+C` 结束这条频率检查，不会关闭 H30 节点。
这时 `imu_used` 尚无消息是正常的，先继续到 4.3 和 4.5。

### 4.3 终端 C：同时观察原始姿态、控制器输入、估计姿态

以下临时脚本只订阅消息，不改配置、不发送运动命令。它把四元数转换为角度，约每个话题每秒打印两次：

```bash
python3 -B - <<'PY_IMU_VIEW'
import math
import time
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

last = {}


def show(label, q):
    now = time.monotonic()
    if now - last.get(label, -1e9) < 0.5:
        return
    last[label] = now
    roll = math.atan2(2.0 * (q.w*q.x + q.y*q.z),
                      1.0 - 2.0 * (q.x*q.x + q.y*q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w*q.y - q.z*q.x))))
    yaw = math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                     1.0 - 2.0 * (q.y*q.y + q.z*q.z))
    print('%-4s roll=%8.2f  pitch=%8.2f  yaw=%8.2f deg' %
          (label, math.degrees(roll), math.degrees(pitch), math.degrees(yaw)),
          flush=True)


rospy.init_node('h30_manual_view', anonymous=True)
subs = [
    rospy.Subscriber('/h30/imu', Imu,
                     lambda m: show('H30', m.orientation), queue_size=1),
    rospy.Subscriber('/controllers/legged_controller/imu_used', Imu,
                     lambda m: show('USED', m.orientation), queue_size=1),
    rospy.Subscriber('/odom', Odometry,
                     lambda m: show('EST', m.pose.pose.orientation), queue_size=1),
]
rospy.spin()
PY_IMU_VIEW
```

三种输出分别表示：

| 标签 | 意义 | 何时出现 |
| --- | --- | --- |
| `H30` | 节点输出的姿态，已应用配置中的安装旋转/航向偏置 | 传感器读取正常就有 |
| `USED` | 控制器实际交给 `updateImu()` 的姿态 | 启动控制器后 |
| `EST` | 当前 Kalman 估计器发布的机身姿态 | 控制器执行状态估计后 |

此时先只看到 `H30` 是预期行为。`USED` 和 `EST` 的 roll/pitch 应跟随 `H30`；输出时刻不同会有少量时间差，yaw 在 ±180° 附近换符号不表示数据中断。
`EST` 姿态跟随传感器不等于 Gazebo 机身真的转到了同一角度。

### 4.4 手动启动前：放平 IMU，建立这一轮的初始姿态

把 H30 放在稳定平面上，按输出 `base_imu` 的轴向作为虚拟机身方向：x 向前、y 向左、z 向上。
以设备轴标和安装配置为准，不凭 USB 插口方向判断轴向。

观察终端 C，让初始 **roll/pitch 尽量接近 0°，例如各在 ±5° 内**，静置几秒。
角速度应接近 0，加速度应主要在 +z 方向，模长约 9.8 m/s²。
如果静置时 roll 仍接近之前记录的 30°，先把 IMU 放平；若实际已经放平，则核对 `sensor_to_base_rpy` 的安装旋转。
修改安装参数后需要完整退出 A 再重新启动，因为节点在启动时读取配置。

本代码没有“启动时自动把当前姿态归零”。把传感器倾斜拿着并不会自动成为模型的水平初始姿态。
开始这一轮后先保持航向不变，先做 roll/pitch 实验；yaw 不必强行改成 0°。

### 4.5 终端 B：手动启动 Kalman 控制器

确认模型已自然趴稳、IMU 已放平，并且 A 已显示 ready 后，执行一次：

```bash
rosservice call /controller_manager/switch_controller "{start_controllers: ['controllers/legged_controller'], stop_controllers: [], strictness: 2, start_asap: true, timeout: 2.0}"
```

这个命令会启动现有的**起立控制**，不是只开启 IMU 显示。预期返回 `ok: True`。
`ok` 只表示切换请求成功；继续确认：

```bash
rosservice call /controller_manager/list_controllers
rostopic echo -n 1 /controllers/legged_controller/imu_used
```

预期控制器为 `running`，终端 C 开始出现 `USED` 和 `EST`。
若仍为 `stopped`，先查看 A 的错误；缺数据、frame/时钟不符或超时都会停止控制，不自动用仿真 IMU 顶替。

**这段起立过渡期间先不晃动 IMU。** 等待 A 依次出现：

```text
Stand-up transition started; MPC/WBC will engage after the posture settles.
Stand-up posture settled; handing over to MPC + WBC.
Waiting for the initial policy ...
Initial policy has been received.
```

前约 5 秒主要是关节 PD 起立过渡，之后满足稳定条件才进入 MPC/WBC；因此过渡时腿展开，不能视为 MPC 正在补偿手的晃动。
当前交接条件包括估计高度大于 0.24 m、roll/pitch 各小于 0.25 rad（约 14.3°）、关节就位并持续稳定 0.5 秒。
约 10 秒仍不满足则起立超时停止。
若本轮始终没有 `Initial policy has been received`，记录为“已验证 IMU 输入，但尚未进入 MPC/WBC”，不要宣称完成了平衡补偿实验。

### 4.6 人为晃动：先小角度、单轴、慢动作

`USED`/`EST` 出现后可以确认输入通路；需要观察 MPC/WBC 控制响应时，等上一步完成交接再做下面动作。
第一轮不发送 `/cmd_vel` 或 trot 指令，保持站立目标。

1. **静止基线**：保持 IMU 平放约 3 秒，记住 roll/pitch 和模型状态。
2. **模拟左右侧倾**：缓慢绕 x 轴转动，让 roll 改变约 3–5°，保持约 1 秒，再回到初始位置；看 `H30 → USED → EST` 是否同向变化。
3. **模拟前后俯仰**：回到基线后，再缓慢绕 y 轴转动，让 pitch 改变约 3–5°，保持约 1 秒，再返回。
4. 每次只改变一个轴，先做两三次。观察角速度在转动时有变化，停止转动后接近零，而倾角在保持倾斜时仍保留。

这里用“转动/倾斜”模拟机身晃动，不用快速甩动、敲击或大幅平移。平移会引入真实线加速度，而 Gazebo 足端仍在描述另一套运动，容易把位置/速度估计一起扰乱。

在 B 可以分别查看角速度、关节输出和 MPC 状态；每条连续查看命令用 `Ctrl+C` 结束后再执行下一条：

```bash
rostopic echo /controllers/legged_controller/imu_used/angular_velocity
rostopic echo -n 1 /joint_states
rostopic echo -n 1 /legged_robot_mpc_observation
```

角速度单位为 rad/s。MPC `state.value[9]` 是 yaw、`[10]` 是 pitch、`[11]` 是 roll，单位也是 rad；`[8]` 是估计高度。
使用 `rostopic echo` 时整个数组按一项一行显示，索引从 0 开始。
`/joint_states` 的 effort 是关节输出力矩读数；它发生变化可以帮助观察输出响应，但不能单凭力矩变化认定补偿成功。

若想对照虚拟机身的实际位置与姿态，B 执行：

```bash
rosservice call /gazebo/get_model_state "{model_name: 'colt', relative_entity_name: 'world'}"
```

这里返回的是 Gazebo 物理状态，终端 C 的 `EST` 是控制器根据外部 IMU 等输入形成的估计，两者可能明显不同。
手持 IMU 不会因虚拟机器人腿部补偿而自动回正，因此即使输入链路正确，模型也可能产生持续修正、漂移或倒下。
这一实验的有效结论是数据和控制响应通路是否工作；要测试真实抗扰闭环，应让全部传感器描述同一个运动机身，或在纯仿真中对模型施加物理扰动。

### 4.7 可选终端 D：保存这一轮输入、估计与输出

D 同样加载两个 setup，在开始动作前执行：

```bash
mkdir -p Imu_test/artifacts
rosbag record -O "Imu_test/artifacts/h30_hand_motion_$(date +%Y%m%d_%H%M%S).bag" \
  /clock /h30/imu /controllers/legged_controller/imu_used \
  /odom /joint_states /legged_robot_mpc_observation /legged_robot_mpc_ready
```

动作完成后在 D 按 `Ctrl+C`，等待 bag 关闭。记录包含传感器原始输入、实际使用的输入、估计和关节状态，便于对照手动动作的顺序。
ROS 节点占用串口期间不要再启动 CSV 串口采集工具。

### 4.8 终端 B：结束动作、停止控制

先将 IMU 放回初始平面并停止晃动，再执行：

```bash
rosservice call /controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['controllers/legged_controller'], strictness: 2, start_asap: true, timeout: 2.0}"
rosservice call /controller_manager/list_controllers
```

预期控制器为 `stopped`，原有命令被清零，模型可能自然趴下；H30 节点继续读取，所以 C 中 `H30` 仍有输出，而 `USED`/`EST` 停止更新。
接着在 C 按 `Ctrl+C` 退出观察脚本，在 D 停止录制，最后到 A 按 `Ctrl+C` 退出本轮。
下一轮建议完整重启，避免把上一轮的估计状态当作全新实验。

发生模型剧烈抖动、倒下、起立超时或 WBC 报错时，不继续增加晃动幅度；在 B 暂停物理并到 A 退出：

```bash
rosservice call /gazebo/pause_physics "{}"
```

停止服务无响应时直接退出 A。暂停后控制循环不再推进，不依赖再次切换控制器来完成退出。

### 4.9 常见现象与判断

| 现象 | 判断与处理 |
| --- | --- |
| `imu_used` 没有消息，提示 simulated time | 先看 `/clock` 和控制器状态；若时钟正常且为 `initialized`，按 4.5 手动启动后才会输出 |
| `/h30/imu` 正常，控制器一启动就 stopped | 看 A 的缺失/失效/过期输入错误，检查 frame、时钟、串口节点是否退出 |
| 有 `USED`/`EST`，但没有 MPC 初始策略日志 | 当前可能仍在 PD 起立过渡，尚未验证 MPC/WBC 的补偿响应 |
| IMU 静止但 roll/pitch 很大 | 先放平并核对安装旋转；启动时不会自动归零 |
| 起立超过约 10 秒后 stopped | 起立稳定条件未满足，保持传感器静止也未必能消除虚拟/真实运动不一致；保存日志后退出本轮 |
| 手一倾斜模型就明显修正甚至倒下 | 外部姿态与虚拟物理状态不一致；缩小动作可用于检查链路，但不能用放宽保护来证明稳定性 |
| yaw 从 +180° 跳到 -180° | 欧拉角显示的换边现象，结合四元数和连续动作判断 |
| 拔出 USB 或停掉 H30 后控制器停止 | 超时保护按预期工作；恢复数据后不会自动起立 |

`rostopic echo` 的 simulated time 提示只说明目标话题尚未收到消息，不能单凭它认定 `/clock` 已停止。
启动时没有有效输入也可能使首个控制更新立即停止，因此 `switch_controller` 的 `ok: True` 不等于 IMU 验收成功。

## 5. 恢复原仿真与清理

在启动终端按 `Ctrl+C`，等待本轮节点关闭。恢复原仿真：

```bash
roslaunch colt mpc.launch gui:=false
```

默认 `use_h30_imu=false`、`imu_source=hardware`、`cheater=true`，不会打开 USB 串口。
测试 rosbag 在 `Imu_test/artifacts/` 中，确认不再需要后自行清理；串口 ACL 如有本轮新增，按原 README 撤销。
本次代码没有安装系统驱动、修改用户组或添加系统 udev 规则。

## 本次验证记录

- `legged_hw`、`legged_controllers`、`colt` 已成功编译。
- 5 项 H30 C++ 测试、3 项话题输入测试、13 项原 Python 测试通过。
- ROS 伪串口端到端测试通过，覆盖单位、安装旋转与超时退出。
- 真实 H30 + 新 C++ ROS 节点读取通过：460800，约 3.005 秒收到 **602 条**消息，接收频率 **200.003 Hz**；frame 为 `base_imu`，数值均有限，归一化四元数模长约为 1。
- 实测统计文件：`Imu_test/artifacts/h30_ros_validation_20261107_204525.json`。
- `src/colt/test/h30_integration.py` 通过：真实 `LeggedController` 消费了伪串口送入的 H30 姿态和 rad/s 角速度，停止串流后进入 `stopped`。
- 已检查两种 launch 参数展开：默认使用硬件句柄且不启动 H30；开启 H30 后使用话题和 Kalman 控制器。
- 本次没有启动实体电机；传感器读取和仿真控制器输入验证通过，不代表实体机器人起立/行走闭环已验收。
