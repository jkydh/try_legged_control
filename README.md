# Colt / legged_control

SolidWorks 四足模型接入 ROS Noetic、Gazebo、OCS2 SQP MPC 和 Weighted WBC。
当前默认流程：**打开 Gazebo → 无动力自然趴下 → 手动启动控制器 → 平滑起立 → MPC + WBC 保持站立**。
站稳后可以执行下文的 **trot 前进**，默认不会自动开始行走。

起立入口使用 legged_control 原有的 `/controller_manager/switch_controller` 服务。
原项目没有独立的趴姿恢复指令，因此本项目在 Colt 控制器启动阶段新增关节 PD 起立过渡；起立完成后才交接 MPC + WBC。
这不是 MPC 直接从任意倒地姿态恢复，也不支持侧翻、倒扣后的自动翻身。

## 自测指令流程

使用两个终端：**A 启动 Gazebo，B 发送起立命令与检查**。每个新终端都先加载同一工作空间环境。
无需单独运行 `roscore`，无需设置 `ROBOT_TYPE`，不需要安装软件或执行 sudo。
不要同时启动旧仿真、`display.launch`、其他 controller launch 或速度命令发布节点。

### 1. 终端 A：准备环境、增量编译

```bash
cd /home/ydh/Desktop/try_legged_control
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rospack find colt
rospack find legged_controllers
catkin build legged_interface legged_gazebo legged_estimation legged_wbc legged_controllers colt --no-deps -j2 -p1
source devel/setup.bash
check_urdf src/colt/urdf/colt.urdf
```

包路径应指向本工作空间；编译不能有 failed/abandoned；URDF 应成功解析，根 link 为 `base`。
`--no-deps` 使用当前已编译的 OCS2、Pinocchio、hpp-fcl 等依赖，不适用于从未构建依赖的新工作空间。

### 2. 终端 A：打开 Gazebo，等待自然趴下

```bash
roslaunch colt mpc.launch cheater:=true
```

没有图形界面时，将上面一条替换为（不要同时运行两条）：

```bash
roslaunch colt mpc.launch cheater:=true gui:=false
```

默认 `start_control:=false`，不会自动起立。
启动器先在暂停状态生成低姿态模型、设置初始关节并加载控制器；CppAD 编译通常需要几十秒，期间不要手动播放或重置。
加载结束后自动启动关节状态发布、解除物理暂停，机器人关节没有主动控制力矩，会靠重力自然落稳。

等待终端 A 出现：

```text
Colt ready and unpowered; start controllers/legged_cheater_controller via /controller_manager/switch_controller to stand up
```

再观察几秒，确认机器人已自然趴稳，尚未起立。此时 `colt_bringup` 正常退出属于预期，Gazebo 继续运行。

### 3. 终端 B：检查等待起立状态

```bash
cd /home/ydh/Desktop/try_legged_control
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosservice call /controller_manager/list_controllers
rosservice call /gazebo/get_model_state "{model_name: 'colt', relative_entity_name: 'world'}"
rostopic echo -n 1 /joint_states
```

预期：

- `controllers/joint_state_controller` 为 `running`。
- `controllers/legged_cheater_controller` 已加载但尚未启动，通常为 `initialized`（曾停用时也可能为 `stopped`）。
- Gazebo 返回 `success: True`，机身实际高度明显低于 0.284 m，机器人处于自然落稳的低姿态。
- 关节主动输出力矩接近 0；此时 MPC 尚未运行，没有 MPC observation 是正常现象。

### 4. 终端 B：使用原生控制器切换命令起立

```bash
rosservice call /controller_manager/switch_controller "start_controllers: ['controllers/legged_cheater_controller']
stop_controllers: []
strictness: 2
start_asap: true
timeout: 5.0"
```

预期返回 `ok: True`。这代表启动请求成功，**不代表起立过程已经完成**。
不需要另发 `/cmd_vel`、不需要重复启动关节状态控制器，也不要启动两个 legged 控制器。
同一轮只需触发一次；若控制器已经 `running`，不要重复发送启动请求。

观察终端 A 的顺序：

```text
Stand-up transition started; MPC/WBC will engage after the posture settles.
Stand-up posture settled; handing over to MPC + WBC.
Waiting for the initial policy ...
Initial policy has been received.
```

机器人先用约 5 秒的平滑关节轨迹从当前姿态展开到站立目标。
达到关节误差、机身高度和倾角要求并持续稳定 0.5 秒后，才启用 MPC + WBC；过渡超过 10 秒仍未就绪会停止并报错。
起立动作依靠关节力矩，不会通过修改 Gazebo 位姿把机器人抬起来。

### 5. 终端 B：检查站立结果

等出现 `Initial policy has been received`、机器人已站稳后执行：

```bash
rosservice call /controller_manager/list_controllers
rostopic echo -n 1 /legged_robot_mpc_observation
rostopic echo -n 1 /joint_states
rosservice call /gazebo/get_model_state "{model_name: 'colt', relative_entity_name: 'world'}"
```

预期关节状态控制器和所选 legged 控制器均为 `running`。
MPC 的 `state.value`、`input.value` 各有 24 项且无 `nan/inf`；`state.value[8]` 是估计基座高度，`[10]`、`[11]` 是俯仰和侧倾角（rad）。
估计高度与 Gazebo 返回的 `pose.position.z` 都应接近 0.284 m，稳定四足接触模式通常为 15，关节最终输出力矩不超过 20 N·m。

可检查数据连续性，观察几秒后 `Ctrl+C` 结束这条检查：

```bash
rostopic hz /legged_robot_mpc_observation
```

### 6. 终端 B：20 秒站立验收

**起立过渡结束后再执行**。以下命令只读取状态，不创建脚本文件，不发送运动命令。
它采集 20 秒仿真时间（最多等待 120 秒墙上时间），核对稳定高度、姿态、力矩、实际 Gazebo 高度与控制器状态。

```bash
python3 -B - <<'PY_CHECK'
import math
import time
import rospy
from controller_manager_msgs.srv import ListControllers
from gazebo_msgs.srv import GetModelState
from ocs2_msgs.msg import mpc_observation
from sensor_msgs.msg import JointState

rospy.init_node('colt_standing_check', anonymous=True)
samples = []
effort = [0.0]
errors = []


def observe(msg):
    state = list(msg.state.value)
    control = list(msg.input.value)
    if len(state) != 24 or len(control) != 24:
        errors.append('MPC state/input dimension is not 24')
        return
    if not all(math.isfinite(v) for v in state + control):
        errors.append('MPC state/input contains NaN or Inf')
        return
    samples.append((msg.time, state[8], state[10], state[11]))


def joint_state(msg):
    if len(msg.name) != 12 or len(msg.effort) != 12:
        errors.append('JointState does not contain 12 joints and efforts')
        return
    if not all(math.isfinite(v) for v in msg.position + msg.velocity + msg.effort):
        errors.append('JointState contains NaN or Inf')
        return
    effort[0] = max(effort[0], max(abs(v) for v in msg.effort))


rospy.Subscriber('/legged_robot_mpc_observation', mpc_observation, observe, queue_size=1000)
rospy.Subscriber('/joint_states', JointState, joint_state, queue_size=100)
deadline = time.monotonic() + 120.0
while not rospy.is_shutdown() and time.monotonic() < deadline:
    if errors or (samples and samples[-1][0] - samples[0][0] >= 20.0):
        break
    time.sleep(0.1)

try:
    assert not errors, errors[0] if errors else ''
    data = list(samples)
    assert data and data[-1][0] - data[0][0] >= 20.0, 'Insufficient data: 20 simulation seconds required'
    assert all(0.20 < s[1] < 0.35 and max(abs(s[2]), abs(s[3])) < 0.15 for s in data), 'Startup height/attitude out of range'
    steady = [s for s in data if s[0] >= data[-1][0] - 5.0]
    assert all(0.27 < s[1] < 0.30 and max(abs(s[2]), abs(s[3])) < 0.05 for s in steady), 'Standing height/attitude not settled'
    assert 0.0 < effort[0] <= 20.0 + 1e-9, 'No joint effort received, or effort exceeds 20 Nm'
    rospy.wait_for_service('/controller_manager/list_controllers', timeout=5.0)
    controllers = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)().controller
    running = {c.name for c in controllers if c.state == 'running'}
    expected = 'controllers/legged_cheater_controller' if rospy.get_param('/colt_bringup/cheater') else 'controllers/legged_controller'
    assert expected in running and 'controllers/joint_state_controller' in running, 'Expected controllers are not running'
    rospy.wait_for_service('/gazebo/get_model_state', timeout=5.0)
    actual = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)('colt', 'world')
    assert actual.success and 0.27 < actual.pose.position.z < 0.30, 'Gazebo actual body height out of range'
    print('PASS: 20 simulated seconds of standing')
    print('Last 5 s estimated height: %.6f .. %.6f m' % (min(s[1] for s in steady), max(s[1] for s in steady)))
    print('Gazebo actual height: %.6f m' % actual.pose.position.z)
    print('Peak joint effort: %.3f Nm' % effort[0])
except (AssertionError, rospy.ROSException, rospy.ServiceException) as exc:
    print('FAIL:', exc)
    raise SystemExit(1)
PY_CHECK
```

预期出现 `PASS`。该脚本验收的是起立后的保持站立；同时还需通过步骤 2–4 观察“无动力趴下 → 命令起立”的完整过程。
`FAIL` 只退出检查脚本，不会关闭仿真；按下一步暂停并退出，保留终端 A 的报错。

### 7. 结束或异常停止

正常结束：在终端 A 按 `Ctrl+C`，等待 Gazebo 和本轮 ROS 节点退出。
出现剧烈抖动、侧翻、起立超时或 WBC 报错时，可先在终端 B 暂停物理，再到 A 按 `Ctrl+C`：

```bash
rosservice call /gazebo/pause_physics "{}"
```

服务无响应时直接在 A 退出本轮。不要用 Gazebo Reset 复用旧的控制状态；重新测试应完整退出后重启。

### 8. 第二轮：Kalman 估计下手动起立

确认第一轮已完全退出。终端 A：

```bash
roslaunch colt mpc.launch cheater:=false
```

无界面时加 `gui:=false`。等待 `Colt ready and unpowered`，观察自然趴稳。
终端 B 检查 `controllers/legged_controller` 已加载且为 `initialized` 或 `stopped`，然后执行：

```bash
rosservice call /controller_manager/switch_controller "start_controllers: ['controllers/legged_controller']
stop_controllers: []
strictness: 2
start_asap: true
timeout: 5.0"
```

等待起立过渡结束、MPC 初始策略就绪，再重复步骤 5、6，最后按步骤 7 退出。
两轮的唯一控制器选择差别：真值模式用 `legged_cheater_controller`，Kalman 模式用 `legged_controller`。

### 常见现象

| 现象 | 处理 |
|---|---|
| 刚启动时暂停、正在生成 CppAD 库 | 等待加载完成，不手动解除暂停 |
| 已自然趴下但没有起立 | 默认等待手动 `switch_controller` 命令，属于正常状态 |
| 服务返回 `ok: True`，尚未出现 MPC 策略 | 起立过渡仍在进行，先观察动作和阶段日志 |
| `Failed to set threads priority` | 本次不调整系统权限，继续检查实际动作与站立 |
| KDL 根 link 惯量提示 | 可视化链路提示，Pinocchio 仍读取惯量 |
| 包路径错误、找不到包 | 在当前终端重新加载两个 setup，核对 `rospack find` |
| 模型已存在或控制器服务冲突 | 完整退出上一轮，不同时运行两套仿真 |
| 起立超时、`WBC QP failed`、安全检查失败或控制器 stopped | 判为失败，暂停并退出；保留报错，不绕过保护 |
| GUI 无法打开 | 使用 `gui:=false`，通过状态和 Gazebo 服务验收 |
| 120 秒没有收齐 20 秒仿真数据 | 检查 `/clock`、控制器及终端 A；运行正常但较慢时适当增大检查超时 |

以上步骤用于起立和静态站立验收，不发送行走速度；完成后可继续下面的 trot 前进流程。
`start_control:=false` 现在表示“解除暂停、自然趴下、等待手动起立”，不再表示始终暂停。
如需自动触发同一段起立流程，可显式设置 `start_control:=true`；默认仍为手动触发。

## trot 前进

先按上文启动仿真、手动起立，等待 `Initial policy has been received`。
保持终端 A 的 Gazebo 运行，在已加载工作空间环境的终端 B 执行：

```bash
rosrun colt trot_forward.py --speed 0.08 --duration 10
```

`--duration` 是匀速段的仿真秒数，`--speed` 单位为 m/s，沿机器人当前朝向前进。
脚本依次完成：

1. 等待 `/legged_robot_mpc_ready` 为真，并连续检测 2 秒稳定四足站立。
2. 调用 `/colt_gait/set_trot` 切换 trot，以零速度等待 2 秒，让新步态在当前 MPC 预测区间之后生效。
3. 用默认 2 秒从零加速到 0.08 m/s，再保持 10 秒。
4. 用 2 秒减速到零，再保持零速度 2 秒。
5. 切换回 stance，继续发送零速度 3 秒，检查站立和速度收敛后退出。

正常结束应出现 `Returned to stance` 和估计前进位移。整个过程约 23 秒仿真时间，墙上耗时取决于仿真速度。
此时仍是原来的 legged 控制器在运行，无需重新调用 `switch_controller`。
可再次运行脚本完成下一段前进。

参数示例：

```bash
rosrun colt trot_forward.py --speed 0.10 --duration 15 --ramp 3
```

当前脚本限制速度为 `(0, 0.15] m/s`、加减速时间不小于 1 秒；默认等待站稳的墙上超时为 120 秒。
行走期间在终端 B 按 **Ctrl+C**，脚本会先减速并回到 stance，再退出；不要紧接着关闭终端 A。
如果出现侧翻、控制器停止或求解错误，按前文异常停止步骤暂停物理并退出。
不要同时运行其他 `/cmd_vel` 发布程序、步态命令或导航节点。

### 控制方式与步态参数

行走继续使用状态估计 → SQP MPC → Weighted WBC → 混合关节接口，无需另一个关节轨迹控制器。
起立阶段的 PD 逻辑和目标站姿保持原样。

| 项目 | 当前配置 |
|---|---|
| trot 支撑顺序 | LF + RH（模式 9）→ RF + LH（模式 6）循环 |
| 周期 | 0.6 秒，每个对角支撑相位 0.3 秒；无主动腾空相位 |
| 摆腿抬高 | 相对接触位置 0.04 m |
| 地面与足端坐标 | 物理地面 z=0；足端坐标在球心，接触目标高度为 0.019 m |
| 目标机身高度 | 0.284 m |
| 速度命令频率 / 超时 | 脚本约 20 Hz；0.5 秒仿真时间无新指令则请求零速度 |

步态周期来自 `legged_controllers/config/colt/gait.info`，服务直接读取该文件，没有在脚本中复制一份周期。
`task.info/swing_trajectory_config/terrainHeight` 表示接触时足端坐标的高度；这里包含 0.019 m 球半径，不是把 Gazebo 地面抬高。
只有 Colt 配置使用该偏移，未配置时其他机器人仍使用原来的零高度。

### 使用原生接口手动控制

也可以在站稳后手动切换步态并持续发送速度；与上面的脚本二选一。

```bash
rosservice call /colt_gait/set_trot "data: true"
rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.08, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

这组原生命令没有脚本的加减速过程。停止 `rostopic pub` 后，速度超时机制会在 0.5 秒后请求零速度。
等待约 2 秒仿真时间，再执行：

```bash
rosservice call /colt_gait/set_trot "data: false"
```

步态服务返回成功仅表示模板已发送，约一个 MPC 预测时域后才开始生效。
起立尚未交接 MPC/WBC 时，trot 请求会被拒绝，速度命令也会被忽略。
速度超时只请求零速度，机器人仍可能原地踏步；需要上述 `false` 请求才能回到四足 stance。

### 自动化仿真验收

`src/colt/test/trot_integration.py` 会发送控制指令，检查真实 Gazebo 位移、两组对角支撑、四足离地、
状态有限性、姿态、20 N·m 瞬时力矩限制和结束后的站立状态。
同时读取 `/torque_diagnostics`：其峰值在每次执行更新时累计，约 100 Hz 发布，避免仅用 100 Hz 关节状态采样漏检瞬态。
在新启动且已自然趴稳的仿真中，终端 B 执行以下命令；它会代替上文的手动起立命令，并运行一轮前进：

```bash
python3 -B src/colt/test/trot_integration.py --start-controller --speed 0.08 --duration 10
```

若已经起立，去掉 `--start-controller`。不要同时运行前进脚本或原生速度命令。
测试末尾应输出 `PASS: trot forward ...`。中断与速度源丢失也可分别验证：

```bash
python3 -B src/colt/test/trot_integration.py --speed 0.10 --duration 10 --interrupt-after 9
python3 -B src/colt/test/trot_integration.py --speed 0.08 --duration 10 --kill-after 9
```

第二条会强制结束它自己启动的前进脚本，检查速度超时已请求零速度，再主动切回 stance。
更换估计模式时，先完整关闭本轮仿真，再用 `cheater:=false` 重新启动并重复验收。

### 20 N·m 瞬时力矩保护

当前所有 12 个关节的最终驱动力矩硬限制为 **±20 N·m**，包括起立、行走、步态切换和停车。
该限制针对电机执行指令，不是地面接触冲击产生的全部被动机械载荷。

旧配置达到 30 N·m 的代码原因包括：URDF 和 WBC 本身允许 30 N·m；起立 PD 不经过 WBC；
WBC 只约束前馈力矩，而执行端还会叠加速度反馈力矩。
在保留旧起立参数、但已经启用 20 N·m 硬保护的复测中，右后髋部 HAA 在起立后约 0.37 秒出现
约 −8.91 rad/s 的反馈角速度，位置项约 3.90 N·m、速度项约 26.52 N·m，总请求约 30.42 N·m。
这是起立 PD 的瞬态请求；本次实际输出已被截为 20 N·m。
原速度反馈由角度差分得到，对接触过程中的位置变化敏感，因此记录中的反馈速度不应直接当作独立实测的真实电机转速。
起立轨迹现已从 3 秒放慢到 5 秒，`Kp` 从 60 降为 40，`Kd` 从 3 降为 1.5，减少起立时的冲击请求。

当前保护分三层：

- `colt.urdf` 的 12 个驱动关节 `effort=20`。
- WBC 为速度反馈预留力矩预算，约束 `−20 ≤ τ前馈 + τ速度反馈 ≤ 20`；保留原有名义前馈动力学和反馈作用。
- `LeggedHWSim` 在发送命令前，再对 **位置 PD + 速度 PD + 前馈之和** 限幅，取 URDF 与 `gazebo/effort_limit` 中更小的值；非有限请求输出零力矩。

查看诊断：

```bash
rostopic echo -n 1 /torque_diagnostics
```

`requested_effort` 是限幅前请求，`applied_effort` 是最终发送值，`measured_effort` 是 Gazebo 执行反馈。
`peak_applied_effort`、`peak_measured_effort` 从本轮仿真初始化开始逐执行周期累计，均不得超过 20 N·m。
`saturation_count` 统计发生最终限幅的次数；`peak_phase` 为 0（未驱动）、1（起立 PD）、2（MPC/WBC），
`peak_time`、`peak_position_term`、`peak_velocity_term`、`peak_feedforward` 对应各关节最大请求时刻。
已有模型实例不会自动更新 URDF 限制；修改后需重新编译并完整退出、重新启动仿真。

执行端限幅有独立边界测试，覆盖正负叠加超限、极小超限、正常命令和非有限命令输出零：

```bash
catkin build legged_gazebo --no-deps -j2 -p1 --make-args legged_torque_limit_test
rosrun legged_gazebo legged_torque_limit_test
```

### 20 N·m 配置复测结果（当前）

最终起立参数为 5 秒、Kp=40、Kd=1.5。真值和 Kalman 模式分别通过 0.08 / 0.15 m/s 前进、
Ctrl+C 中断和速度源丢失后停车，共 8 项仿真验收；两种模式各从无动力趴姿完成一次起立。

| 估计模式 | 所有执行周期累计输出峰值 / N·m | Gazebo 执行反馈峰值 / N·m | 最终限幅次数 |
|---|---|---|---|
| 真值 | 16.3984 | 16.3984 | 0 |
| Kalman | 19.2565 | 19.2565 | 0 |

累计检查 188760 个执行周期，没有出现最终输出超过 20 N·m，也没有依靠持续限幅完成动作。
0.08 m/s 指令含加减速时，真值 / Kalman 实际前进约 0.9634 / 1.0226 m；
0.15 m/s 时约 1.7449 / 1.8692 m。各项结束均回到四足站立。
执行端的 4 项边界单元测试全部通过。复测使用相同动力学模型的 CppAD 缓存，正式配置仍强制重编译。
仿真已关闭，根因复现数据和逐关节峰值见 [torque_limit_validation_2026-10-29.json](src/colt/test/results/torque_limit_validation_2026-10-29.json)。

### 2026-10-29 首轮仿真验证记录（历史，旧 30 N·m 配置）

以下记录保留用于追溯，不是当前 20 N·m 配置的验收结果。旧配置允许 30 N·m，现已停用。

相关包编译通过。两种估计模式均从无动力趴姿手动起立，然后验证 trot 前进、减速与回到 stance。
下表的匀速段均为 10 秒、加减速各 2 秒，实际位移包含加减速段，来自 Gazebo 世界坐标下的真实位姿。

| 估计模式 | 指令速度 / m/s | 实际前进 / m | 横向偏移 / m | 最大俯仰或侧倾 / rad | 结束实际高度 / m |
|---|---|---|---|---|---|
| 真值 | 0.08 | 0.9478 | −0.0060 | 0.01035 | 0.28513 |
| 真值 | 0.15 | 1.7435 | −0.0073 | 0.00943 | 0.28529 |
| Kalman | 0.08 | 1.0038 | −0.0085 | 0.01034 | 0.28499 |
| Kalman | 0.15 | 1.8694 | −0.0013 | 0.00982 | 0.28515 |

四项均检查到实际接触模式 9、6 交替，四足球心最大高度约 0.0585–0.0589 m，结束为模式 15，
最终实际水平速度小于 0.0003 m/s，状态均有限且关节力矩未超过 30 N·m。
个别瞬态达到 30 N·m 限幅，不能据此推断实机驱动力有余量；Kalman 的估计位移也不等同于实际位移。

另外通过了真值模式下的两项异常路径测试：0.10 m/s 行走中发送 Ctrl+C 后平滑减速并站立；
0.08 m/s 行走中强制结束速度发布程序后，检测到超时零速度目标，再通过步态服务恢复 stance。
全部测试已关闭仿真。数值记录保存于 [trot_validation_2026-10-29.json](src/colt/test/results/trot_validation_2026-10-29.json)。

## 模型和参数约定

| 对象 | 当前约定 |
|---|---|
| URDF | `src/colt/urdf/colt.urdf`，Gazebo 与 MPC 使用同一文件 |
| 基座 / IMU link | `base` / `base_imu` |
| 驱动关节向量 | LF、LH、RF、RH，每条腿 HAA、HFE、KFE |
| 接触力与足端向量 | LF、RF、LH、RH，足端 link 为 `*_FOOT` |
| 足端固定关节 | `*_foot_fixed`，与足端 link 分开命名 |
| 生成时低姿态 | 基座高度 0.08 m，每条腿 `[0, 1.4, -2.6]` rad，随后无动力自然落稳 |
| 起立目标关节角 | 每条腿 `[0, 0.72, -1.44]` rad |
| 起立过渡 | 5 秒平滑关节 PD（Kp=40、Kd=1.5）；姿态稳定 0.5 秒后交接 MPC + WBC |
| 基座参考高度 | 0.284 m，`reference.info` 的 `comHeight` 实际作用于基座原点 |
| 足端半径 / 最终力矩上限 | 0.019 m / 20 N·m（与 URDF 和执行端一致） |
| CppAD 模型目录 | `/tmp/legged_control/colt_v1` |

质量、惯量、几何和驱动关节坐标轴保留导出数据；基座、IMU、足端标识已适配，网格文件名保持原样。
Gazebo IMU 句柄和采样 link 均为 `base_imu`；保留 CAD 中相对基座的 0.0215 m 安装偏移。估计器没有补偿该平移偏移，动态运动前需进一步评估。
控制器按 Pinocchio 实际广义坐标索引获取关节句柄，不再假定硬件句柄的排列顺序。
Gazebo 对有限位转动关节直接读取实际角度分支，避免深度弯曲时反馈多出 `2π`、导致起立 PD 选错转向。

参数入口：

- `src/colt/config/initial_pose.yaml`：生成模型时的低姿态，区别于站立目标。
- `src/colt/config/gazebo.yaml`：IMU、接触与执行延迟。
- `src/legged_control/legged_controllers/config/colt/task.info`：MPC、WBC、自碰撞与估计参数。
- 同目录 `reference.info`：默认关节角与目标高度；`gait.info` 提供 stance 和 trot。

`initial_pose.yaml` 定义无动力生成姿态；`task.info/initialState` 与 `reference.info/defaultJointState` 定义 MPC 名义站姿，两者不要求相同。
改变站立目标时，应同步核对这两份 `.info` 中的姿态和高度；`task.info/standup` 配置过渡时间与 PD 增益。
单独使用 `gazebo.launch` 时，其 `spawn_model -J` 参数也需同步。重新从 SolidWorks 导出会覆盖适配内容，需要重新核对命名、transmission 和 Gazebo 插件。

## 检查与限制

```bash
check_urdf src/colt/urdf/colt.urdf
rosservice call /controller_manager/list_controllers
rostopic echo /legged_robot_mpc_observation
rostopic echo /joint_states
```

站立时应有 24 维 MPC 状态，基座高度在 `state.value[8]` 附近，目标约 0.284 m；四足接触模式为 15。
站立稳定前不要发布移动速度或切换动态步态。当前只配置平地低速直线 trot，未验证转弯、复杂地形或实机。
机械关节限位仍保留导出值 ±3.14 rad。软件已将最终驱动力矩限制为 20 N·m，机械限位与速度上限仍需结合实际电机与结构确认。
WBC 求解失败或检测到非有限状态时会停止控制并清零命令；这不保证失去控制后机器人仍能站立。
线程实时优先级设置失败时使用普通调度，本次不修改系统权限。KDL 的根 link 惯量提示不影响 Pinocchio 读取该惯量。

## 此前手动起立验证记录（历史，旧 30 N·m 配置）

2026-10-28：相关包编译、默认参数、launch/Python 语法检查通过。
两种估计器分别验证：关节无动力自然落稳 5 秒 → 调用上述原生切换服务 → 平滑起立 → MPC/WBC 接管。
服务触发后连续验证约 25 秒仿真时间，其中起立过渡约 3.5 秒，其后保持站立超过 20 秒。

| 模式 | 无动力落稳高度 / m | 最后 5 秒估计高度 / m | 最终实际高度 / m | 最后 5 秒最大侧倾或俯仰 / rad | 起立及保持全程峰值力矩 / N·m |
|---|---|---|---|---|---|
| 真值估计 | 0.002000 | 0.283734–0.283826 | 0.283746 | 0.002047 | 30.00 |
| Kalman 估计 | 0.002000 | 0.283871–0.283937 | 0.283860 | 0.001444 | 21.33 |

两项均通过，测试仿真已关闭。高度指 URDF `base` 原点的世界高度，不是机身顶面高度。
真值模式起立瞬态达到 30 N·m 限幅，尚不能据此认定实机驱动力已有余量。
复测仅复用相同动力学模型的 CppAD 缓存，正式配置仍强制重编译；未验证动态步态、任意侧翻恢复或实机。

## 此前直接站姿启动的验证记录（历史基线）

以下是改为手动起立前的测试记录，不能单独证明新的趴姿起立流程通过。

2026-10-28：URDF 解析、名称/配置对应、名义姿态足端正运动学和相关包增量编译通过。
名义姿态足球最低点距地面约 0.224 mm，总质量 10.29417088 kg。
最终 Gazebo 设置采用足端接触刚度 1e6、阻尼 100、ODE 500 次迭代；Colt 的 WBC 工作集迭代预算为 100。

两种估计器分别从启动起连续采样 20 秒仿真时间，检查 24 维状态有限、控制器持续运行、机身高度/姿态和力矩上限；最后 5 秒结果：

| 模式 | 估计高度范围 / m | Gazebo 最终实际高度 / m | 最大侧倾或俯仰 / rad | 全程峰值力矩 / N·m |
|---|---|---|---|---|
| 真值估计 | 0.284300–0.284323 | 0.284302 | 0.000780 | 4.83 |
| Kalman 估计 | 0.283466–0.283528 | 0.283451 | 0.000915 | 4.63 |

两项均通过，测试仿真已关闭。最终复测复用了相同动力学模型的 CppAD 缓存，正式配置仍强制重编译。
这些结果仅覆盖当前平地静态站立，不代表动态步态、外力扰动或实机已验证。
