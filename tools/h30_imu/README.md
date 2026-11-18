# Jetson Orin Nano Super 8GB：UART1 读取 H30

适用：Jetson ARM64、Ubuntu 22.04、JetPack 6，H30 UART 460800 baud、8N1、无软/硬件流控。

`h30_read` 复用 `src/legged_control/legged_hw` 的 C++ 串口与 YESENSE 解析器，
可以在原生 Ubuntu 22.04 独立编译，无需 ROS、catkin、Gazebo、OCS2、pyserial 或 USB 转串口驱动。
本次提供传感器读取入口，整个 ROS 1 四足控制栈仍使用原有 Noetic 构建方式。

## 1. 接线与 UART 设备

以下接法针对 **NVIDIA 官方开发套件的 J12 40 针接口**；第三方载板应查该载板手册。
先断电接线，按板上针脚编号定位，不按照片朝向猜测。

| H30 的 TTL UART 信号 | Jetson J12 物理针脚 |
|---|---|
| TX | 10，UART1_RXD |
| RX | 8，UART1_TXD；本程序不发送数据，仅接收时可不接这一根 |
| GND | 6 或 9，GND |
| VCC | 按具体 H30 型号的供电规格连接，供电与信号电平分别确认 |

J12 UART 信号是 **3.3V 电平**，不能把 RS232、RS485 A/B 或 5V 信号直接接到 RX。
H30 MINI 的供电规格为 4.5–5.2V，H30WP 为 5–26V；不要把“3.3V UART 信号”误当成 H30 MINI 的供电要求。
H30 插座针序、线色与 TTL 输出电平以具体型号手册/原理图为准；需要时加电平转换。
供电和信号必须共地。这里只用 TX/RX/GND，不接 RTS/CTS。

NVIDIA 支持说明：40 针 UART1 在 JetPack 6 通常为 `/dev/ttyTHS1`，JetPack 5 曾为 `/dev/ttyTHS0`。
**UART1 的名称不保证 Linux 节点尾号恒定为 1。** 本程序默认 `/dev/ttyTHS1`，允许用 `--port` 覆盖，
不会因读不到数据而自动打开其他串口。

在 Jetson 上检查：

```bash
cat /etc/nv_tegra_release
ls -l /dev/ttyTHS*
python3 -B Imu_test/imu_test.py --list
dmesg | grep -E '3100000|ttyTHS|serial'
fuser /dev/ttyTHS1
```

`dmesg` 无权限时可在主机手动用 `sudo dmesg`；查看 `3100000.serial` / `uarta@3100000` 对应节点。
没有该节点时，检查载板型号、设备树和 pinmux。官方开发板可用
`sudo /opt/nvidia/jetson-io/jetson-io.py` 查看 40 针配置，按实际需要保存并重启。
程序不会修改设备树或系统串口服务。

如果权限不足，且设备所属组为 `dialout`，在 Jetson 主机执行：

```bash
sudo usermod -aG dialout "$USER"
```

随后注销并重新登录，用 `id` 确认 `dialout` 生效，再普通用户运行读取程序。
可参考 [旧 USB 测试文档](../../Imu_test/README.md) 使用临时 ACL 代替长期组权限。
不要把设备权限永久改成 777，也不要同时运行两个串口读取程序。
若 `fuser` 显示占用，先退出相应程序；只有确认 getty 服务占用了这个数据口后才调整对应服务，
不要把 `ttyTCU0` 调试口当成 UART1，也不要一律禁用系统控制台。

## 2. 独立编译和运行

下面命令均在项目根目录执行，无需加载 ROS 环境。

仅用 g++（最少依赖）：

```bash
mkdir -p build/h30_imu
g++ -std=c++14 -O2 -Wall -Wextra -Wpedantic -pthread \
  -I src/legged_control/legged_hw/include \
  src/legged_control/legged_hw/src/H30Imu.cpp \
  tools/h30_imu/h30_read.cpp -o build/h30_imu/h30_read
./build/h30_imu/h30_read --port /dev/ttyTHS1 --baud 460800
```

持续读取直到 `Ctrl+C`。终端约 5 Hz 显示，加速度为 m/s²（保留重力）、角速度为 rad/s、
欧拉角为 rad，四元数按 **x,y,z,w** 显示。字段只在本帧包含时显示，缺失字段不补旧值。
程序不发送设备配置、校准或保存命令。H30 自身必须已配置为 460800 且开启 UART 数据输出。

采集 30 秒，并保存每一帧（CSV 路径必须尚不存在，父目录需先创建）：

```bash
./build/h30_imu/h30_read --port /dev/ttyTHS1 --baud 460800 \
  --duration 30 --csv build/h30_imu/h30_capture.csv
```

重复采集请换文件名。CSV 保存主机接收时间、TID、加速度、角速度、四元数、欧拉角、温度和 `complete` 标志。
时间戳不是设备内部采样时间。CSV 无用户态缓冲，每条记录及时写出；`Ctrl+C` 保留已写数据并恢复串口参数，退出码为 130。
没有设备上报的欧拉角/温度时，对应列为空；不会从其他帧拼接，也不会把默认零值当成测量值。

也可以用 CMake：

```bash
cmake -S tools/h30_imu -B build/h30_imu -DCMAKE_BUILD_TYPE=Release
cmake --build build/h30_imu -j2
```

若缺编译工具，在 Jetson 安装 `build-essential`；CMake 路径另需 `cmake` 和用于测试的 `python3`。

## 3. 数据有效性与超时

| 参数 | 默认值 | 含义 |
|---|---|---|
| `--port` | `/dev/ttyTHS1` | 指定串口，不自动探测/切换 |
| `--baud` | `460800` | 同时支持 115200、230400、921600 |
| `--duration` | `0` | 0 表示持续读取；正数为墙上秒数 |
| `--startup-timeout` | `5` | 等待首个被接受的 IMU 帧 |
| `--data-timeout` | `1` | 已收数据后，IMU 帧断流的墙上超时 |
| `--csv` | 不保存 | 显式指定新 CSV 文件 |
| `--require-complete` | 关闭 | 只接受同帧包含 acc、gyro、quaternion 的数据 |

默认是诊断模式，允许设备仅输出欧拉角或部分 IMU 字段，便于先确认通信。
`complete=0` 不能用于替代完整控制输入。用于检查原控制器需要的数据时：

```bash
./build/h30_imu/h30_read --port /dev/ttyTHS1 --baud 460800 \
  --duration 10 --require-complete
```

如果诊断模式有数据、严格模式没有，检查上位机输出配置，确保 **0x10、0x20、0x41 在同一帧**。
现有 ROS 驱动始终保持严格模式，不接受诊断模式的残缺帧。无效四元数、坏校验和 TLV 越界会拒绝。

无数据、无有效 IMU 帧、持续只有温度、后续断流或串口错误均非零退出；
到达指定时长且收过有效 IMU 帧、没有发生超时才输出 `PASS`。超时使用单调时钟，不依赖 ROS `/clock`。

验收时静置观察：加速度模长约 9.8 m/s²，角速度接近 0，四元数模长约 1；
再绕明确轴缓慢转动，核对姿态和角速度方向。读取通过只证明通信及解析，精度、安装变换和整机控制需分别验证。
独立工具输出传感器原坐标；原 ROS 节点的 `sensor_to_base_rpy` / `world_yaw_offset` 不在此工具中应用。

原 Python 工具也可直接读 UART：

```bash
python3 -B Imu_test/imu_test.py --jetson-uart1 --baud 460800 --duration 10
```

注意其 CSV 保持旧接口：角速度/欧拉角是 **°/s、°**，四元数是 **w,x,y,z**，与新 C++ 工具明确区分。
它自动保存至 `Imu_test/artifacts/`；两种工具任选一种，不同时打开同一串口。

## 4. 原有 ROS 1 入口

已有可用 ROS 1 环境时，新增了仅启动 IMU 的 Jetson 配置：

```bash
roslaunch legged_hw h30_imu_jetson.launch port:=/dev/ttyTHS1 baud:=460800
```

发布 `/h30/imu`（`sensor_msgs/Imu`），安装旋转和超时沿用 `h30_imu.yaml`。
它仍是 catkin/ROS 1 节点；本次没有将四足控制栈迁移到 ROS 2，也不把此 launch 当成原生 ROS 2 入口。
不要在 Ubuntu 22.04 上直接照抄旧 Noetic 工作空间的 `source /opt/ros/noetic/setup.bash`。

## 5. 无硬件测试

先按第 2 节编译，然后：

```bash
python3 -B tools/h30_imu/test_h30_read.py build/h30_imu/h30_read
python3 -B -m unittest discover -s Imu_test -p 'test_imu_test.py'
```

CMake 构建后也可执行 `ctest --test-dir build/h30_imu --output-on-failure`。
测试用 Linux 伪终端向真实 C++ 读取程序发送独立构造的二进制帧，覆盖单位和顺序、拆包/粘包、坏帧、
可选字段、严格模式不跨帧拼接、超时、停止后 CSV 保留、不覆盖文件、串口设置恢复及不发送/回显数据。
这些测试不能替代 Jetson 针脚上的实际接收测试。

2026-11-18 本次验证：在 ARM64 环境以 g++ 编译通过（包含 `-Werror` 检查），
13 项 C++ 读取程序伪串口测试和 14 项 Python 回归测试通过。已识别本机为 Orin Nano Super / L4T R36.4.7。
主机 `/dev/ttyTHS1` 实际打开返回 `Permission denied`，免密码 sudo 不可用，因此尚未取得 H30 实物数据，
需要先在主机完成上述串口权限设置。当前工具环境未安装 CMake/ROS，未执行 CMake 构建或 ROS 节点测试。

## 官方依据

- [H30 官方资料及 Python 示例](https://github.com/Seeed-Projects/WHEELTEC-H30-IMU/blob/1983d788604d8b4b9a8b6714eef37387721b4662/6.Routines%20related%20to%20the%20inertial%20navigation%20module/3.Python%20routine/Yesense_Decode.py)：核对版本 `1983d788604d8b4b9a8b6714eef37387721b4662`。帧头 `59 53`、小端 TID/整数、TLV、双累加校验，以及温度、加速度、角速度、欧拉角和四元数字段；欧拉角线序是 pitch/roll/yaw。
- [Seeed H30 产品规格](https://wiki.seeedstudio.com/wheeltec_imu/)：型号供电、UART 接口与资料入口。
- [NVIDIA UART1 节点映射说明](https://forums.developer.nvidia.com/t/how-to-access-uart1-on-expansion-pins-8-10/328934/5)：JP5/JP6 节点差异及 `3100000` 控制器识别。
- [NVIDIA 开发套件载板规格下载入口](https://developer.nvidia.com/embedded/downloads)：`Jetson Orin Nano Developer Kit Carrier Board Specification`，3.3 节说明 J12 电平与针脚。
- [NVIDIA Jetson-IO 官方说明](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/HR/ConfiguringTheJetsonExpansionHeaders.html)：扩展口配置与设备树。
