# H30 mini USB IMU 读取测试

独立于 ROS/catkin 的 Linux 串口测试，默认 **460800 baud、8N1、无流控**。
仅依赖 Python 3 标准库，不需要 `pip install`、虚拟环境或编译整个机器人项目。
代码只接收数据，不发送校准、配置、保存或运动命令。打开串口会设置主机端串口参数，退出时尝试恢复原参数并关闭文件描述符。

## 1. 本次主机检查结果

2026-11-07 检查记录：

| 项目 | 结果 |
| --- | --- |
| 系统 | Ubuntu 20.04.6 LTS / x86_64 |
| 内核 | `5.15.0-139-generic` |
| USB | `1a86:55d4`，QinHeng USB Single Serial（CH9102） |
| 设备节点 | `/dev/ttyACM0` |
| 稳定路径 | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B31016849-if00` |
| 当前驱动 | `cdc_acm`，已经绑定 |
| 权限 | `root:dialout`、`0660`；当前用户 `ydh` 不在 `dialout` 组 |
| 实际打开 | `PermissionError: [Errno 13] Permission denied` |
| 数据读取结论 | **尚未完成实机读取；当前阻塞是权限，不能判定为驱动不支持** |

沁恒说明 CH9102 支持 Linux 自带的 CDC-ACM 驱动，详见[官方驱动说明](https://github.com/WCHSoftGroup/ch343ser_linux)。因此应先完成下面的临时授权和读取测试；成功后无需另装驱动。
本次没有安装软件、加载厂商模块、修改用户组或写入系统规则。

## 2. 先使用现有驱动测试

以下命令在**主机终端**执行，路径或序列号变化时以实际设备为准。
先关闭其他占用同一串口的上位机或 ROS 驱动。

```bash
cd /home/ydh/Desktop/try_legged_control
python3 -B Imu_test/imu_test.py --list
ls -l /dev/serial/by-id/
fuser /dev/ttyACM0
```

`fuser` 没有输出且返回非零，通常表示没有发现占用；它受进程查看权限限制。

### 2.1 临时授予当前用户访问权限

使用 ACL，仅针对当前设备，不修改 `dialout` 成员关系；拔插后授权通常失效。
`setfacl` 来自 `acl` 包，本机已存在。`sudo` 密码仅在主机终端输入。

```bash
cd /home/ydh/Desktop/try_legged_control
mkdir -p Imu_test/artifacts
H30_PORT=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B31016849-if00
getfacl -p "$H30_PORT" > Imu_test/artifacts/tty-before.acl
sudo setfacl -m "u:$(id -un):rw" "$H30_PORT"
python3 -B Imu_test/imu_test.py --port "$H30_PORT" --baud 460800 --duration 50
```

备份 ACL 的命令在同一轮授权前执行一次，避免覆盖授权前的状态。
若提示设备不存在，重新执行 `--list` 并更新 `H30_PORT`。

### 2.2 输出与成功条件

程序约每 0.2 秒显示一帧，持续接收和解析所有字节：

```text
OPEN: ... 460800 baud, 8N1, no flow control; receiving 10.0 s
tid=...  acc_m_s2=(...)  gyro_deg_s=(...)  rpy_deg=(...)
SUMMARY: bytes=... valid_frames=... imu_frames=... checksum_errors=... ...
Fields seen: ...
PASS: 已读取并校验 IMU 数据（仅验证通信与解码，不代表精度或标定验收）。
```

上面是输出格式示意，**不是实机成功记录**。
`PASS` 要求至少收到 3 个通过校验、TLV 结构完整且包含已支持 IMU 字段的帧。
静置时角速度应接近零，加速度模长通常接近重力加速度；缓慢转动时观察姿态变化。
这一步验证通信和解码；输出频率统计包含所有有效协议帧，不是传感器内部采样率，也没有做精度验收。

| 字段 | 顺序及单位 |
| --- | --- |
| `acc_m_s2` | x、y、z，m/s² |
| `gyro_deg_s` | x、y、z，°/s |
| `rpy_deg` | roll、pitch、yaw，°；已调整协议的 pitch、roll、yaw 顺序 |
| `quat_wxyz` | w、x、y、z，无量纲 |
| `temp_C` | 摄氏度 |

仅显示当前帧实际包含的字段。设备输出配置不同，字段可能不齐全；未知 TLV 会按长度跳过。
协议依据为[H30 资料仓库的 YESENSE Python 示例](https://github.com/Seeed-Projects/WHEELTEC-H30-IMU/blob/main/6.Routines%20related%20to%20the%20inertial%20navigation%20module/3.Python%20routine/Yesense_Decode.py)：`59 53` 帧头、小端 TID、1 字节负载长度、TLV 负载和双字节累加校验。

### 默认保存 CSV

每次采集会自动创建 `Imu_test/artifacts/imu_<UTC时间>.csv`，例如 `imu_20261107T103000_123456Z.csv`。
默认目录相对于脚本位置，和运行命令时所在目录无关；终端的 `CSV:`、`SAVED:` 会打印实际绝对路径。
每个通过校验和 TLV 解析的帧保存一行，包含时间、TID、加速度、角速度、欧拉角、四元数及温度；当前帧未提供的字段留空。
**保存所有有效帧，不受终端每 0.2 秒显示一次的限制。** 坏帧不写入 CSV。

- `time_unix_s`：主机读取这一批串口字节时的 Unix 时间（秒）。
- `elapsed_s`：从开始采集到读取这一批字节的单调时钟时间（秒）。
- 同一批字节中解码的帧共享接收时间；这不是 IMU 的内部采样时间。
- 每批数据写入后刷新文件，正常结束、`Ctrl+C` 或串口异常时关闭文件并保留已写入记录。
- 未收到有效帧时 CSV 只有表头；无法打开串口时不创建 CSV。

指定文件名并采集 30 秒（自动创建父目录，文件已存在则报错，不覆盖）：

```bash
python3 -B Imu_test/imu_test.py --port /dev/ttyACM0 --duration 30 \
  --csv Imu_test/artifacts/my_imu.csv
```

CSV 列名中的单位与上表一致，x/y/z 及四元数分量各占一列，可直接用表格软件或 Python 分析。

可选：保留原始数据及完整终端输出。原始文件必须尚不存在，程序不会覆盖旧文件。

```bash
cd /home/ydh/Desktop/try_legged_control
mkdir -p Imu_test/artifacts
set -o pipefail
python3 -B Imu_test/imu_test.py --baud 460800 --duration 10 \
  --raw Imu_test/artifacts/imu-01.bin 2>&1 | tee Imu_test/artifacts/imu-01.log
echo "test exit code: $?"
```

默认仅自动选择唯一的 `1a86:55d4` 设备；多个设备时用 `--port` 明确指定。

| 退出码 | 含义与下一步 |
| --- | --- |
| `0` | `PASS`；或 `--list` 正常执行（列举本身不验证数据） |
| `2` | 串口选择、参数或权限错误；先检查节点、ACL |
| `3` | 串口可打开但没有字节；检查供电、线缆、460800 和设备是否主动输出 |
| `4` | 收到字节但无有效 YESENSE 帧；检查协议和波特率，保留首 64 字节/原始数据 |
| `5` | 有有效帧但 IMU 数据帧不足 3 个；检查输出配置或增加 `--duration` |
| `6` | I/O 错误，例如拔出设备、端口忙、保存路径不存在或原始文件已存在 |
| `130` | 用户 `Ctrl+C` 停止 |

### 2.3 测试结束后撤销授权

设备未拔插、仍为原设备时，恢复先前备份：

```bash
sudo setfacl --restore=Imu_test/artifacts/tty-before.acl
getfacl -p "$H30_PORT"
```

如果已经拔插，不再对旧设备路径恢复 ACL；重新检查新节点即可。
只想移除本轮新加的个人 ACL、且确认授权前没有该条目时，也可使用：

```bash
sudo setfacl -x "u:$(id -un)" "$H30_PORT"
```

长期使用可选择 `sudo usermod -aG dialout "$(id -un)"`，然后完整注销并重新登录。
本机检查时用户原本不在该组；若以后需要撤销本轮新增成员关系，执行 `sudo gpasswd -d "$(id -un)" dialout` 并重新登录。
临时测试建议使用 ACL，避免修改账户配置。

## 3. 现有驱动确实无法读取时：测试官方 CH9102 驱动

仅在排除权限、占用、线缆、波特率及输出配置后进行。
Linux 厂商驱动使用沁恒的 **`ch343ser_linux` / `ch343.ko`**，不是 CH340 的 `ch341` 驱动。
下面先在本目录下载、编译、临时加载并只切换目标 USB 接口；不会全局卸载或屏蔽 `cdc_acm`。
官方参考：[仓库](https://github.com/WCHSoftGroup/ch343ser_linux)、[Makefile](https://github.com/WCHSoftGroup/ch343ser_linux/blob/main/driver/Makefile)、[设备绑定规则](https://github.com/WCHSoftGroup/ch343ser_linux/blob/main/udev/99-ch34x.rules)。

### 3.1 准备与构建（用户权限）

```bash
cd /home/ydh/Desktop/try_legged_control
mkdir -p Imu_test/.driver-work
uname -r
test -e "/lib/modules/$(uname -r)/build/Makefile"
command -v git
command -v make
command -v gcc
modinfo ch343
```

`modinfo ch343` 若已成功，先检查已有模块来源；不要覆盖已有安装。
缺少构建依赖时才安装，并记录本轮新装包；本机已检查到当前内核的 headers 目录存在：

```bash
sudo apt update
sudo apt install build-essential git "linux-headers-$(uname -r)"
```

继续下载和编译；`.driver-work/ch343ser_linux` 已存在时检查现有内容，不重复覆盖：

```bash
git clone --depth 1 https://github.com/WCHSoftGroup/ch343ser_linux.git Imu_test/.driver-work/ch343ser_linux
git -C Imu_test/.driver-work/ch343ser_linux rev-parse HEAD \
  > Imu_test/.driver-work/source-commit.txt
uname -r > Imu_test/.driver-work/kernel-version.txt
make -C Imu_test/.driver-work/ch343ser_linux/driver
modinfo Imu_test/.driver-work/ch343ser_linux/driver/ch343.ko
```

编译失败则停止，不执行后续加载。若内核更新，需为实际运行内核重新编译。
若加载时报 `Key was rejected by service`，检查 `mokutil --sb-state`；Secure Boot 环境需按发行版流程签名并注册模块，本流程不修改 Secure Boot 设置。

### 3.2 临时加载并切换这一台设备

先关闭 IMU 测试及其他串口程序。以下步骤使用同一个 Bash 终端，变量在后续回退中还要使用。
若端口已变化，更新 `H30_ORIGINAL_PORT`：

```bash
H30_ORIGINAL_PORT=/dev/ttyACM0
H30_INTERFACE=$(basename "$(readlink -f "/sys/class/tty/$(basename "$H30_ORIGINAL_PORT")/device")")
printf 'target interface: %s\n' "$H30_INTERFACE"
udevadm info --query=property --name="$H30_ORIGINAL_PORT"
```

核对 VID/PID 为 `1a86:55d4`、序列号为目标 IMU、当前驱动为 `cdc_acm`。
本次接口为 `1-3:1.0`，拔插后可能变化，不能直接照抄旧接口号。

```bash
sudo insmod Imu_test/.driver-work/ch343ser_linux/driver/ch343.ko
test -d /sys/bus/usb/drivers/usb_ch343
```

只有加载成功、上述目录存在才继续；模块已存在或加载失败时先调查原因。

```bash
sudo sh -c 'printf "%s" "$1" > /sys/bus/usb/drivers/cdc_acm/unbind' sh "$H30_INTERFACE"
sudo sh -c 'printf "%s" "$1" > /sys/bus/usb/drivers/usb_ch343/bind' sh "$H30_INTERFACE"
python3 -B Imu_test/imu_test.py --list
```

新节点通常为 `/dev/ttyCH343USB0`。如果绑定失败，直接执行 3.4 回退；若设备已拔插，重新检查接口号。
为新节点单独授权后测试：

```bash
sudo setfacl -m "u:$(id -un):rw" /dev/ttyCH343USB0
python3 -B Imu_test/imu_test.py --port /dev/ttyCH343USB0 --baud 460800 --duration 10
```

临时模块未写入系统模块目录，重启后不会因本流程自动加载。
它仍会保留在内存中直到卸载或重启，完成测试后按 3.4 清理。
若新驱动也失败，保留日志及原始数据，继续检查设备侧输出设置，不反复重装驱动。

### 3.3 可选：验证成功后安装到明确的系统目录

仅当厂商驱动确实解决问题、需要跨重启保留模块时执行。
记录安装内核和文件位置；这里不用上游 `make install`，便于精确回收自己安装的文件。
**确认 `modinfo ch343` 原本找不到已安装模块，且以下目标文件尚不存在。**

```bash
H30_KERNEL=$(cat Imu_test/.driver-work/kernel-version.txt)
test "$H30_KERNEL" = "$(uname -r)"
test ! -e "/lib/modules/$H30_KERNEL/updates/h30-imu/ch343.ko"
```

两条检查均成功后安装：

```bash
sudo install -D -m 0644 Imu_test/.driver-work/ch343ser_linux/driver/ch343.ko \
  "/lib/modules/$H30_KERNEL/updates/h30-imu/ch343.ko"
sudo depmod -a "$H30_KERNEL"
modinfo -n ch343
```

应指向 `updates/h30-imu/ch343.ko`。以后可用 `sudo modprobe ch343` 加载。
本流程不安装自动切换规则；重新插入或重启后若 `cdc_acm` 先绑定，仍需重新识别接口并执行 3.2 的单设备 unbind/bind（用 `modprobe` 替代 `insmod`）。
这不是 DKMS 安装；升级内核后需重新构建。无需厂商专用功能且原驱动已可读取时，保留系统 `cdc_acm` 即可。

### 3.4 回退至系统驱动

关闭使用目标串口的程序。在接口号仍有效且当前绑定 `usb_ch343` 时执行：

```bash
sudo sh -c 'printf "%s" "$1" > /sys/bus/usb/drivers/usb_ch343/unbind' sh "$H30_INTERFACE"
sudo modprobe cdc_acm
sudo sh -c 'printf "%s" "$1" > /sys/bus/usb/drivers/cdc_acm/bind' sh "$H30_INTERFACE"
sudo rmmod ch343
python3 -B Imu_test/imu_test.py --list
```

若 3.2 的厂商绑定失败、接口当前未绑定，跳过第一条 unbind，直接加载并绑定 `cdc_acm`。
若其他设备仍使用 `ch343`，不要强行卸载；先结束相关使用。
重新生成的 ACM 节点使用系统默认权限，原节点上的临时 ACL 不会迁移。

**仅执行过 3.3 时**，再删除本流程安装的那一个模块文件。使用记录的内核版本，即使当前已启动到其他内核也不误删别处文件：

```bash
H30_KERNEL=$(cat Imu_test/.driver-work/kernel-version.txt)
sudo rm -i -- "/lib/modules/$H30_KERNEL/updates/h30-imu/ch343.ko"
sudo rmdir -- "/lib/modules/$H30_KERNEL/updates/h30-imu"
sudo depmod -a "$H30_KERNEL"
```

删除前确认路径与 3.3 的安装记录一致；`rmdir` 仅在目录为空时成功。
本流程没有创建 `/etc/udev/rules.d/` 或 `/etc/modprobe.d/` 文件，因此没有对应系统规则需要清理。
不要卸载项目原本依赖的构建工具或 headers；仅当明确记录为本轮新增且不再需要时，另行评估卸载。

## 4. 本目录验证与清理

无需设备即可运行模拟协议与伪终端串口检查：

```bash
cd /home/ydh/Desktop/try_legged_control
python3 -B -m unittest discover -s Imu_test -p 'test_*.py' -v
```

自动化测试覆盖分包、连续帧、校验错误恢复、TLV 长度检查、数值解码，以及伪终端上的 460800 配置、接收、原始数据保存和串口参数恢复；CSV 检查还覆盖逐帧保存、缺失字段留空、中断保留及防止覆盖旧文件。
模拟测试通过不等于实机通过。本次运行实际读取脚本仍返回退出码 `2`（权限不足），实机结论以第 1 节为准。

文件布局：

```text
Imu_test/
  imu_test.py       # 串口读取入口
  test_imu_test.py  # 协议及伪终端测试
  README.md        # 使用、安装及回退
  .gitignore
  artifacts/       # 默认 CSV、可选原始数据、日志和 ACL 备份（忽略版本控制）
  .driver-work/    # 可选驱动源码、构建物、版本记录（忽略版本控制）
```

清理顺序：先停止读取 → 撤销临时 ACL → 如有厂商驱动则回退/卸载 → 最后处理本地文件。
保存好有用日志；确认不再需要 ACL 备份或内核版本记录后，可交互删除可选目录：

```bash
cd /home/ydh/Desktop/try_legged_control
rm -rI -- Imu_test/artifacts Imu_test/.driver-work
```

删除不存在的目录会报不存在，属于正常提示。若要移除整个测试工具，在完成系统回退后使用 `rm -rI -- Imu_test`。
