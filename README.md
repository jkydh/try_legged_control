# try_legged_control

本项目包含多个 Git Submodule，包括：

- `hpp-fcl`
- `legged_control`
- `ocs2`
- `ocs2_robotic_assets`
- `pinocchio`

因此请不要只使用普通的 `git clone`，否则子模块目录中的源码不会自动下载。

---

## 1. 首次克隆项目

推荐使用：

```bash
git clone --recurse-submodules git@github.com:jkydh/try_legged_control.git
```

然后进入项目目录：

```bash
cd try_legged_control
```

`--recurse-submodules` 会在克隆主仓库的同时，自动下载项目所依赖的所有 Git Submodule。

正常情况下，`src` 目录中应该包含：

```text
src/
├── colt/
├── hpp-fcl/
├── legged_control/
├── ocs2/
├── ocs2_robotic_assets/
└── pinocchio/
```

其中：

```text
hpp-fcl
legged_control
ocs2
ocs2_robotic_assets
pinocchio
```

都是 Git Submodule。

---

## 2. 已经使用普通 git clone 克隆过项目

如果之前使用的是：

```bash
git clone git@github.com:jkydh/try_legged_control.git
```

然后发现：

```text
src/hpp-fcl
src/legged_control
src/ocs2
src/ocs2_robotic_assets
src/pinocchio
```

这些目录为空，或者其中没有完整的源码，这是因为普通的 `git clone` 默认不会自动下载 Git Submodule。

首先进入项目目录：

```bash
cd try_legged_control
```

然后执行：

```bash
git submodule update --init --recursive
```

该命令会：

1. 初始化所有 Submodule；
2. 下载对应的远程仓库；
3. 切换到本项目记录的正确 Commit；
4. 递归初始化 Submodule 中可能存在的其他 Submodule。

下载完成后，可以查看：

```bash
ls src
```

并检查各个子模块：

```bash
ls src/hpp-fcl
ls src/legged_control
ls src/ocs2
ls src/ocs2_robotic_assets
ls src/pinocchio
```

如果能够看到各项目的源代码，则说明 Submodule 已经成功下载。

---


## 3. 本项目使用的 Submodule

### hpp-fcl

```text
https://github.com/leggedrobotics/hpp-fcl.git
```

项目目录：

```text
src/hpp-fcl
```

### legged_control

```text
https://github.com/qiayuanl/legged_control.git
```

项目目录：

```text
src/legged_control
```

### ocs2

```text
https://github.com/leggedrobotics/ocs2.git
```

项目目录：

```text
src/ocs2
```

### ocs2_robotic_assets

```text
https://github.com/leggedrobotics/ocs2_robotic_assets.git
```

项目目录：

```text
src/ocs2_robotic_assets
```

### pinocchio

```text
https://github.com/leggedrobotics/pinocchio.git
```

项目目录：

```text
src/pinocchio
```

---


