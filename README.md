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

## 3. 更新主项目

进入项目：

```bash
cd try_legged_control
```

更新主仓库：

```bash
git pull
```

然后同步 Submodule 配置：

```bash
git submodule sync --recursive
```

再更新所有 Submodule：

```bash
git submodule update --init --recursive
```

完整命令为：

```bash
cd try_legged_control

git pull

git submodule sync --recursive

git submodule update --init --recursive
```

---

## 4. 查看 Submodule 状态

可以使用：

```bash
git submodule status
```

查看当前所有 Submodule 对应的 Commit。

正常情况下会看到类似：

```text
8be41cba990eaef717bd4d2b4a7bb05c8ae66ccd src/hpp-fcl
a7f381c0367e98e31c01336e678eef47e304d40d src/legged_control
26386754b8bf31ab78b503971d0cbf4fdbcd7cb4 src/ocs2
xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx src/ocs2_robotic_assets
b86197e04265892c32966736000a01ee4862a060 src/pinocchio
```

---

## 5. 推荐的完整克隆方式

第一次下载本项目时，建议直接执行：

```bash
git clone --recurse-submodules git@github.com:jkydh/try_legged_control.git
cd try_legged_control
```

如果发现某些 Submodule 没有正常下载，再执行：

```bash
git submodule update --init --recursive
```

---

## 6. 已经 clone，但 Submodule 目录为空

如果主项目已经下载完成，但是：

```text
src/hpp-fcl
src/legged_control
src/ocs2
src/ocs2_robotic_assets
src/pinocchio
```

目录中没有代码，不需要重新下载整个项目。

只需要进入项目：

```bash
cd try_legged_control
```

执行：

```bash
git submodule update --init --recursive
```

即可。

---

## 7. Submodule 下载失败时

先同步 Submodule 配置：

```bash
git submodule sync --recursive
```

然后重新初始化：

```bash
git submodule update --init --recursive
```

如果希望查看更详细的信息，可以执行：

```bash
git submodule status
```

---

## 8. 本项目使用的 Submodule

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

## 9. 常用命令汇总

### 第一次克隆

```bash
git clone --recurse-submodules git@github.com:jkydh/try_legged_control.git
```

### 进入项目

```bash
cd try_legged_control
```

### 初始化 Submodule

```bash
git submodule update --init --recursive
```

### 同步 Submodule 配置

```bash
git submodule sync --recursive
```

### 查看 Submodule 状态

```bash
git submodule status
```

### 更新主仓库

```bash
git pull
```

### 更新后重新同步 Submodule

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

---

## 10. 注意事项

本项目中的以下目录并不是普通文件夹：

```text
src/hpp-fcl
src/legged_control
src/ocs2
src/ocs2_robotic_assets
src/pinocchio
```

它们是 Git Submodule。

因此，在 GitHub 网页中这些目录的图标旁边会显示一个箭头，这是正常现象。

普通命令：

```bash
git clone git@github.com:jkydh/try_legged_control.git
```

只会克隆主仓库，不会自动下载 Submodule。

推荐始终使用：

```bash
git clone --recurse-submodules git@github.com:jkydh/try_legged_control.git
```

如果已经使用普通方式克隆，则执行：

```bash
git submodule update --init --recursive
```

即可补充下载所有 Submodule。
