# try_legged_control

本项目包含多个 Git Submodule，包括：

- `hpp-fcl`
- `legged_control`
- `ocs2`
- `ocs2_robotic_assets`
- `pinocchio`

因此请不要只使用普通的 `git clone`，否则子模块目录中的源码不会自动下载。

## 1. 首次克隆项目

推荐使用：

```bash
git clone --recurse-submodules git@github.com:jkydh/try_legged_control.git
