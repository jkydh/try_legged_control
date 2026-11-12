# Vendored source repositories

These upstream checkouts are stored as ordinary files in this repository, including local modifications.
Cloning the main repository supplies their source and the initialized nested dependencies; no submodule initialization is required.
Upstream LICENSE/COPYING and copyright notices remain in their original locations.

| Directory | Upstream | Original checkout commit |
| --- | --- | --- |
| `src/hpp-fcl` | https://github.com/leggedrobotics/hpp-fcl.git | `8be41cba990eaef717bd4d2b4a7bb05c8ae66ccd` |
| `src/hpp-fcl/cmake` | https://github.com/jrl-umi3218/jrl-cmakemodules.git | `95917116c0f1b0b0690a22e12d3871b3bb7cae27` |
| `src/hpp-fcl/third-parties/qhull` | https://github.com/qhull/qhull.git | `5a79a0009454c86e9848646b3c296009125231bf` |
| `src/legged_control` | https://github.com/qiayuanl/legged_control.git | `a7f381c0367e98e31c01336e678eef47e304d40d` |
| `src/ocs2` | https://github.com/leggedrobotics/ocs2.git | `26386754b8bf31ab78b503971d0cbf4fdbcd7cb4` |
| `src/ocs2_robotic_assets` | https://github.com/leggedrobotics/ocs2_robotic_assets.git | `b126d00d55f1e67905c3c1516995466df6873c5c` |
| `src/pinocchio` | https://github.com/leggedrobotics/pinocchio.git | `b86197e04265892c32966736000a01ee4862a060` |
| `src/pinocchio/.travis` | https://github.com/jrl-umi3218/jrl-travis | `0f12f18e62a504278ef31f0118f1da48607a524e` |
| `src/pinocchio/cmake` | https://github.com/jrl-umi3218/jrl-cmakemodules | `95917116c0f1b0b0690a22e12d3871b3bb7cae27` |
| `src/pinocchio/models/others` | https://github.com/Gepetto/example-robot-data.git | `f096f11b8d13d045178720040919ebb71b3bf86d` |
| `src/pinocchio/models/others/cmake` | https://github.com/jrl-umi3218/jrl-cmakemodules.git | `e6e09c0b944530c92c17c4fb64cd2a2f63ed46c0` |

## Local conversion backup

The original Git metadata and `.gitmodules` files are moved, not deleted.
Local backup: `.vendor-git-backup/20261112_114648/` (ignored by the main repository).
`conversion-plan.json` records the source hashes, original repository state, and moved paths.
`main-index.before` is a backup of the main repository index before conversion.
Restore metadata to its recorded original path if independent repository histories are needed again.
Do not overwrite a newer main index with the old index backup without preserving any later staging changes.

The conversion stages source files for review; it does not create a commit or push.
Generated build/devel/log directories, local backups, and `Imu_test/artifacts/` are excluded.
