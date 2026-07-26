# 仓库第三方资产审计

## 审计范围与结论

本审计基于提交 `3c25a58`，覆盖 `third_party/`、预编译库、模型/媒体、ROS 构建输出和本地环境。本轮只调整 Git 索引与忽略规则，没有物理删除本地文件，也没有删除用途不明的 SDK。

原先“5101 个路径包含 `build/install/log`”是宽泛路径段统计，其中包含 9 个真实源码头文件：

```text
third_party/unitree_sdk2/include/unitree/common/log/*.hpp
```

这些头文件必须保留。重新分类后的索引清理为：

| 类别 | 文件数 | 处理 |
| --- | ---: | --- |
| 第三方生成的 `build/install/log` | 5092 | 已用精确路径从 Git 索引移除；本地文件保留 |
| 根 `.venv` | 5 | 已从 Git 索引移除；本地环境保留 |
| 缓存/字节码命中 | 54 | 其中 49 项位于上述 install 树，5 项为根 `.venv` |
| 唯一索引移除总数 | 5097 | 5092 个生成物 + 5 个 `.venv` 条目 |
| SDK `common/log` 源码头文件 | 9 | 继续追踪 |

基线提交中，仓库根 `build/`、`install/`、`log/` 的追踪数均为 0；生成物都来自下表中的第三方构建树。

## 已从索引移除的生成树

容量为基线 Git blob 逻辑大小。

| 路径 | 文件数 | 大小 | 类型 | 引用/重建关系 |
| --- | ---: | ---: | --- | --- |
| `third_party/unitree_sdk2/build/` | 446 | 115.395 MiB | CMake/x86_64 编译输出，含示例 executable | 由 SDK2 源码和保留的架构库重建 |
| `third_party/unitree_sdk2/install/` | 720 | 40.466 MiB | 本地 install 副本 | 与源码、头文件和保留库重复，应按环境重建 |
| `third_party/unitree_ros2/cyclonedds_ws/build/` | 2776 | 22.529 MiB | ROSIDL/CMake 构建输出 | 由 `cyclonedds_ws/src/` 重建 |
| `third_party/unitree_ros2/cyclonedds_ws/install/` | 1096 | 3.371 MiB | x86_64/Python 3.10 install 树 | 含 924 个符号链接，其中 915 个指向个人工作区，不能跨机器复用 |
| `third_party/unitree_ros2/cyclonedds_ws/log/` | 54 | 1.399 MiB | colcon 日志 | 无运行时保留价值 |
| **合计** | **5092** | **183.159 MiB** | 生成物 | 已从索引移除，本地未删除 |

## 必须保留的第三方依赖

| 路径 | 大小/类型 | 仓库引用 | 是否必须保留 | 环境与后续建议 |
| --- | --- | --- | --- | --- |
| `third_party/unitree_sdk2/{CMakeLists.txt,cmake,include,thirdparty/include,example}` | 约 6.194 MiB，SDK 源码、头文件和 CMake | `src/rk_go2_sdk_bridge/CMakeLists.txt` 和 D1 CMake | 是，属于源码依赖 | VM 和机器人均可能构建；未来增加上游 tag/commit、许可证和校验和记录 |
| `third_party/unitree_sdk2/lib/{aarch64,x86_64}/libunitree_sdk2.a` | 两个预编译静态库，共约 53.291 MiB | CMake 按 `CMAKE_SYSTEM_PROCESSOR` 选择 | 是 | aarch64 用于机器人，x86_64 用于 VM；未来评估 Git LFS 或带 SHA-256 的安装脚本 |
| `third_party/unitree_sdk2/thirdparty/lib/{aarch64,x86_64}/libddsc*.so*` | CycloneDDS 预编译动态库，共约 20.372 MiB | SDK target 的 RPATH、D1 使用说明 | 是 | 按 CPU 架构使用；不能按普通生成 `.so` 删除 |
| `third_party/unitree_ros2/cyclonedds_ws/src/` | 50 项，约 0.012 MiB，ROS message 源码 | `go2_motion_client.py` 使用 `unitree_api.msg.Request` | 是 | 应分别在 Humble/Foxy 环境的 ignored 目录中构建 |
| `third_party/unitree_ros2` 其余源码、示例、文档 | 保留后整树 160 项，约 3.063 MiB | vendor 示例和说明 | 本轮保留 | 不是当前主链全部必需；未来可评估固定上游版本安装脚本，不建议 LFS |
| `third_party/unitree_d1_sdk/` | 33 项，约 0.133 MiB，D1 DDS/C++ 源码 | `d1_pick_node.py`、D1 CMake、`test_grasp.py` | 是 | 真实执行仅限 D1 硬件；目录内未发现明确许可证文件，公开分发前需确认授权 |
| `third_party/unitree_sdk2_official/` | 当前为空，Git 不保存空目录 | Go2 CMake、动作脚本优先尝试该 SDK root | 当前无可保留文件 | 属于悬空候选路径；未来统一为一个可配置 SDK 根 |
| RealSense | 仓库内没有 SDK 或模型 | `rk_bringup/package.xml` 和 6 个 launch 引用 `realsense2_camera_node` | 不应 vendor | 通过 rosdep/apt 为 Humble、Foxy 分别安装；preflight 作为外部 executable WARN |

### 保留的预编译库

六个真实二进制载荷合计约 73.663 MiB：

| 架构 | 文件 | 大小 |
| --- | --- | ---: |
| aarch64 | `libunitree_sdk2.a` | 27.300 MiB |
| x86_64 | `libunitree_sdk2.a` | 25.991 MiB |
| aarch64 | `libddsc.so` | 7.269 MiB |
| x86_64 | `libddsc.so` | 7.146 MiB |
| aarch64 | `libddscxx.so` | 2.992 MiB |
| x86_64 | `libddscxx.so` | 2.965 MiB |

其中四个实体文件超过 5 MiB；另有动态库版本符号链接。它们被真实 CMake/SDK 链路引用，因此本轮继续保留并由 preflight 输出 WARN，不判定为生成物。

当前没有 `.gitattributes` 或 Git LFS 配置。未来只应对这六个必要二进制评估 LFS；生成的 `build/install/log` 不应进入 LFS。若机器人部署环境不能可靠执行 `git lfs pull`，优先使用版本固定、带 SHA-256 校验的安装脚本。

## 其他媒体与大文件

- `docs/report_figures/`：12 项、约 0.924 MiB；当前 Markdown 未引用，可能属于报告材料，确认归属前保留。
- vendor PNG：11 个、约 2.117 MiB，属于上游 README/示例资产。
- vendor WAV：2 个、约 0.252 MiB，供 G1 音频示例使用。
- `motion.seq`：2 个 ASCII 轨迹文件、约 1.956 MiB，供 vendor 示例使用。
- 未发现压缩包、ONNX/PT/TFLite 模型、视频、bag、DB3 或 MCAP。
- 基线 55 个 `.log` 文件均位于已清理的 vendor 生成树；清理后索引中为 0。
- 本地 `.vscode/browse.vc.db` 约 953 MB，已忽略且从未受 Git 控制。
- 本地根 `build/install/log` 继续保留在磁盘，由 `.gitignore` 隔离。

## 审计命令

```bash
git ls-tree -r -l 3c25a58 third_party
git diff --cached --name-only --diff-filter=D | wc -l

git ls-files \
  'third_party/unitree_sdk2/lib/**' \
  'third_party/unitree_sdk2/thirdparty/lib/**'

find . -path './.git' -prune -o -type f -size +5M \
  -printf '%s\t%p\n' | sort -nr

rg -n \
  'unitree_sdk2|unitree_ros2|unitree_d1_sdk|realsense2_camera' \
  src scripts third_party \
  --glob 'CMakeLists.txt' --glob '*.py' --glob '*.sh' \
  --glob '*.launch.py' --glob 'package.xml'
```
