# 依赖、独立构建与环境选择

## 已核验的环境

| 项目 | 版本 / 来源 | 处理 |
|---|---|---|
| 架构 / 系统 | aarch64，Ubuntu 20.04.6 | 系统依赖，不随源码包附带 |
| ROS | Noetic，roscpp 1.17.4 | `/opt/ros/noetic`，只共享系统 ROS |
| C++ / CMake / Python | GCC 9.4.0 / CMake 3.16.3 / Python 3.8.10 | 系统依赖 |
| OpenCV C++ 与 Python | 4.5.4 | 必须匹配；不混用系统另一份 Python 4.2 绑定 |
| PCL | 1.10.0 | 系统头文件与库 |
| Eigen / Boost / TurboJPEG | 系统开发包 | Eigen3；Boost thread/filesystem/system/python；libturbojpeg |
| Sophus | `a621ff2e56c56c839a6c40418d42c3c254424b5c` | 仓库内源码重新构建；MIT |
| vikit_common / vikit_ros | `6c886c8e5d83997806e00294826d528cea3581dd` | 仓库内源码重新构建；原相机模型及许可证保留 |
| cv_bridge | 本机源包 1.16.2，未附 Git 提交元数据 | 源文件逐个哈希登记；针对同一 OpenCV 重建 |
| compressed_image_transport | `cae592ce0816ea03bc38f0b6d27b6eae907feb10` | 标准插件源码重建，仅保留 RGB 需要的插件 |
| livox_ros_driver2 消息 | `13eb05e4e6dd7a765b934d0c5fd6236676a57b49` | 原 `.msg` 文件；仅消息子包，不是硬件驱动 |

`third_party/SOURCES.json` 包含依赖来源、版本、原始文件 SHA256。修改仅涉及 Sophus 和 driver2 消息子包的构建包装；不替换依赖算法。vikit 源码按当前相机参数接口固定，不能任意换成另一版同名库。

## 干净构建流程

1. 安装对应系统 ROS Noetic、PCL、Eigen3、Boost、OpenCV、Python3/numpy/scipy/yaml、libturbojpeg、catkin 和消息生成开发依赖。
2. 在 `src/LIVO2_degeneration` 中放入源码，不复制任何 `build/`、`devel/`、`install/` 或缓存。
3. 执行 `tools/build.sh`。脚本创建一个指向本发布依赖源码的 `src/livo2_third_party` 符号链接，独立编译 Sophus 到本工作空间的 `deps/sophus`，再执行 catkin。
4. 执行 `tools/env.sh python3 tools/check_environment.py` 检查 RGB 转换、图像操作、消息与核心库路径；再用 `tools/env.sh command ...` 使用新构建。无需修改 `.bashrc`、系统库或 ROS 默认环境。

在新机器上可用 `rosdep install --from-paths <workspace>/src --ignore-src -r -y` 解析 ROS 系统依赖，但应先确认 OpenCV 及 Python 绑定兼容性。本项目的脚本不自动执行 apt/pip，不安装另一套 OpenCV。Jetson 上绑定为 `/usr/lib/python3.8/dist-packages/cv2`；脚本只在新工作空间 `runtime_python/` 建立引用以优先选择它，不复制或更改系统模块。

可显式设置：

```bash
LIVO2_JOBS=2 \
LIVO2_OPENCV_DIR=/usr/lib/aarch64-linux-gnu/cmake/opencv4 \
LIVO2_OPENCV_PYTHON_DIR=/usr/lib/python3.8/dist-packages \
./tools/build.sh
```

`LIVO2_WS` 可指定当前项目所在工作空间；`LIVO2_DRIVER2=OFF` 只切换 CustomMsg 编译类型，不自动提供另一设备的相机、LiDAR 或时间配置。该开关不代表已验证 AVIA 上的管理兼容性。

核心输出为 `devel/lib/livo2_degeneration/fusion_mapping` 及 `libdegeneration_{laser_mapping,vio,lio,pre,imu_proc}.so`。构建不链接其他项目的旧核心对象或旧核心库。系统第三方库及本工作空间重建依赖仍是运行前提。

## 安装与可移交性

默认验证使用 catkin devel 空间。`catkin_make install` 提供节点、消息、Python 模块、配置、launch 和库；运行包装器当前以新工作空间 devel 为目标，不能将一个孤立安装目录当成带完整系统依赖的便携程序。移动源码时应重新构建，而不是拷贝旧 CMake 缓存。

本仓库不包含完整 Livox 硬件驱动。真正硬件部署需另外按设备官方驱动安装，与本消息子包避免同名冲突；当前回放不会启动驱动。

包的维护者邮箱使用明确的无效占位值 `noreply@example.invalid`，不冒充上游联系人；发布维护者可以填写自己的有效联系方式。

## 针对性测试

`tools/test.sh` 在本包的 CTest 目录运行六个 C++ 测试（测试断言保持开启），检查信息/提交、生命周期、图像保留、传感器时间、真实视觉调用反馈和墙钟调度，然后运行环境转换与编排/分析测试。cv_bridge 和压缩插件的可选测试集默认关闭，vikit 原构建自带的三个小测试目标保留；本包测试使用独立 `LIVO2_BUILD_TESTS=ON` 开关。
