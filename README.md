# LIVO2_degeneration

## 面向 FAST-LIVO2 的视觉融合退化检测与恢复方法研究

这是一个 ROS Noetic 工程，使用**给定的融合参数**运行 LiDAR–IMU–RGB 前端。工作范围是视觉输入与真实 LIO 处理的调度、视觉更新反馈、退化撤回、有限探测及持续恢复确认。FAST-LIVO2 主体由 **hku-mars / Chunran Zheng 等作者**开发；本项目没有重新提出其滤波器、观测剔除或回滚算法。

唯一推荐配置是：

```yaml
dynamic_visual:
  management_enabled: true
  feedback_mode: measurement_commit_v1
  recovery_schedule: burst
```

撤回会清理视觉参考，重新探测时重建。生产入口不包含分散探测预算或保留历史参考的实验路径。相机内参、外参、给定时间补偿、模型身份及输入新鲜度检查仍然必要。

## 构建

已核验平台为 Jetson Orin NX、Ubuntu 20.04、ROS Noetic、OpenCV 4.5.4。源码包包含必须重建的小型第三方源码及消息定义，不附带 ROS、OpenCV、PCL 或二进制。

```bash
mkdir -p "$HOME/livo2_degeneration_ws/src"
# 将本项目目录放在该 src 下，或 git clone 到此处。
cd "$HOME/livo2_degeneration_ws/src/LIVO2_degeneration"
./tools/build.sh
./tools/env.sh rospack find livo2_degeneration
./tools/test.sh
```

`tools/build.sh` 从干净的系统 ROS 环境重新生成消息、构建 Sophus、vikit、cv_bridge、压缩图像插件和五个 `degeneration_*` 核心库。`tools/env.sh` 是命令包装器，**不要 source 它**；它不读取用户 shell 初始化文件，也不加载其他项目工作空间。详细依赖、版本和异机安装步骤见 [DEPENDENCIES.md](docs/DEPENDENCIES.md)。

## 回放与回归

数据路径必须显式提供。仓库不包含 bag，不启动硬件驱动。M3DGR 参数只用于其已核验 MID360 + RGB 配置，不可直接套到 AVIA 或其他相机。

```bash
P="$HOME/livo2_degeneration_ws/src/LIVO2_degeneration"
WS="$HOME/livo2_degeneration_ws"
OUT="$HOME/livo2_degeneration_regression_$(date +%Y%m%d_%H%M%S)"
python3 "$P/tools/regression/register.py" \
  --bag "$HOME/bags/Wheel-float01/Wheel-float01.bag" \
  --workspace "$WS" --output "$OUT" --env "$P/tools/env.sh" --port 11361
python3 "$P/tools/regression/batch.py" --plan "$OUT/RUN_PLAN.json" --execute
python3 "$P/tools/regression/analyze.py" --root "$OUT" --out "$OUT/analysis"
```

注册入口只接受已核验的 Wheel-float01 哈希；实际文件名可通过 `--bag` 指定。四组依次为健康、RGB 断流、黑图、参数服务停止重启；半速、从起点至自然 EOF，+15 传感器秒允许接入，+43–51 秒固定故障。执行错误停止后续组，不自动补跑。故障并非原始公开数据的一部分。请查看 [RUNNING.md](docs/RUNNING.md) 和每次生成的 `RUN_PLAN.json`，不要将启动 PID 当作完成证据。

## 方法与证据边界

- 将调用的执行事实、最终保留的纯视觉位姿测量信息、有限提交分开。零信息调用不刷新成功；合法的零位姿增量可以提交。
- 非零测量信息不保证对应正确，也不代表定位准确。包含先验的求解矩阵不被当成视觉信息矩阵。
- PROBING 使用真实滤波状态，并非无影响的隔离试验；撤回不清除此前视觉对轨迹的影响。
- 已有 Wheel 开发验证显示可处理指定输入、服务及黑图故障，但伴随自然撤回、恢复等待和切换代价。Retail 的管理开启迁移未建立持续信息提交，是已观察到的兼容性限制。
- 当前版本的真实播放器暂停、1 倍速、严格定位精度及普遍跨配置能力没有完成验收。半速回放不能证明实机实时性。

方法说明见 [METHOD.md](docs/METHOD.md)，适用范围见 [VALIDATION_AND_LIMITS.md](docs/VALIDATION_AND_LIMITS.md)。本次独立构建和四组迁移回归已完成，实际结果见 [MIGRATION_VALIDATION.md](docs/MIGRATION_VALIDATION.md)，精确产物及源码哈希见 [RELEASE.json](RELEASE.json)。

## 目录

- `src/`、`include/`：调度、融合模型应用、视觉反馈和 FAST-LIVO2 前端；
- `msg/FusionModel.msg`、`python/`、`scripts/`：给定融合参数、CameraInfo 及服务会话；
- `launch/`、`config/`：当前可复现入口和固定参数；
- `tools/regression/`：有界故障注入、输入身份记录和离线分析；
- `tests/`：反馈、状态转换、队列和边界针对性检查；
- `third_party/`：带出处的源代码依赖，详见其 `SOURCES.json`。

版权和许可证见 [LICENSE](LICENSE)、[NOTICE](NOTICE)、[THIRD_PARTY.md](docs/THIRD_PARTY.md)。依赖保留原名称和真实作者，不代表由本项目原创。

## 源码发布记录

Windows 发布整理与 Linux 执行权限说明见 [SOURCE_PUBLICATION.md](docs/SOURCE_PUBLICATION.md)。运行证据仍对应原独立迁移版本，本次仅整理文档与发布属性，没有新增算法实验。
