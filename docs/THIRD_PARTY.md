# 上游归属与第三方源码

本项目的 FAST-LIVO2 主体归属 hku-mars。保留上游文件头、署名、联系信息及根 GPLv2 `LICENSE` 原文。`package.xml` 使用 GPL-2.0 标识该根许可，不沿用与根许可不一致的 BSD 标签。第三方组件保留各自许可证；这不构成对其重新授权。

| 组件 | 来源 | 许可记录 | 随包范围 |
|---|---|---|---|
| FAST-LIVO2 | https://github.com/hku-mars/FAST-LIVO2 | 根 LICENSE / 源文件头 | 所需前端及改动 |
| M3DGR 适配与参数 | https://github.com/sjtuyinjie/M3DGR | 原适配署名及来源记录 | 已核验参数，不含数据 |
| vikit | https://github.com/xuankuzcr/rpg_vikit ，基于 UZH-RPG | 原 package.xml 声明 GPLv3；附 COPYING.GPL-3 | common / ros，不含 Python 包 |
| Sophus | https://github.com/strasdat/Sophus | 源文件 MIT 授权文本 | 必需库源码及轻量构建包装 |
| cv_bridge | https://github.com/ros-perception/vision_opencv | BSD 源文件文本；module.hpp 为 Apache-2.0；Boost 子集为 BSL-1.0 | 1.16.2 源包，附完整 Apache / Boost 文本 |
| image_transport_plugins | https://github.com/ros-perception/image_transport_plugins | package.xml 与源文件 BSD 文本 | compressed_image_transport |
| Livox ROS Driver2 | https://github.com/Livox-SDK/livox_ros_driver2 | 原 LICENSE.txt；消息部分 MIT | 两个 ROS1 消息及本地生成包装 |
| Livox ROS Driver 旧类型 | https://github.com/Livox-SDK/livox_ros_driver | 原 2019 LICENSE.txt；消息部分 MIT | 两个生成头，仅用于旧 wire type 编译兼容 |

每个依赖的具体版本及复制时源文件摘要见 `third_party/SOURCES.json`。同名 ROS 包 `livox_ros_driver2` 仅用于保持 bag 消息类型和 MD5 合约；**它不是完整硬件驱动**。旧 Livox 类型头、Sophus/vikit 命名、FAST-LIVO2 署名和第三方法律文本不做关键词替换。

依赖存在不同许可声明：尤其 FAST-LIVO2 根 GPLv2 文本与 vikit 的 GPLv3 package 声明分别保留。本项目不声称已获得上游重新许可，也不把这些组件统一标为 BSD/MIT。分发源码时必须保留相关文本；若扩展到公开二进制分发或商用，应由维护者核查适用许可及上游条件。
