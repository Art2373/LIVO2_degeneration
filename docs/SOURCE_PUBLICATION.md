# 源码发布整理记录（2026-10-12）

本次是在 Windows 上对 Jetson 独立迁移源码的文档及发布属性整理。没有修改算法、门限、配置、构建脚本、求解器或测试源码，也没有追加构建、测试运行或数据回放。

## 版本与证据

- 运行版本仍为 `LIVO2_degeneration-1.0.0-migration-20261012`。
- 本次源码发布标识为 `LIVO2_degeneration-1.0.0-publication-20261012`。
- 原始迁移 ZIP 的 SHA256 为 `243a59296a35ae247bb9ec780902a01ee1a33dd323b9ac8c6bbadb00f4dfc9cf`，原包保留。
- `RELEASE.json` 中的既有 66 项源码哈希、构建产物哈希和四组回归记录均保持原值。新发布文档不构成新增实验结果。
- 历史论文实验使用其各自冻结版本；独立拆分版本另由四组迁移功能回归支持。不能把历史实验数字直接改标为新发布版本的实测结果。

本次只修正 `METHOD.md` 的时钟表述，补充此说明及 README 链接，并加入 Git 换行约束。维护者联系信息仍为明确占位，未推断或虚构用户身份。

## Windows 与 Linux 之间移交

原始源文件按字节保留，避免换行转换破坏既有源码哈希；Linux 入口及本次文档明确使用 LF。以下五个文件在 Git 索引和发布 ZIP 内均应为可执行模式 `100755`：

```text
tools/build.sh
tools/env.sh
tools/test.sh
scripts/fusion_parameter_provider.py
scripts/publish_camera_info.py
```

Git 提交前可检查 `git ls-files --stage`；从 Windows 重新建仓时需要按上述清单设置执行位。若复制方式丢失 Linux 权限，应在 Linux 项目目录恢复这些文件的执行权限再按 README 构建。不要把 Windows 解压目录或孤立的旧二进制当作可直接运行的 Linux 安装包。

仓库不包含 bag、实验大日志、构建产物、凭据或旧研究仓库历史。源包文件清单与 SHA256 放在发布包旁边；GitHub 是否上传成功以实际仓库和远端提交核验为准。
