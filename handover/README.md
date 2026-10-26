# 交接归档

项目源码、tests和docs直接放在仓库中；下列压缩包保存实测证据和可复核的构建产物。下载本分支完整ZIP或git clone后，在项目根目录执行：

```powershell
python handover/restore.py
python handover/restore.py --extract
```

第一条只校验分卷、压缩包及每个文件的SHA256；第二条将归档恢复到原来的.build/路径。已有内容相同则跳过，不同则拒绝覆盖；确需替换时使用 --extract --overwrite。恢复与校验均不连接硬件、不执行归档脚本。完整清单见 manifest.json。

- hardware-evidence.zip.part001/002：原始实测日志、回读、USB检查、分析图、历史辅助脚本及模块参考资料。
- validated-baselines.zip：90 RPM源码/产物基线及100 RPM候选镜像。
- firmware-and-validation.zip：五种构建的HEX、AXF、map、日志与回归结果。对象文件、测试可执行缓存不包含。
- local-support.zip：历史本地辅助脚本与阶段发布元数据。部分脚本含旧固件硬编码或会操作硬件，仅作交接记录，不能批量执行。

归档恢复后，根目录.build仍被Git忽略。工具链、pyOCD依赖缓存、重复worktree、.git、助手记忆及工作区外微信原始录像不打包。分析数据中已有的视频提取帧和图像会保留。Keil/GCC需单独安装，Python硬件依赖见tools/requirements-hardware.txt。

manifest.json记录的是归档生成时的本地文件。源代码和文档以本次Git提交为准，旧基线中的源码和历史构建不代表当前运行版本。接手流程和验证边界见 [工作交接](../docs/工作交接-2026-10-26.md)。
