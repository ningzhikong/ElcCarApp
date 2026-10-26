# RPM 台架自动调参

入口：`tools/autotune/rpm_tune.py`。复用现有串口协议和 `tune.optimize` 的基线、候选、基线复测、独立确认及回退流程。无需额外 AI 调用，使用本地小范围候选规则。

## 已验证的接入方式

主控必须已运行调参模式。串口三线连接，下载器供电脚不接小车，车轮架空；绿板保持供电，停止由固件取消 PWM。运行前会检查 build ID、CPR 和 RPM 反馈算法身份，全部吻合后才 ARM。

```powershell
python tools/autotune/rpm_tune.py --port COM10 --build-id 0ef01c6b12fa4807 --rounds 1 --repeats 3 --first-wheel left
python tools/autotune/rpm_tune.py --port COM10 --build-id 0ef01c6b12fa4807 --rounds 1 --repeats 3 --first-wheel right
```

两条命令是两个独立搜索，第二条不会自动继承第一条的结果。接受新参数后继续搜索，须显式传入报告中的四个最佳增益。也可 `--rounds 2` 在同一次搜索内依次验证左右轮，第二轮从第一轮最佳参数继续。每轮只将一侧 Ki 降低20%，Kp及另一侧不变；不是完整参数空间搜索。

每组各3次10 RPM和20 RPM恒速试验，每次5.5秒，PWM默认150/999且不允许提高到该上限以上。每次都保存原始CSV、请求参数、校验身份、得分和停机结果。一次搜索最多2轮，每组重复数限制3～5；发生通信、固件或轨迹完整性异常即停止，不自动清故障或重启。

评分包含起转阶段目标跟踪、稳态误差、波动、左右轮差异和停机残余，并按较差转速及较差轮评估。匹配或平稳性判据不通过会加罚。仅当候选和独立确认均比前后基线中位数至少改善3%，且最坏表现及任一侧最坏表现不退步时才接受。默认允许基线评分漂移25%；超过阈值停止并报告漂移，不能当作改善。

## 结果与限制

数据位于 `.build/hardware_checks/rpm_search_*/`，`summary.json` 给出最终参数和决策，失败见 `failure.json`，报告生成前的进度见 `checkpoint.json`。保持停止状态结束，不将参数自动写入普通行驶配置。

报告使用 `real_motor_rpm` 和 `ELC_RPM_PI_ADAPTIVE_V1` 身份，旧计数/IIR参数导出器会拒绝此类报告，防止误用于普通行驶控制器。CPR1080/1320仍是手工标定参考，结论只适用于架空短时测试；不是独立转速计或地面负载验证。
