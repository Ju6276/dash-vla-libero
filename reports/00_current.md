# DASH-VLA 当前状态

更新：2026-10-28。机器：tamsgpu8，RTX 4090，设备 0，项目在 `/homeL/dong/Projects/dash-vla-libero`。

## 研究问题

不重训底层 VLA，在已有 Harness VLA（RPent，arXiv 2607.08448）上，让物理安全信息改变技能调用、执行监测和失败恢复，并在新布局下保持任务完成能力。

Motivation 必须来自**不做修改的原始 Harness** 在**第三方公开 Benchmark** 上的表现。当前实验：SafeLIBERO，见 `02_safelibero_motivation.md`。

## 2026-10-28：此前的 LIBERO-Safety 结论全部作废

LIBERO-Safety 自带的 `third_party/robosuite-1.4` 改过控制器：OSC 的 `output_max` 是 2，官方 robosuite 1.4.1 是 `[0.05, 0.05, 0.05, 0.5, 0.5, 0.5]`。fullshot VLA 是按官方控制器训练的，在这个环境里每步动作被放大约 40 倍。

所以此前所有用 fullshot VLA 的 LIBERO-Safety 结果都不再作为证据，包括：8 条轨迹的统计、案例 A 和 B、"放书任务里规划器绕开 `pi0_pick`"、E1 调用条件扫描。原始数据移到了 `runs/_invalid_libero_safety_robosuite/`。

仍然有效、和控制器无关的一条：LIBERO-Safety 官方聚合的 `cost` 不能当安全指标（同名谓词互相覆盖；`checkrobotcontact` 恒为 0；成功步返回空字典）。

SafeLIBERO 现在改用官方 robosuite 1.4.1（`third_party/robosuite-stock/robosuite-1.4.1`，环境变量 `SAFELIBERO_ROBOSUITE_ROOT`）。

## 相关工作的约束

SafeHarness（arXiv 2609.20822）用同样的冻结 π0.5，在 SafeLIBERO 上用包围盒做路线验证，报告任务成功 81.2%、避碰 91.9%。DASH-VLA 不能只做"给 `move_to` 加障碍检查"。

## 固定协议

- 权重：`RLinf-Pi05-LIBERO-130-fullshot-SFT`，RPent LIBERO 预设（每次推理执行 5 个动作）。规划器：`gpt-6.1-sol`，reasoning `low`。
- 原始 Harness：Planner、SAM3、Memory、冻结 VLA、解析原语。不用 `full_task_vla`，不做安全微调。
- 记忆：从官方 RPent 记忆（`47f301ef`）开始，episode 0 探索并写入，独立 episode 评估。
- 安全判断只用环境进程里的计算（SafeLIBERO 官方位移规则加逐步接触审计），不进入规划器。

## 已停用的分支

历史诊断，不作为证据。数字和经过见 `08b_libero_safety_feasibility.md`。

| 分支 | 为什么停用 |
|---|---|
| G5-L0 到 L4.2 环境接入与审计 | 已完成，逐项审计已并入环境 |
| L5.0、L5.1 技能链筛选 | 安全微调权重加手写技能链，不是原论文 Harness |
| L5.2b、L5.3 规划器适配，`full_task_vla` 对比 | 偏离 Harness 主线 |
| OOD、微波炉、刀具 | 泛化是后续维度 |
| E1 调用条件扫描、H0 LIBERO-Safety 运行 | 控制器错误，作废 |
| Shadow、gates（`dash_vla/harness/`） | 旁路记录原型，暂停 |
