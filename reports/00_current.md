# DASH-VLA 当前状态

更新：2026-10-30。机器：tamsgpu8，RTX 4090，设备 0，项目在 `/homeL/dong/Projects/dash-vla-libero`。

## 研究问题

不重训底层 VLA，在已有 Harness VLA（RPent，arXiv 2607.08448）上，让物理安全信息改变技能调用、执行监测和失败恢复，并在新布局下保持任务完成能力。

Motivation 必须来自**不做修改的原始 Harness** 在**第三方公开 Benchmark** 上的表现。当前实验：SafeLIBERO，见 `02_safelibero_motivation.md`。

## 已作废、已移出仓库的工作

- LIBERO-Safety 阶段（G5、L4–L5、OOD、E1、H0 等）：环境自带的 robosuite 改过控制器，结论全部作废。相关脚本、记录和 `dash_vla/` 原型已从仓库删除（git 历史可查），服务器上的数据移到项目目录外的 `/homeL/dong/Projects/dash-vla-libero_archive_void/`。
- SafeLIBERO 预实验（2026-10-28 至 10-30 的 B0、B1、B2 试点）：规划器能看到 SafeLIBERO 名称、`_auditor` 提示和带 `obstacle` 的物体名，B2 还有三处实现错误。数字不作为证据，数据同样移出。

## 相关工作的约束

SafeHarness（arXiv 2609.20822）用同样的冻结 π0.5，在 SafeLIBERO 上用包围盒做路线验证，报告任务成功 81.2%、避碰 91.9%。DASH-VLA 不能只做"给 `move_to` 加障碍检查"。B0、B1、B2 是对照组，不含 DASH-VLA 的任何机制；DASH-VLA 等正式轮结果出来后再设计。

## 固定协议

- 权重：`RLinf-Pi05-LIBERO-130-fullshot-SFT`，RPent LIBERO 预设（每次推理执行 5 个动作）。规划器：`gpt-6.1-sol`，reasoning `low`。
- 原始 Harness：工具表、提示、记忆机制和运行参数与上游 RPent 一致；`full_task_vla` 不在工具表中，只供 A 组调用。
- 记忆：从官方 RPent 记忆（`47f301ef`，`runs/base_memory/`）开始。
- 规划器看不到任何基准名称或安全评测信息；碰撞判定只在环境进程里计算，记录写到规划器输出目录之外。
