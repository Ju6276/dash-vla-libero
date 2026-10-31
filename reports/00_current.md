# DASH-VLA 当前状态

更新：2026-10-30。机器：tamsgpu8，RTX 4090，设备 0，项目在 `/homeL/dong/Projects/dash-vla-libero`。

## 研究问题

不重训底层 VLA，在已有 Harness VLA（RPent，arXiv 2607.08448）上，让物理安全信息改变技能调用、执行监测和失败恢复，并在新布局下保持任务完成能力。

Motivation 必须来自**不做修改的原始 Harness** 在**第三方公开 Benchmark** 上的表现。当前实验：SafeLIBERO，见 `02_safelibero_motivation.md`。

## 已作废、已移出仓库的工作

- LIBERO-Safety 阶段（G5、L4–L5、OOD、E1、H0 等）：环境自带的 robosuite 改过控制器，结论全部作废。相关脚本、记录和 `dash_vla/` 原型已从仓库删除（git 历史可查），服务器上的数据已删除。
- SafeLIBERO 预实验（2026-10-28 至 10-30 的 B0、B1、B2 试点）：规划器能看到 SafeLIBERO 名称、`_auditor` 提示和带 `obstacle` 的物体名。数字不作为证据，数据已删除。
- p2 第一次尝试（10-30 凌晨至下午，10 个回合）：评估阶段读不到任务记忆，代码版本不一致，B2 是早期版本。数据移到项目外的 `/homeL/dong/dash_archive/`，原因见 `04_b2_reproduction.md`。

## 当前进度（2026-10-30 晚，暂停）

- 正式轮 p2 已完成，64 个回合全部跑完。结果和 G1–G4 判定见 `02_safelibero_motivation.md`，每个回合的数据在 `reports/data/p2/`。
  - G1、G2 满足。
  - G3 按 900 步截断的主指标不满足：一句通用提示就把评估回合的碰撞从 5 降到 2，任务完成率没有下降。
  - G4 偏弱。
  - 按事先定的规则，"原始 Harness 需要新的安全算法"这个动机在 p2 上不成立。
- 记忆对照实验（`05_memory_control.md`，2026-10-31）：
  - 去掉场景自己的探索记忆，碰撞没有减少（16/20 对 13/20），"记忆污染"被否定。p2 里 7/18 对 0/14 的差距来自组别和场景的混杂。
  - 换上带障碍物标注的 B1 笔记后，同样两个场景碰撞为 0/10（own 是 4/10，p = 0.043），完成 10/10。
  - 相关工作见 `06_related_work_memory.md`。
- B2 冻结在 v7（标签 `b2-final`）。正式回合有 2 个因显存不足重跑，存档在 `/homeL/dong/dash_archive/p2_oom/`。
- TAMS 额度用到约 60%。
- 离线验证：模块 1、3 完成（`03_dash_offline.md`）；模块 2 暂缓。

**暂停时未完成的事项**
- 方向待定。可能的方向：自动生成带安全标注的记忆（用碰撞或位移信号审计并改写探索记忆），先要在更多场景上复现 b1 的效果。
- 步数记录和仿真器有少量差异：B2 spatial_I e2 是 495 对 499，测试回合 4 是 1531 对 1532。原因待查，对碰撞归因的影响很小。

## 相关工作的约束

SafeHarness（arXiv 2609.20822）用同样的冻结 π0.5，在 SafeLIBERO 上用包围盒做路线验证，报告任务成功 81.2%、避碰 91.9%。DASH-VLA 不能只做"给 `move_to` 加障碍检查"。B0、B1、B2 是对照组，不含 DASH-VLA 的任何机制；DASH-VLA 等正式轮结果出来后再设计。

## 固定协议

- 权重：`RLinf-Pi05-LIBERO-130-fullshot-SFT`，RPent LIBERO 预设（每次推理执行 5 个动作）。规划器：`gpt-6.1-sol`，reasoning `low`。
- 原始 Harness：工具表、提示、记忆机制和运行参数与上游 RPent 一致；`full_task_vla` 不在工具表中，只供 A 组调用。
- 记忆：从官方 RPent 记忆（`47f301ef`，`runs/base_memory/`）开始。
- 规划器看不到任何基准名称或安全评测信息；碰撞判定只在环境进程里计算，记录写到规划器输出目录之外。
