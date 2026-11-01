# DASH-VLA 当前状态

更新：2026-11-01。机器：tamsgpu8，RTX 4090，设备 0，项目在 `/homeL/dong/Projects/dash-vla-libero`。

## 研究问题

不重训底层 VLA，在已有 Harness VLA（RPent，arXiv 2607.08448）上加入物理安全模块，让它在 SafeLIBERO 上不碰撞障碍物，同时保持任务完成率。

文章类型：方法文章，不做诊断类文章。方法是"规划器 + VLA + CBF + 带安全记录的记忆"。

## 报告索引

| 报告 | 内容 |
|---|---|
| `02_safelibero_motivation.md` | p2 正式轮：B0、B1、B2 的结果，G1–G4 判定 |
| `03_dash_offline.md` | 离线模块验证 |
| `04_b2_reproduction.md` | B2（SafeHarness）复现，冻结在 v7 |
| `05_memory_control.md` | 记忆对照：否定"记忆污染"，发现带障碍物标注的记忆有效 |
| `06_related_work_memory.md` | 记忆相关工作 |
| `07_cbf_round.md` | 第一轮：自研深度点云 CBF，暂停开发 |
| `08_aegis_cbf_harness.md` | 第二轮：AEGIS 原版 CBF + 规划器 + 反馈 + 记忆（H、H+M） |
| `09_step_budget.md` | 各论文的步数口径；A、AEGIS 在 900 步下重跑；官方 π0.5 对比 SFT 版 |
| `10_memory_ablation.md` | 记忆消融：H+E（只有探索笔记） |

每组的逐回合数据：`data/p2/`（B0、B1、B2），`data/cbf/`（第一、二轮和消融，以及 A、AEGIS 的 900 步结果）。

## 汇总结果

都是同样的 16 个评估回合：8 个场景，各取 task 0 的 episode 1、2。VLA 默认是 RLinf-Pi05-LIBERO-130-fullshot-SFT。

| 组 | 内容 | TSR | CAR | SSR | SSR@900 | 官方步数内 SSR | 报告 |
|---|---|---|---|---|---|---|---|
| A-SFT | 只有 VLA，900 步 | 13 | 3 | 3 | 3 | 3 | 09 |
| AEGIS-官方 | 官方 π0.5 + AEGIS CBF，900 步 | 15 | 12 | 12 | 12 | 12 | 09 |
| AEGIS-SFT | SFT 版 VLA + AEGIS CBF，900 步 | 15 | 15 | 14 | 14 | 12 | 09 |
| B0 | 原版 Harness | 16 | 11 | 11 | 10 | 2 | 02、08 |
| B1 | B0 + 安全提示 | — | — | — | 13 | 2 | 02、09 |
| B2 | SafeHarness 复现 | 16 | 16 | 16 | 12 | 2 | 04、08 |
| H | Harness + AEGIS CBF + 反馈 | 16 | 13 | 13 | 12 | 3 | 08 |
| H+E | H + 探索笔记 | 16 | 13 | 13 | 13 | 4 | 10 |
| H+M | H + 探索笔记 + CBF 记录 | 16 | 15 | 15 | 14 | 3 | 08 |

A、AEGIS 两组的上限是 900 步，所以它们的 TSR、CAR、SSR 也是 900 步内的结果。官方步数是 300 / 550 个控制步，开局 20 个等待步不计入。

- **不限步数**：B2 最好（16/16）。
- **900 步**：H+M 和 AEGIS-SFT 并列（14/16）。
- **官方步数**：AEGIS 最好（12/16）；规划器类的组太慢，只有 2 到 4/16。
- **记忆**：H+E 和 H 一样，H+M 高 2 个回合，差别在 long_I。提升来自 CBF 记录（报告 10）。
- **碰撞**：H、H+E、H+M 的所有碰撞都发生在 VLA 抓取（`pi0_pick`）中。
- 每组只跑了一次，VLA 输出有随机性，差一两个回合不能算差别。

## 下一步（待定）

1. H+M、H+E、AEGIS-SFT 各重复 2 到 3 遍，确认 14/16 对 14/16、15/16 对 13/16 是否稳定。
2. 针对抓取阶段的碰撞改进方法：保护手臂连杆，或者把记忆里的教训转成抓取前的接近约束。
3. 跨场景迁移：记忆来自同场景 episode 0，需要测换场景后是否仍有效。
4. 第二个基准（ResponsibleRobo benchmark）：待确认仿真器、机器人和数据。

## 已作废的工作

- LIBERO-Safety 阶段：环境自带的 robosuite 改过控制器，结论全部作废，代码和数据已删除（git 历史可查）。
- SafeLIBERO 预实验（10-28 至 10-30）：规划器能看到基准名称和带 `obstacle` 的物体名，数据已删除。
- p2 第一次尝试（10-30）：评估读不到任务记忆，存档在 `/homeL/dong/dash_archive/`（`04_b2_reproduction.md`）。
- 第二轮第一次启动（10-31）：GroundingDINO 找不到文本模型，存档在 `/homeL/dong/dash_archive/aegiscbf_failed_hfhome/`。
- 已更正：官方步数之前少算了 20 步，B0、B2 的官方步数结果没有排除碰撞（报告 08、09）。

## 相关工作的约束

- SafeHarness（arXiv 2609.20822）：同样的冻结 π0.5，在 SafeLIBERO 上用包围盒验证路线，900 步。它的基线数字照抄 AEGIS 的 300 / 550 步结果。
- AEGIS / VLSA（arXiv 2512.11891）：VLA + 末端椭球 CBF，300 / 550 步。

## 固定协议

- 权重：`RLinf-Pi05-LIBERO-130-fullshot-SFT`，RPent LIBERO 预设（每次推理执行 5 个动作）。AEGIS-官方组用 openpi `pi05_libero`。
- 规划器和障碍物命名：`gpt-6.1-sol`，reasoning `low`，所有组一致。
- 原始 Harness：工具表、提示、记忆机制和运行参数与上游 RPent 一致。B2 冻结在 v7（标签 `b2-final`）。
- 记忆：从官方 RPent 记忆（`47f301ef`，`runs/base_memory/`）开始。
- 规划器看不到任何基准名称或安全评测信息；碰撞判定只在环境进程里计算。
- 额度：TAMS 周用量约 80%（2026-11-01 01:30）。
