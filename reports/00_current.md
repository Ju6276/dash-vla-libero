# DASH-VLA 当前状态

更新：2026-11-01 晚。机器：tamsgpu8，RTX 4090，设备 0，项目在 `/homeL/dong/Projects/dash-vla-libero`。

## 研究问题

不重训 VLA，让 Harness VLA（RPent，arXiv 2607.08448）在有障碍物的场景里安全地完成任务。文章类型是方法文章。

**当前的论证思路**（待基线确认）：

- VLA + CBF（AEGIS）只能完成 VLA 学过的任务；
- 规划器（Harness）能泛化到新指令，但不保证安全；
- 我们的方法（H+M）要让两者同时成立。

## 已经确定的事

1. **原 SafeLIBERO task 0 区分不了方法**（报告 08–10、12）。VLA 学过这些任务，AEGIS-SFT 两次运行都是 14/16。规划器类的方法慢，波动有 3 个回合；900 步内没有超过 AEGIS。不限步数时，SafeHarness 是 16/16。
2. **新指令上 AEGIS 做不成任务**（报告 14）。6 条不在 LIBERO-130 里的指令，48 个回合，成功 7/48，撞 20/48。
3. **碰撞几乎都来自手臂，不是夹爪**（报告 13）。r2 和开发轮里有逐步记录的 17 次碰撞，16 次是 link5 / link6。只保护夹爪的 CBF 挡不住这类碰撞。

## 下一步（只做这一件）

在新指令的同样 48 个回合上（6 条指令 × 等级 I、II × episode 0–3），跑带规划器的方法。代码和方法都不改。

| 组 | 内容 | 优先级 |
|---|---|---|
| H+M（我们的方法） | Harness + AEGIS CBF + 反馈 + 带 CBF 记录的安全记忆，同报告 08 | 先跑 |
| B2 | SafeHarness，冻结在 v7 | 先跑 |
| B0 | RPent 原版 Harness（不是我们的方法），作参考 | 后跑 |

- 有探索的组在 episode 4 上探索，在 episode 0–3 上评估。AEGIS 不探索。
- 要看的：
  - 任务成功率是否明显高于 AEGIS 的 7/48；
  - 各撞了多少次，撞在哪里。

**额度**：每组大约 60 个规划器回合（48 个评估 + 12 个探索），约 12%。现在 TAMS 周用量 95%，下次重置是 2026-11-04 09:04；也可以用重置卡。

等这一步的结果出来，再决定方法要改哪一部分，一次只改一个。

## 已停止或冻结

| 内容 | 状态 | 报告 |
|---|---|---|
| 交给 VLA 前的安全检查、相对障碍物的记忆 | 没有效果，已停止 | 12 |
| 连杆（link5 / link6）CBF 约束 | 冻结，代码保留，默认关闭（`SL_CBF_LINKS`）。实测的"5 次碰撞减到 1 次"用的是有 bug 的第一版；修正版只做过离线测试 | 13 |
| 重复运行 r3 | 中途停止；AEGIS-SFT 的 16 个回合已完成，H、H+M 只跑了一部分 | 12 |
| 自研深度点云 CBF | 暂停 | 07 |

## 报告索引

| 报告 | 内容 |
|---|---|
| `02_safelibero_motivation.md` | B0、B1、B2 正式轮 |
| `03_dash_offline.md` | 离线模块验证 |
| `04_b2_reproduction.md` | SafeHarness（B2）复现，冻结在 v7 |
| `05_memory_control.md`、`06_related_work_memory.md` | 记忆对照、相关工作 |
| `07_cbf_round.md` | 自研深度点云 CBF（暂停） |
| `08_aegis_cbf_harness.md` | AEGIS CBF + 规划器 + 反馈 + 记忆（H、H+M） |
| `09_step_budget.md` | 步数口径；A、AEGIS 的 900 步结果；官方 π0.5 对比 SFT 版 |
| `10_memory_ablation.md` | 记忆消融（H+E） |
| `11_cbf_signal.md` | 离线：过滤器干预能不能预示碰撞 |
| `12_handoff_check.md` | 交接检查（无效）；r2 重复运行的波动 |
| `13_link_contacts.md` | 碰撞部位分析；连杆约束（冻结） |
| `14_new_instructions.md` | 新指令上的 AEGIS-SFT |

逐回合数据：`data/p2/`（B0、B1、B2），`data/cbf/`（H 系列、A、AEGIS）。新指令的结果在服务器上：`runs/newinstr/aegis_sft_900/`。

## task 0 上的汇总（16 个回合：8 个场景 × episode 1、2）

| 组 | TSR | CAR | SSR | SSR@900 | 官方步数内 SSR |
|---|---|---|---|---|---|
| A-SFT（只有 VLA，900 步） | 13 | 3 | 3 | 3 | 3 |
| AEGIS-官方 π0.5（900 步） | 15 | 12 | 12 | 12 | 12 |
| AEGIS-SFT（900 步，r1 / r2） | 15 | 15 | 14 | 14 / 14 | 12 |
| B0 原版 Harness | 16 | 11 | 11 | 10 | 2 |
| B1 B0 + 安全提示 | — | — | — | 13 | 2 |
| B2 SafeHarness | 16 | 16 | 16 | 12 | 2 |
| H（r1 / r2） | 16 / 15 | 13 / 12 | 13 / 12 | 12 / 9 | 3 / 3 |
| H+E | 16 | 13 | 13 | 13 | 4 |
| H+M（r1 / r2） | 16 / 14 | 15 / 13 | 15 / 13 | 14 / 11 | 3 / 1 |

规划器类的组，同一配置两次运行能差 3 个回合，差一两个回合不能算差别。

## 已作废的工作

- LIBERO-Safety 阶段：控制器被改过，结论全部作废。
- SafeLIBERO 预实验（10-28 至 10-30）：规划器能看到基准名称。
- p2 第一次尝试（10-30）、第二轮第一次启动（10-31）：都是基础设施故障，存档在 `/homeL/dong/dash_archive/`。

## 相关工作

- SafeHarness（arXiv 2609.20822）：冻结的 π0.5，在 SafeLIBERO 上用包围盒检查路线，900 步。它的基线数字照抄 AEGIS 的 300 / 550 步结果。
- AEGIS / VLSA（arXiv 2512.11891）：VLA + 末端椭球 CBF，300 / 550 步。

## 固定协议

- **VLA 权重**：`RLinf-Pi05-LIBERO-130-fullshot-SFT`，每次推理执行 5 个动作。AEGIS-官方组用 openpi `pi05_libero`。
- **规划器和障碍物命名**：`gpt-6.1-sol`，reasoning `low`，所有组一致。
- **原版 Harness 与 B2**：与上游 RPent 一致；B2 冻结在 v7（标签 `b2-final`）。
- **记忆**：从官方 RPent 记忆（`47f301ef`）开始。
- **规划器能看到的信息**：看不到基准名称和安全评测信息。碰撞判定只在环境进程里计算。
- **新指令**：用 `SL_VARIANT=<名字>` 加载（`runs/newinstr/bddl/variants.json`）。只改语言指令和成功条件。
- **做实验的规矩**：一次只改一个部分；先在新设置上跑基线，再考虑改方法。
