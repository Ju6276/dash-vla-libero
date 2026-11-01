# 步数上限：原文口径，以及统一 900 步的基线重跑（2026-11-01）

本文的设计写于运行之前。

## 原文怎么规定步数

| 论文 | 每回合步数上限 | 出处 |
|---|---|---|
| AEGIS / VLSA（arXiv 2512.11891） | spatial、object、goal 300 步，long 550 步。开局 20 个等待步不计入 | §V-A："Maximum episode horizons are 300 steps for the Spatial, Goal, and Object suites, and 550 for the Long suite."；`main/main_aegis.py` 第 234 行在等待后把 `t` 置零 |
| SafeHarness（arXiv 2609.20822） | 所有套件都是 900 步 | §4.1："On SafeLIBERO, an episode is allowed at most 900 environment steps; a successful run takes about 400 steps on average, and about 700 on Libero-10." |
| Harness VLA / RPent（arXiv 2607.08448） | 论文没有给出数字，只说部署时的步数上限"大幅缩短"。公开代码的默认值是 10000 步（`--max-episode-steps`） | §2.1、§2.2、附录 C；代码 `robots/libero/robot_spec.py`、`env_server.py` |

- SafeHarness 表 1 里 π0.5、OpenVLA-OFT、AEGIS 的数字和 AEGIS 论文表 I 完全一样，看起来是直接抄来的。也就是说，基线用的是 300 / 550 步，SafeHarness 自己用的是 900 步，两者步数上限不同。论文没有说明这一点。
- AEGIS 的策略是 openpi 的 π0.5-LIBERO（`pi05_libero`）。

## 我们统计里的一个错误

我们之前统计"官方步数内"时，把 20 个等待步也算进了上限，控制步只给了 280 / 530。A 组（只有 VLA）当时也是按 280 / 530 跑的。现在改为 300 / 550：
- `scene_env.py` 和 `run_vla_only.py` 已改；
- `analyze_harness.py` 根据记录下来的成功步和碰撞步重新计算，所以已经跑完的回合不用重跑。

## 这一轮：所有组统一 900 步

参照 SafeHarness 的 900 步，但这次让基线也用 900 步，各组上限相同。同样是 8 个场景 × episode 1、2，结果写在 `runs/budget_round.log`。

| 组 | 内容 | 输出 |
|---|---|---|
| AEGIS-官方 | 按 AEGIS README 复现：AEGIS 自带的 openpi（`serve_policy.py --env LIBERO`），官方 `pi05_libero`（JAX），原版策略客户端。只有障碍物命名换成 Codex sol | `runs/aegis/p2_official_900` |
| AEGIS-SFT | 和报告 07 相同（RLinf SFT 版 VLA），只是上限改为 900 | `runs/aegis/p2_900` |
| A-SFT | 只有 VLA（RLinf SFT），上限 900 | `runs/vla_only/p2_900` |

- 900 步的上限通过替换 `eval_libero` 里四个按套件设定的 `max_steps` 实现，这是对原版源码文本唯一的改动。
- 官方步数内的结果从同一批回合里读出：一次 300 / 550 步的官方运行，前面的轨迹和 900 步运行完全相同，到成功那一步就会停止。所以只要成功步在 300 / 550 以内、且成功之前没有碰撞，就算作官方步数内成功且无碰撞。
- 这样一共有两种对比口径：900 步内所有组比较；官方步数内所有组比较。另外，"AEGIS-官方 对比 AEGIS-SFT"可以看出换用 SFT 版 VLA 的影响。

冒烟测试（spatial_II e1，AEGIS-官方，300 步）：任务成功，无碰撞，用了 122 步；Codex 说出的障碍物是 red milk carton。

## 结果（2026-10-31 23:28 跑完）

运行情况：
- AEGIS-官方的第一个回合（spatial_I e1）出了基础设施故障：第一次推理要编译 JAX 模型，耗时超过 websocket 的 keepalive 时限，回合在第一步之前就中断了。这条记录已移到 `infra_failures.jsonl`。`run_aegis.py` 现在会先用一次空推理预热，然后重跑了这个回合。
- 其余回合没有报错。

成功且无碰撞（SSR），所有组都是同样 16 个评估回合：

| 组 | TSR@900 | CAR@900 | SSR@900 | 官方步数内 SSR | 平均步数 |
|---|---|---|---|---|---|
| A-SFT：只有 VLA | 13/16 | 3/16 | 3/16 | 3/16 | 333 |
| AEGIS-官方 π0.5 | 15/16 | 12/16 | 12/16 | 12/16 | 256 |
| AEGIS-SFT | 15/16 | 15/16 | **14/16** | 12/16 | 331 |
| B1：规划器 + 安全提示 | — | — | 13/16 | 2/16 | — |
| B2：SafeHarness | — | — | 12/16 | 2/16 | 663 |
| H+M（报告 08） | — | — | **14/16** | 3/16 | 524 |

- **步数放宽到 900，AEGIS 也追上来了**：AEGIS-SFT 在 900 步内是 14/16，和 H+M 持平；官方步数内两种 AEGIS 都是 12/16。所以在 900 步口径下，H+M 目前并没有超过 AEGIS。
- **官方 π0.5 和 SFT 版的差别**：
  - 任务成功相同，都是 15/16；
  - 官方版碰撞更多：12/16 无碰撞，SFT 版是 15/16。碰撞出在 long_I 两个回合，以及 spatial_I 两个回合；
  - 官方步数内两者相同，都是 12/16。
- **只有 VLA 的组**：步数放宽到 900 后几乎没变（3/16）。它的碰撞大多发生在前 100 步。
- **同一配置两次运行，结果也会有出入**：VLA 的输出带随机采样。比如 AEGIS-SFT 在报告 07 里 spatial_I 两个回合都没完成，这次 e2 在 235 步完成了；goal_I e1 在报告 07 里 300 步内就完成了，这次用了 623 步。每个组只有 16 个回合，差一两个回合不能算差别。

记忆消融（H+E）的结果见 `10_memory_ablation.md`。
