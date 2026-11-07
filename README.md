# AI 画板工具包（嘉立创EDA 专业版 + Freerouting）

> **English summary.** A field-tested kit for letting an AI agent design a PCB end to end in EasyEDA Pro
> (嘉立创EDA 专业版): requirements → part selection and pin assignment → schematic built from JSON page specs and
> verified pin by pin → placement → Freerouting autorouting with parameter variants → ground pours and stitching
> → DRC → Gerber / drill / BOM / pick-and-place export with an automated release check. It drives EasyEDA through
> the `easyeda-agent` CLI. Proven on a real 2-layer 80 × 87 mm STM32F407 controller board (87 nets, 90 parts):
> DRC 0, all nets connected, rebuilt from scratch in one day. Start with `AGENTS.md` if you are an agent,
> with "快速开始" below if you are a human. Docs are in Chinese.

这是一套**让 AI agent 从零画出一块能下单的板子**的工具和方法。它不是概念演示：里面每一步都在一块真实的板子上跑通过，踩过的坑都已经改进成脚本或写进了检查项。

**实际战绩**（示例 [examples/nuedc-car-2026](examples/nuedc-car-2026)）：电赛小车上板，2 层、80 × 87 mm、天空星 STM32F407 核心板插座 + 传感/交互/串口，90 个元件、87 个网络。

- 原理图 11 页（上下两块板），每页读存盘文件逐脚核对，0 问题
- 上板：DRC 0，87/87 网络连通，两面 GND 铺铜 + 缝合孔，地面评分 94.2
- 工作区丢失后，用这套流程**一天内**从空工程重建到上述状态

## 流程总览

```
需求 ──► 方案/选型 ──► 引脚分配 ──► 原理图规格(JSON) ──► 建图+逐脚核对
                                                          │
出板文件+自动检查 ◄── DRC+连通+评分 ◄── 铺地+缝合 ◄── 布线(Freerouting 多版本) ◄── 布局 ◄── 导入PCB+板框+规则
```

| 阶段 | 人做什么 | AI 做什么 | 工具 | 过关标准 |
|---|---|---|---|---|
| 1 需求与方案 | 回答选择题、拍板 | 写需求文档、电源方案、给 2–3 个选项 | [templates/requirements.md](templates/requirements.md) | 需求分级（P0/P1/P2）都已确认 |
| 2 选型与引脚 | 确认关键器件 | 查 LCSC 料号/库存，写引脚表并自检 | `easyeda lib`、[templates/pinmap_template.py](templates/pinmap_template.py) | 0 重脚、0 外设冲突 |
| 3 原理图 | 看图 | 写每页规格 JSON → 建图 → 读存盘文件核对 | `kit/sch/` | 每页 0 problems；0 单脚网络 |
| 4 布局 | 定机械约束、调整 | 按功能分区出坐标表、摆放、查重叠 | `kit/pcb/` | 无重叠、无出界、不压安装孔 |
| 5 布线与铺地 | 看前几名快照选方向 | 多参数并行布线、打分、后处理、导入、铺铜、缝合 | `kit/route/` | 0 未布、DRC 0、连通 0 断开 |
| 6 出板 | 下单（只能人做） | 导出 Gerber/钻孔/BOM/坐标/网表 + 自动检查 + 备份 | `kit/fab/` | release_check 无 FAIL |

每个阶段的做法、命令和坑写在 [docs/](docs/)。

## 快速开始

1. **准备环境**（Windows）：嘉立创EDA 专业版 V4.1.60+、easyeda-agent CLI v1.8.x、Python 3.11+、Freerouting 2.5.0。详见 [docs/00_准备环境.md](docs/00_准备环境.md)。
   ```bash
   pip install -r requirements.txt
   python tools/check_env.py
   ```
2. **建空工程**：在嘉立创EDA 里新建工程（每块板一个 Board = 原理图 + PCB），保持打开。
3. **写配置**：复制 [board.example.toml](board.example.toml) 为 `board.toml`，填工程/PCB UUID、板子尺寸、安装孔、电源线宽、布线参数。
4. **交给 AI**：让你的 agent（Claude Code 等）读 [AGENTS.md](AGENTS.md)，从阶段 1 开始。人只需要在每个阶段末尾看结果、做选择。
5. **自己跑**：每个阶段的命令都能手动执行，见 [docs/](docs/) 或示例 [examples/nuedc-car-2026/README.md](examples/nuedc-car-2026/README.md)。

## 仓库结构

| 路径 | 内容 |
|---|---|
| [AGENTS.md](AGENTS.md) | 给 AI agent 的操作手册：阶段门、硬规则、每步命令和验收 |
| [board.example.toml](board.example.toml) | 板子配置模板（所有脚本都读它，不再写死 UUID 和尺寸） |
| [docs/](docs/) | 分阶段方法说明 + 踩坑清单 + 备份与恢复 |
| [templates/](templates/) | 需求文档、引脚表、原理图页规格、布局坐标表的模板 |
| `kit/common.py` | 读配置、调用 easyeda（重试/超时）、单位换算、PCB 快照 |
| `kit/sch/` | 原理图：规格检查、逐页建图、读存盘文件逐脚核对、删件 |
| `kit/pcb/` | 板框和孔、设计规则 + 网络线宽、按坐标表摆放、按快照复位；`guard.py`：给 auto-place / route-short / beautify 加快照、读回和自动回滚，按快照恢复走线 |
| `kit/route/` | DSN 预处理、Freerouting 多版本并行、成束布线（`route_bundle.py`，低速板「线成束、地成片」）、SES 检查与拉直、后处理、导入、铺地、缝合、GND 孤岛补接、连通、地面评分、出图 |
| `kit/fab/` | 出板导出（Gerber/钻孔/BOM/坐标/IPC-356 + 工程备份）和出板检查 |
| `kit/backup.py`、`kit/recover/` | 工程备份；工作区丢失后从 AI 会话记录恢复文件 |
| `tools/check_env.py` | 环境自检 |
| [examples/nuedc-car-2026/](examples/nuedc-car-2026/) | 完整示例：配置、11 页规格及其生成脚本、布局快照、成品数据和图 |

## 适用范围和限制

- **只支持嘉立创EDA 专业版**，通过 easyeda-agent CLI 操作，需要 EDA 窗口开着。
- **2 层板全流程已实测**；4 层板（内电层）脚本支持配置，但布线/铺地流程没有实测过。
- 自动布线适合中低密度数字/控制板。**射频、高速差分、大电流功率回路要人工布**或至少人工审查（`kit/route/` 只做通用规则）。
- 器件封装以嘉立创库为准；库里的封装会更新（本示例的测试点就变过），**布局前要核对焊盘尺寸**。
- 本工具包**不会下单**：EasyEDA 有下单接口，脚本从不调用，下单必须由人在嘉立创完成。

## 许可

本仓库以 [MIT 许可证](LICENSE) 发布：可以自由使用、修改、分发和商用，保留版权声明即可。

第三方软件不随本仓库分发，各自遵循自己的许可：

- **Freerouting**：GPL-3.0，需自行下载（见 [docs/00](docs/00_准备环境.md)）。本仓库只通过命令行调用它，不包含它的代码。
- **嘉立创EDA 专业版、easyeda-agent CLI**：按各自的用户协议使用。示例里引用的元件库 UUID、LCSC 料号指向嘉立创/立创商城的库，库数据本身不在本仓库里。
