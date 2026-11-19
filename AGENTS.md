# AGENTS.md — DSP 嵌入式实验课程：智能体工作手册

> 仓库：`D:\DSP_experiment`
> 课程：南京大学 DSP 嵌入式实验（E1–E8）
> 最后更新：2026-11-19
> **给未来会话的智能体：进入仓库先读本文件，再读对应实验的 PPT/报告/README；不要直接改代码。**

---

## 0. 为什么有这个文件

这门课的 8 个实验共用同一套 **CCES + SC589 EZ-Kit + ADC/DAC 回环** 框架。每次换实验，代码结构、宏开关、采集流程都很像，但坑也高度重复：

- 改了工作区源码，CCES 却编译另一个目录的副本；
- Build 的是 Release，Debug 却加载 Debug 的旧 dxe；
- 实时块里塞了太重的运算，超过 DMA 回填期限，`AdcCount` 卡在 6；
- 采集 warmup 没到就暂停，数组全 0；
- 500 ms 三组采集超出默认 LDF，链接报 out of memory；
- 目标板连不上（`0x80047344`），误以为是代码问题。

本文件是**课程级经验库 + 智能体操作规范**。新实验开始时先按第 2 节 checklist 走；遇到 `AdcCount` 卡住先看第 4 节；每次解决新 bug 后，把“现象 / 根因 / 修复”补到第 12 节。

---

## 1. 仓库结构

```text
D:\DSP_experiment\
├─ AGENTS.md                  ← 本文件（智能体先读）
├─ README.md                  ← 给人看的仓库简介
├─ E1_fir_iir\                ← 32 阶 FIR / 8 阶 IIR
├─ E2_resample\               ← 48k → 8k → 48k 多速率
├─ E3_fft\                    ← 32 点实时 FFT / IFFT
├─ E4_overlapsave\            ← 重叠保留法实时 FIR（255 / 1023 阶）
├─ E5_FADF\                   ← 频域自适应滤波（后续）
├─ manual\                    ← SC589 板卡/芯片/编译器手册
└─ 数字信号处理2025-郭霞生\    ← 课件
```

每个实验通常包含：

- `adc_dac_playback\`：`adc_dac_playback.c/.h` + 本实验头文件；
- `.pptx`：老师要求；
- `E<n> 实验报告.md`：报告；
- `data_*`、`analysis_figs`、`*.m` / `*.py`：采集数据与后处理。

---

## 2. 新实验（E5+）开工 checklist

按顺序做，不要跳：

1. **读材料**：本文件 → 实验 PPT → 上一实验报告 → 对应 `adc_dac_playback.c/.h`。
2. **确认源码唯一**：选一个“源目录”，CCES 工程必须编译它。推荐把 CCES 工程建在实验目录下，`src` 直接链接工作区文件；如果复用 SDK 示例工程，见第 5 节。
3. **确认构建配置**：Debug 还是 Release；`C/C++ Application` 指向哪个 `.dxe`。两者必须一致。
4. **加版本标记**：在新代码里加 `volatile uint32_t gE<n>CodeVersion = YYYYMMDD;`，初始化时打印；以后先看它判断跑的是不是新代码。
5. **加阶段计数器**：像 E4 的 `gE4Debug[16]` 一样，在实时路径的关键步骤加 `volatile` 计数；卡住时能定位到“进块 / FFT 完成 / 输出完成 / callback 错误”。
6. **Clean Build**：`Project → Clean` → `Build`；检查 `.doj` 和 `.dxe` 时间戳变成现在。
7. **不设断点全速跑 3 秒**：等 `Capture complete`；不要跑几毫秒就暂停看数组。
8. **先看 `AdcCount` 是否增长**：卡住按第 4 节诊断；不要先怀疑采集代码。
9. **量 cycles**：实时块耗时必须小于块周期；超过就减负载/拆模式/换 Release/换库 FFT，不要硬塞。
10. **采数据、导出、后处理**：按第 8 节格式；每次实验把新坑补进第 12 节。

---

## 3. 构建与调试环境

- **IDE**：CrossCore Embedded Studio 2.9.3；
- **编译器**：`C:\Analog Devices\CrossCore Embedded Studio 2.9.3\cc21k.exe`（SHARC）；
- **板卡**：ADSP-SC589 EZ-Kit Lite；
- **工程示例**：
  `C:\Analog Devices\ADSP-SC5xx_EZ-KIT_Lite-Rel2.0.2\ADSP-SC5xx_EZ-KIT\Examples\drivers\adc\adau1979\adc_dac_playback\SC589\sharc\adc_dac_playback_Core1`
- **核心**：SHARC Core1；ADC/DAC 48 kHz，4 通道，每次回调每通道 64 帧；
- **块大小**：`AUDIO_BUFFER_SIZE = 1024 B`，即 64 帧 × 4 ch × 4 B；
- **E4 的块周期**：4 次回调 = 256 点 = **5.33 ms**；
- **DMA 回填期限**：ping-pong 只有 2 个缓冲，主循环阻塞超过约 **2.66 ms** 就可能饿死 DMA。

### 3.1 Debug 与 Release

- **Debug** 默认 `-O0`，适合看变量，但 FFT/逐点 FIR 很慢；
- **Release** 默认开 `-O`，快很多，但调试符号少；
- E4 的实测数据是在 **Debug 构建**下采的（OS255 1.59 ms、OS1023 1.96 ms）；
- 如果 Debug 下实时超时，优先切 Release，或在工程设置/`subdir.mk` 里给编译器加 `-O2`。

### 3.2 每次 Build 后必须核对

```text
<工程>\Debug\src\adc_dac_playback.doj
<工程>\Debug\ADC_DAC_Playback_SC589_SHARC_Core1.dxe
```

时间戳必须是“刚才”。如果还是旧时间，说明：

- 你 Build 的是另一个配置（Release）；
- 你 Build 的是另一个工程；
- 编译失败但没看 Console；
- Debug Configuration 指向另一个 dxe。

---

## 4. 高频故障：`AdcCount` 卡住 / 采集数组全 0

### 4.1 先分清“没采”还是“采到 0”

`AdcCount` 是 ADC 回调次数，不是采集数组的填充数。正常采集：

```text
AdcCount >= 1500（2 s warmup）→ gCaptureActive=1
→ 250 ms 后 gCaptureFramesCaptured=12000 → gCaptureDone=1
```

- `AdcCount=6`、`gCaptureActive=0`、`gCaptureFramesCaptured=0`：**采集根本没开始**，不是数组写错；
- `AdcCount` 很大、`gCaptureFramesCaptured=12000`、数组仍 0：**ADC 静音或看错内存/格式**；
- `gCaptureDone=1` 但数组 0：检查输入 WAV 是否在播放、Memory 窗口是否选 Data/DM 16-bit。

### 4.2 诊断树

先看这些变量（E4 对应 `gE4Debug` / `gE4Timing`）：

| 现象 | 结论 | 处理 |
|---|---|---|
| `AdcCount` 卡在 6 左右，`bEventError=1` | 实时块处理太慢，DMA 饿死 | 减负载、拆模式、移走逐点 FIR、Release/`-O2` |
| `gE4Debug[1..4]` 都 =1，只有 `[5]` 卡 6 | 第一块完整跑完，之后 DMA 停 | 同上，量 `[14]+[15]` 是否 >1.2M cycles |
| `gE4Debug[2]=0` | 卡在 OS256 | 查 FFT/内存/编译优化 |
| `gE4Debug[3]=0` | 卡在 OS1024 | 查分区历史/FFT |
| split 模式 `gE4Debug[2]` 和 `[3]` **同时=1** | **跑的是旧二进制**（split 版只会一个为 1） | 查源码路径、Build 配置、dxe 时间戳 |
| `gE4CodeVersion` 不是当前日期 | 跑的是旧源码 | 同步源码、Clean Build |
| `gE4Timing[14]/[15]` 有值但数组 0 | 只跑了 benchmark，采集没开始 | 等 `Capture complete`，看 `gCaptureActive` |
| `AdcCount` 根本不变 | 回调没来/程序退出/断点 | 看 `bEventError`、Console、是否设断点 |

### 4.3 E4 的真实根因（已解决）

E4 一开始把 **逐点 1023 阶 FIR** 也放进每个音频块：

```text
Direct 1023 = 8,678,553 cycles/块 ≈ 19.3 ms
块周期 = 5.33 ms
```

结果 `AdcCount` 卡 6，采集数组全 0，`bEventError=1`。修复：

1. 逐点 FIR 只在 `E4_Init()` 启动时用 5 个 1k+6k 测试块跑，记录 cycles/误差；
2. 实时链路只跑一套重叠保留法；
3. 用 `E4_MODE_256` / `E4_MODE_1024` 分两次采集；
4. 内联 FFT 蝶形/复乘，减少 `-O0` 函数调用开销；
5. 最终 OS255 716k cycles（1.59 ms）、OS1023 880k cycles（1.96 ms），都能实时跑。

**教训：调试 DSP 实时问题时，先量 cycles，不要先怀疑采集代码。**

### 4.4 采集 warmup

`CAPTURE_WARMUP_SECONDS=2`，`AdcCount` 到 1500 才开始。只跑几百毫秒就暂停，数组当然是 0。E4 的 `E4 capture start` / `Capture complete` 打印就是用来确认这件事的。

---

## 5. CCES 工程与源码副本问题（最容易浪费时间）

CCES 工程很可能编译 **SDK 示例目录**里的源码，而不是你正在编辑的工作区文件。E4 就发生过：

- 工作区 `D:\DSP_experiment\E4_overlapsave\adc_dac_playback` 是最新代码；
- CCES 工程编译的是 `C:\Analog Devices\...\adc_dac_playback.c`；
- 两边不同步，Build 后 `.doj/.dxe` 时间戳不变；
- 运行时两个 OS 都跑，`gE4Debug[2]/[3]` 都=1，看起来像“算法没改”。

### 5.1 检查方法

1. Project Explorer → `src` → 右键 `adc_dac_playback.c` → Properties → Resource → Location；
2. 打开 `<工程>\Debug\src\subdir.mk`，看 `cc21k` 命令行里的绝对路径；
3. 看 `Debug\src\adc_dac_playback.doj` 时间戳；
4. 在代码里加 `gE<n>CodeVersion`，运行时读它。

### 5.2 修复原则

- **一个源目录**：要么让工程指向工作区文件，要么把工作区文件同步到工程源目录；
- 同步时 `.c`、`.h`、本实验专用头文件（如 `overlap_save_fir.h`）一起同步；
- 同步后 `Clean` + `Build`，确认 `.doj/.dxe` 时间戳变化；
- 不要把“Build 成功”当成“编译了新代码”。

---

## 6. 实时 DSP 的时限规则

每次音频回调 = 64 帧 = **1.333 ms**；E4 每 4 次回调处理一块 = **5.33 ms**。但 DMA 只有 ping-pong 两个缓冲，主循环阻塞超过约 **2.66 ms** 就可能停。

新实验设计时：

- 先算每块/每回调的预算；
- 用 `cycle_count.h` 或 `gE<n>Debug` 实测；
- 重运算（长 FIR、大 FFT、逐点对比）放到 **启动 benchmark**，不要放实时路径；
- 需要对比两套算法时，用宏 **分两次采集**，不要同时塞进实时块；
- 必要时用 Release/`-O2`，或换 `libdsp` 的优化 FFT（`rfft512`/`cfft512`/`ifft512`）。

---

## 7. E1–E4 经验摘要

| 实验 | 内容 | 关键点 |
|---|---|---|
| E1 | 32 阶 FIR / 8 阶 IIR | 计算量小，Debug 也能跑；FIR 系数按 48k、fc=2k 设计 |
| E2 | 48k→8k→48k | 143 点抗混叠 FIR + 8k 域 IIR + 143 点多相上采样；采集 pipeline |
| E3 | 32 点 FFT/IFFT | 64 帧/回调 = 2 帧 32 点；`gFft*` 观测变量 |
| E4 | 重叠保留法 FIR | N=512、hop=256、L=256；255 阶 + 1023 阶 4 分区；启动 benchmark；250 ms 采集 |
| E5 | 频域自适应滤波（FDAF）辨识 | N=512/hop=256；标准 256 点 + 分区 1024 点（4×256）；梯度约束/不约束；启动 benchmark 5 变体（含 N=2048 标准 1024 点）；实时链路拆成 S0/S1/S2 三阶段，每次回调 ≤2 次 512 点变换（Debug -O0 下 < 2.66 ms DMA 期限） |

E4 的具体宏：

```c
#define E4_OVERLAP_SAVE
//#define E4_MODE_1024
#ifndef E4_MODE_1024
#define E4_MODE_256
#endif
```

E4 的数据导出（250 ms = 12000 点）：

```text
gRawCapture.dat              int16 × 12000
gProcessedCapture.dat        int16 × 12000（本次模式输出）
gProcessed1024Capture.dat    int16 × 12000（256 模式全 0）
gE4Timing.dat                float × 13
```

E4 的 `gE4Timing` 下标：

```text
[0] OS256 cycles/块     [1] Direct256
[2] OS1024              [3] Direct1024
[4] 256 加速比          [5] 1024 加速比
[6] 块数                [7] 256 相对误差
[8] 1024 相对误差       [9] 256 绝对误差
[10] 1024 绝对误差      [11] 采样率
[12] 本次模式（256/1024）
```

E4 的 `gE4Debug` 下标：

```text
[0] ProcessBuffers 进入 E4 次数
[1] 进入 256 点块次数
[2] OS256 完成次数      [3] OS1024 完成次数
[4] 输出循环完成次数
[5] AdcCount            [6] DacCount
[7] bEventError         [8] gE4InCount
[9] gE4OutRd            [10] gE4Timing[6]
[11] gE4BlockCount      [12] gCaptureActive
[13] gCaptureFramesCaptured
[14] 第一块 OS256 cycles [15] 第一块 OS1024 cycles
```

---

## 8. 数据导出与后处理约定

- CCES 导出格式：**每行一个字节，小端**（hex 或十进制均可）；
- E4 每次 250 ms，12000 点；三个 `int16` 数组各 24000 字节；
- `gE4Timing` 13 个 float = 52 字节；
- `data_256/`、`data_1024/` 分开放；
- 后处理脚本：`e4_postprocess.py`（Python，生成报告图）与 `analyze_e4.m`（MATLAB 版）；
- 图统一放 `analysis_figs/`，报告引用相对路径；
- 每次实验建议保留 `data_*`，方便复现。

---

## 9. 代码风格与约定

- 新实验在 `adc_dac_playback.h` 末尾加宏，格式：

```c
// E<n>
#define E<n>_XXX
//#define E<n>_YYY
```

- 算法代码放独立头文件（如 E1 `fir_iir.h`、E2 `lpf_down_then_up.h`、E3 `my_fft.h`、E4 `overlap_save_fir.h`）；
- 实时回调里**不要 malloc/free**，用 static/全局状态；
- 跨回调保持的状态加 `static`，观测/导出变量加 `volatile`；
- 大数组注意 LDF 内存：E4 的 3 个 500 ms 采集数组会超默认 LDF，250 ms 可放下；
- 不要为了“跑通”直接改 `SAMPLE_RATE`；先确认 `SAMPLES_PER_PERIOD`、缓冲区、ADC/DAC 配置是否还成立。

---

## 10. 目标板连接问题

典型错误：

```text
[TpsdkServer] Failed to load file: ...ezkitSC589_preload_core0_v01
[Error: Failed to connect to target., Code=0x80047344]
```

这是 **CCES/仿真器连不上目标板**，不是代码错。排查顺序：

1. 完全关闭 CCES，结束 `ccstudio.exe` / `TpsdkServer.exe`；
2. 拔 USB 和 5V，等 10 秒，重新上电；
3. 换 USB 数据线/USB 2.0 口，确认插在调试口；
4. 设备管理器看 `ADI ICE-1000/2000` 是否有黄色感叹号；
5. Debug Configuration → Target：Processor `ADSP-SC589`，Emulator 选实际仿真器，点 **Test**；
6. 检查 boot mode 拨码、复位键、板子是否在跑 Linux/ARM 程序；
7. 用 ADI 自带 SC589 Hello World 验证连接；仍失败就修驱动/CCES 或换电脑。

---

## 11. 给未来智能体的硬性规则

1. **先读本文件**，再读对应实验的 PPT/报告/源码。
2. **不要假设代码改动已经进入二进制**：查源码路径、`.doj/.dxe` 时间戳、`gE<n>CodeVersion`。
3. **不要假设 Build 配置**：Debug/Release 的 dxe 可能不是同一个；查 `C/C++ Application`。
4. **`AdcCount` 卡住先量 cycles**：实时块是否超过 5.33 ms / DMA 2.66 ms。
5. **不要把逐点对比/大 FFT 放进实时块**：启动 benchmark 或分模式。
6. **不要直接改 `SAMPLE_RATE`**：先检查缓冲区宏和驱动配置。
7. **不要同时开 E1/E2/E3/E4 的重代码**：用宏隔离，测哪个开哪个。
8. **每次新 bug 解决后更新第 12 节**：写清现象、根因、修复、验证。
9. **保持一个源目录**：工作区与 CCES 工程源码必须同步，或工程直接指向工作区。
10. **新实验加版本标记和阶段计数器**，比反复猜快得多。

---

## 12. 变更日志 / 故障档案

### 2026-11-19 — E5 FDAF 系统辨识（代码完成，含两个算法级坑与一条 LDF 内存坑）

- **内容**：新增 `E5_FADF/adc_dac_playback/fadf.h`（512/2048 点 FFT、白噪声、未知系统 A/B、标准/分区 FDAF、梯度约束、启动 benchmark、实时三阶段链路），修改 `adc_dac_playback.c/.h` 挂上 E5 段；后处理 `E5_FADF/e5_postprocess.py`；主机端参考模型 `E5_FADF/analysis_host/`。
- **坑 1（算法）**：误差帧必须 `ef=[0, e_m]`（前半零填充）。按最初写法 `[e_{m-1}, e_m]`，`IFFT(conj(X)E)` 前 256 点不等于逐点梯度 `g[k]=Σ e(mL+i)x(mL+i-k)`（参考模型实测相对误差 1.04；零填充版 8.6e-8）。零填充还省掉 e 历史缓存。
- **坑 2（算法）**：约束操作必须保留 `g(0..255)`、后一半置零；反方向时 W 的时域前 256 点永远不更新（失配恒 0 dB）。
- **坑 3（步长）**：初稿 μ=1.5e-5 在 300 块内几乎不收敛（-0.8 dB）。主机模型扫描后：标准/分区 256 点分块 μ=8e-4、标准 1024 点（N=2048）μ=2e-4、分区不约束 μ=6e-3。
- **坑 4（内存/LDF）**：E5 工作区 + 3 个 250 ms 采集数组约 246 KB，默认 LDF 的 block0 BW 区（~176 KB）放不下，链接报 `li1040 Out of memory`。用 `#pragma section("seg_bsz_data")` 把 `gE5Work/gE5State` 放到 block0..3 的零初始化段（LDF 已映射 `seg_bsz_data`），链接时自动分散到多个 block；会有 `li2131 incompatible init qualifier` 警告，数组在 `E5_Init()` 里显式 memset，无影响。再放不下就把 `CAPTURE_MILLISECONDS` 改 125。
- **验证**：`cc21k` 按工程 Debug 选项实测编译通过、链接生成 832 KB dxe；主机端参考模型 5 个变体各 300 块：失配 -44.8/-44.9/-43.3/-28.0/-44.6 dB。
- **无板模拟器**：`E5_FADF/sim/SIM_NOTES.md` 记录 CCES SC589 模拟器（Functional-Sim）能加载执行 dxe，但 `CCES_runner` 导出内存与 `Cycle-Accurate-Sim` 测 cycles 需要写 `%LOCALAPPDATA%\Analog Devices` / `C:\ProgramData\FLEXnet`，普通权限下或在 CCES GUI 的 Simulator 调试配置里做。
- **E5 宏**：`#define E5_FADF`，`E5_MODE_PART`（默认，1024 点分区）/`E5_MODE_STD`（256 点标准）二选一，`E5_NO_CONSTRAINT`、`E5_PART_CONSTRAIN` 可选。
- **结果数据**：请按 `E5_FADF/E5_使用说明.md` §6/§7 导出并用 `e5_postprocess.py` 出图。

### 2026-11-19 — E4 `AdcCount=6`、采集全 0

- **现象**：`AdcCount=6`，`gCaptureActive=0`，三个采集数组全 0；`gE4Debug[2]` 和 `[3]` 同时为 1，`[14]=2,002,030`、`[15]=2,484,919`。
- **根因 1**：实时块里同时跑 OS256 + OS1024 + 两套逐点 FIR，第一块约 4.49M cycles ≈ 10 ms，超过 5.33 ms/2.66 ms，DMA 饿死，`bEventError=1`。
- **根因 2**：CCES 工程实际编译的是 SDK 目录副本，工作区改动没进去；且 Build 的是 Release，Debug 加载的是 8:03 的旧 Debug dxe，两个计数器同时为 1。
- **修复**：
  1. 逐点 FIR 移到 `E4_Init()` 启动 benchmark；
  2. 实时只跑一套 OS，用 `E4_MODE_256/1024` 分两次采集；
  3. 内联 FFT 蝶形/复乘；
  4. 同步工作区 `.c/.h/overlap_save_fir.h` 到 CCES 工程源目录；
  5. Clean + Build + 下载对应 dxe；
  6. 加 `gE4CodeVersion` 和 `gE4Debug`。
- **结果**：`AdcCount` 正常增长，采集 250 ms 成功；OS255 716k cycles（1.59 ms，3.04×），OS1023 880k cycles（1.96 ms，9.86×）；最大相对误差 4.04e-6 / 2.91e-6。

### 2026-11-12 — E4 目标板连不上

- **现象**：`TpsdkServer ... preload_core0_v01 ... Failed to connect to target., Code=0x80047344`。
- **根因**：仿真器/目标板连接失败（USB/驱动/复位/启动模式），不是 E4 代码。
- **修复**：重启 CCES、断电重上电、换 USB 口/线、Target Test、检查 boot mode；连接恢复后再下载。
- **教训**：preload 文件本身存在，报“Failed to load file”时先看括号里的 “Failed to connect to target”。

### 2026-11-09 — E4 500 ms 采集链接 out of memory

- **现象**：3 个 500 ms 采集数组 + E4 状态，SC589 默认 LDF 链接报 `Out of memory`。
- **修复**：`CAPTURE_MILLISECONDS` 改 250（12000 点），链接通过；频谱/耗时仍够用。
- **教训**：新增采集数组前先估算 LDF 余量；500 ms 三组不是默认配置能放下的。

---

## 13. 文件命名约定（agent 工程常见做法）

- **`AGENTS.md`**：本仓库的**规范入口**，Codex/DSH 等智能体默认会读；推荐作为唯一权威。
- **`CLAUDE.md`**：Claude Code 的约定；如果使用 Claude Code，可让 `CLAUDE.md` 只写一行 `见 AGENTS.md`，避免两份内容不同步。
- **`.cursor/rules/*.mdc`**：Cursor 的规则文件；同样建议引用 `AGENTS.md`。
- **`README.md`**：给人看的仓库简介/快速开始。
- **`CONTRIBUTING.md`**：人类协作规范；本课程仓库可不用。
- **`docs/troubleshooting.md`** 或 **`TROUBLESHOOTING.md`**：只放故障档案；本课程规模小，直接放本文件第 12 节即可。

**结论：本仓库用 `AGENTS.md` 作为唯一权威文档；其他 agent 的入口文件只做引用，不要复制内容。**
