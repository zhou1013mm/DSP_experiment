# E5 FDAF 系统辨识：导出数据分析

> 数据目录：`D:\DSP_experiment\E5_FADF\postprocess_test\partial\data`  
> 图目录：`D:\DSP_experiment\E5_FADF\postprocess_test\partial\analysis_figs`  
> 平台：fs = 48000 Hz，SC589 Core1 450 MHz；1 块 = 256 点 = 5.333 ms；实时块预算 5.33 ms，DMA 单次回调期限 2.67 ms。

## 1 导出数据与实时模式

| 变量 | 状态 |
|---|---|
| `gE5Hhat.dat` | 已读取 |
| `gE5HtrueT.dat` | 已读取 |
| `gE5Timing.dat` | 已读取 |
| `gE5Debug.dat` | **缺失** |
| `gE5BenchMSE.dat` | **缺失** |
| `gE5RtMSE.dat` | **缺失** |
| `gRawCapture.dat` | **缺失** |
| `gProcessedCapture.dat` | **缺失** |
| `gProcessed1024Capture.dat` | **缺失** |

- 实时模式：**进阶（分区 1024 点）**（`gE5Timing[11]` = 1024）
- 约束标志：**约束**（`gE5Timing[12]` = 1）
- μ = 1.50e-05；实时块数 = 0；benchmark 完成标志 = 0

## 2 benchmark 变体：耗时与精度

| 变体 | cycles/块 | cycles/点 | ms/块 | 折合 256 点 ms | 满足 5.33 ms | 失配 / dB | 收敛 -20 dB（块 / ms） |
|---|---:|---:|---:|---:|:---:|---:|---:|
| 标准256-约束 | 1,420,000 | 5,547 | 3.16 | 3.16 | 是 | -31.0 | 未达到 |
| 标准256-不约束 | 1,260,000 | 4,922 | 2.80 | 2.80 | 是 | — | 未达到 |
| 分区1024-约束 | 2,060,000 | 2,012 | 4.58 | 1.14 | 是 | -34.0 | 未达到 |
| 分区1024-不约束 | 1,930,000 | 1,885 | 4.29 | 1.07 | 是 | — | 未达到 |
| 标准1024(N=2048)-约束 | 3,050,000 | 2,979 | 6.78 | 1.69 | 是（按 4 分区等效） | -30.5 | 未达到 |

说明：变体 4（标准 1024 点，N=2048）按 1024 点块统计，与 hop=256 的变体不能直接比 cycles/块，故给“折合 256 点 ms”；收敛到 -20 dB 取 10 块滑动平均首次 ≤ -20 dB 的块序号。

## 3 实时链路（S0/S1/S2 分阶段）

| 阶段 | cycles（估算） | ms | 满足 2.66 ms |
|---|---:|---:|:---:|
| S0（收集 + FFT X） | 420,000 | 0.93 | 是 |
| S1（2 中变换，出 y/d/e） | 840,000 | 1.87 | 是 |
| S2（梯度 + 约束 + 更新） | 840,000 | 1.87 | 是 |

- 实时块平均：2,100,000 cycles = 4.67 ms （预算 5.33 ms：满足）
- 首块：2,250,000 cycles = 5.00 ms；最大：2,500,000 cycles = 5.56 ms
- 阶段 cycles 无单独导出变量，按变换数 1:2:2（S0:S1:S2）从 `gE5Timing[8]` 分摊估算；阶段实测请以 `gE5Debug` 计数为佐证。

### 实时学习曲线

- `gE5RtMSE` 无有效数据。

## 4 250 ms 内部采集

采集数组不全，未统计。

## 5 辨识精度

导出真值对应未知系统：**系统 B（1024 点混响）**。

| 比较范围 | 失配（10log10 Σe²/Σh²） | 峰值相对误差 | 峰值处逐点最大相对误差 | 对齐 lag |
|---|---:|---:|---:|---:|
| 前 256 点 | -39.83 dB | 1.34 % | 22.2 % | 0 点 |
| 前 512 点 | -37.26 dB | 1.34 % | 78.5 % | 0 点 |
| 全 1024 点 | -33.99 dB | 1.34 % | 90.8 % | 0 点 |

幅频响应（`e5_freq_response.png`）：

- 前 256 点（512 点 FFT，对照系统 A 长度）：6 kHz：真值 -27.7 dB / 辨识 -27.7 dB；12 kHz：真值 -23.4 dB / 辨识 -23.5 dB
- 系统 B：全 1024 点（1024 点 FFT）：6 kHz：真值 -28.0 dB / 辨识 -28.2 dB；12 kHz：真值 -24.0 dB / 辨识 -24.1 dB

## 6 图

- 5 变体学习曲线：`e5_learning_curves.png` 未生成

![系统前 256 点冲激响应](../analysis_figs/e5_ir_256.png)

![系统 512/1024 点冲激响应](../analysis_figs/e5_ir_1024.png)

![真值 vs 辨识幅频响应](../analysis_figs/e5_freq_response.png)

- 实时学习曲线：`e5_rt_learning.png` 未生成

- 250 ms 时域波形：`e5_capture_time.png` 未生成

- 250 ms 采集频谱与系统频响：`e5_capture_spectrum.png` 未生成

![耗时柱状图](../analysis_figs/e5_timing.png)

## 7 数据缺失说明

- gE5Debug.dat 缺失
- gE5BenchMSE.dat 缺失
- gE5RtMSE.dat 缺失
- gRawCapture.dat 缺失
- gProcessedCapture.dat 缺失
- gProcessed1024Capture.dat 缺失

