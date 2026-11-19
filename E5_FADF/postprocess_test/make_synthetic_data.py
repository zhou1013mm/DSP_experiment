# -*- coding: utf-8 -*-
"""生成 E5(FDAF 系统辨识) 后处理用的合成测试数据。

输出 CCES 导出格式：每个字节一行的小端十六进制（0xXX），
变量名/长度严格按 docs/E5_FADF_设计说明.md §9：
    gE5Hhat float[1024], gE5HtrueT float[1024],
    gE5Timing float[16], gE5Debug uint32[16],
    gE5BenchMSE float[5*300], gE5RtMSE float[512],
    gRawCapture int16[12000], gProcessedCapture int16[12000],
    gProcessed1024Capture int16[12000].

数据构造思路（“看起来合理”的模拟结果）：
  * 未知系统用设计说明的系统 B（1024 点类房间混响，L1 归一化 0.9）；
  * 辨识 hhat = htrue + 小噪声，失配约 -34 dB；
  * 5 个变体的 benchmark 学习曲线 = 指数收敛 + 起伏，变体 4 仅 150 块有效；
  * 实时学习曲线 460 块有效、末尾补 0（测试脚本的“跳过结尾 0”逻辑）；
  * 250 ms 采集：x = 白噪声 + 1k/2k/6k 小测试音，d = htrue*x，
    e = d - hhat*x（同一缩放系数存 int16）。

用法：python make_synthetic_data.py
"""
import os

import numpy as np

BASE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(BASE, "data")
os.makedirs(OUT, exist_ok=True)

FS = 48000.0
BLOCK = 256
N_CAP = 12000
SYS_L1 = 0.9
SCALE = 24000.0          # 内部 float -> int16 的统一缩放系数

rng = np.random.default_rng(20261119)


# ---------------------------------------------------------------------------
def xorshift32(seed, n):
    """标准 xorshift32 -> [-0.5, 0.5)。"""
    s = seed & 0xFFFFFFFF
    out = np.empty(n)
    for i in range(n):
        s ^= (s << 13) & 0xFFFFFFFF
        s ^= s >> 17
        s ^= (s << 5) & 0xFFFFFFFF
        s &= 0xFFFFFFFF
        out[i] = s / 4294967296.0
    return out - 0.5


def normalize_l1(h):
    return h / np.sum(np.abs(h)) * SYS_L1


def sys_a():
    n = np.arange(256)
    k = n - 2
    h = np.zeros(256)
    h[2] += 0.70
    m = k >= 0
    h[m] += 0.35 * np.exp(-k[m] / 40.0) * np.cos(2 * np.pi * 1800.0 * k[m] / FS)
    return normalize_l1(h)


def sys_b():
    r = xorshift32(0x1234567, 1024) * 2.0      # 零均值均匀，幅度 1
    n = np.arange(1024)
    k = n - 2
    h = np.zeros(1024)
    h[2] += 0.60
    m = k >= 0
    h[m] += np.exp(-k[m] / 150.0) * r[m]
    return normalize_l1(h)


def write_bytes(name, b):
    with open(os.path.join(OUT, name), "w", encoding="ascii") as f:
        f.write("\n".join("0x%02X" % x for x in b))
        f.write("\n")


def write_array(name, arr, dtype):
    a = np.asarray(arr, dtype=dtype)
    write_bytes(name, a.tobytes())


# ---------------------------------------------------------------------------
# 1. 未知系统与辨识结果
# ---------------------------------------------------------------------------
hA = sys_a()
hB = sys_b()
htrue = hB.copy()

# hhat = htrue + 小噪声，目标失配约 -34 dB
sum_h2 = float(np.sum(htrue ** 2))
target_mis_db = -34.0
noise_std = np.sqrt(sum_h2 * 10 ** (target_mis_db / 10.0) / len(htrue))
hhat = htrue + rng.normal(0.0, noise_std, len(htrue))
# 加一点低阶端的系统偏差，让曲线更“真实”
hhat[:8] += rng.normal(0.0, noise_std * 0.5, 8)
mis_true = 10 * np.log10(np.sum((hhat - htrue) ** 2) / sum_h2)

# ---------------------------------------------------------------------------
# 2. benchmark 学习曲线：5 变体 x 300 块
# ---------------------------------------------------------------------------
floors = np.array([-31.5, -24.0, -33.8, -25.5, -28.2])   # 各变体稳态 MSE
taus = np.array([70.0, 55.0, 85.0, 65.0, 110.0])         # 收敛时间常数（块）
starts = np.array([0.5, 0.5, 1.0, 1.0, 1.5])
bench = np.zeros((5, 300))
m = np.arange(300, dtype=float)
for i in range(5):
    curve = floors[i] + (starts[i] - floors[i]) * np.exp(-m / taus[i])
    curve += rng.normal(0.0, 0.7, 300)
    bench[i] = curve
bench[4, 150:] = 0.0        # 变体 4 只跑 150 块（其余导出为 0）

# ---------------------------------------------------------------------------
# 3. 实时学习曲线：512 点，前 460 块有效
# ---------------------------------------------------------------------------
rt = np.zeros(512)
nrt = 460
mrt = np.arange(nrt, dtype=float)
rt[:nrt] = -30.0 + (1.0 + 30.0) * np.exp(-mrt / 80.0) + rng.normal(0.0, 0.8, nrt)

# ---------------------------------------------------------------------------
# 4. gE5Timing / gE5Debug
# ---------------------------------------------------------------------------
cyc = [1.42e6, 1.26e6, 2.06e6, 1.93e6, 3.05e6]     # cycles/块（变体 4 为 1024 点块）
timing = np.zeros(16)
timing[0:5] = cyc
timing[5] = -31.0        # std256 失配（系统 A，benchmark 记录）
timing[6] = mis_true     # part1024 失配（系统 B）
timing[7] = -30.5        # std1024 失配
timing[8] = 2.10e6       # 实时块平均 cycles
timing[9] = 512.0        # 实时块数
timing[10] = FS
timing[11] = 1024.0      # 实时模式：进阶（分区 1024 点）
timing[12] = 1.0         # 约束
timing[13] = 1.5e-5      # mu
timing[14] = 2.25e6      # 首块
timing[15] = 2.50e6      # 最大块

debug = np.zeros(16, dtype=np.uint32)
debug[0] = 2048
debug[1] = 512
debug[2] = 512
debug[3] = 512
debug[4] = 512
debug[5] = 512
debug[6] = 6000          # AdcCount
debug[7] = 6000          # DacCount
debug[8] = 0             # bEventError = 0
debug[9] = 1234
debug[10] = 1            # gCaptureActive
debug[11] = N_CAP        # gCaptureFramesCaptured
debug[12] = int(np.sum(hhat ** 2) * 1e6)
debug[13] = int(np.sum(1e-3 ** 2) * 1e6)
debug[14] = int(2.30e6)
debug[15] = 1            # bench 完成

# ---------------------------------------------------------------------------
# 5. 250 ms 采集（x / d / e），同一缩放系数
# ---------------------------------------------------------------------------
n = N_CAP
x = 0.5 * xorshift32(0x0BADF00D, n)          # [-0.25, 0.25) 白噪声
t = np.arange(n) / FS
x += 0.05 * np.sin(2 * np.pi * 1000.0 * t)
x += 0.05 * np.sin(2 * np.pi * 2000.0 * t)
x += 0.04 * np.sin(2 * np.pi * 6000.0 * t)
d = np.convolve(x, htrue)[:n]
e = np.convolve(x, htrue - hhat)[:n]         # 与 d 严格对齐的残余误差


def to_i16(v):
    return np.clip(np.round(v * SCALE), -32768, 32767).astype(np.int16)


xd, dd, ed = to_i16(x), to_i16(d), to_i16(e)
rms = lambda v: float(np.sqrt(np.mean(np.asarray(v, float) ** 2)))    # noqa: E731
ratio_db = 20 * np.log10(rms(ed) / rms(dd))

# ---------------------------------------------------------------------------
# 6. 写文件
# ---------------------------------------------------------------------------
files = [
    ("gE5Hhat.dat", np.concatenate([hhat, np.zeros(0)])[:1024], "<f4"),
    ("gE5HtrueT.dat", htrue, "<f4"),
    ("gE5Timing.dat", timing, "<f4"),
    ("gE5Debug.dat", debug, "<u4"),
    ("gE5BenchMSE.dat", bench.reshape(-1), "<f4"),
    ("gE5RtMSE.dat", rt, "<f4"),
    ("gRawCapture.dat", xd, "<i2"),
    ("gProcessedCapture.dat", ed, "<i2"),
    ("gProcessed1024Capture.dat", dd, "<i2"),
]
for name, arr, dt in files:
    write_array(name, arr, dt)
    print(f"[写出] {name:<28s} {len(np.asarray(arr).reshape(-1)):6d} 点  "
          f"{os.path.getsize(os.path.join(OUT, name)) / 1024.0:8.1f} KiB")

print(f"\n合成数据目录: {OUT}")
print(f"系统 A 前 4 点: {np.array2string(hA[:4], precision=4)}")
print(f"系统 B 前 4 点: {np.array2string(hB[:4], precision=4)}")
print(f"系统 B 峰值: {np.max(np.abs(hB)):.4f}，L1 = {np.sum(np.abs(hB)):.4f}")
print(f"hhat vs htrue 失配 = {mis_true:.2f} dB（目标 {target_mis_db:.0f} dB）")
print(f"采集 e/d RMS 比例 = {ratio_db:.2f} dB（{100 * rms(ed) / rms(dd):.3f} %）")
print(f"bench 曲线末块: {np.round([bench[i, bench.shape[1] - 1] for i in range(4)], 1)}"
      f" / 变体4 第150块 {bench[4, 149]:.1f} dB")
print(f"实时曲线末块 {rt[nrt - 1]:.1f} dB，之后 {512 - nrt} 点为 0")
