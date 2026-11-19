# -*- coding: utf-8 -*-
"""E5 频域自适应滤波（FDAF）系统辨识：CCES 导出数据后处理与报告配图。

用法：
    python e5_postprocess.py [数据目录]
        数据目录默认 = 本脚本同级 data/；
        图输出到 数据目录/../analysis_figs/（可用图目录固定为该位置）。

数据格式（见 docs/E5_FADF_设计说明.md §9）：
    CCES Export，每行一个字节（十六进制 0xXX 或十进制），小端。
    float 4 字节小端，int16 2 字节小端，uint32 4 字节小端。

变量：
    gE5Hhat              float[1024]   辨识出的时域冲激响应
    gE5HtrueT            float[1024]   未知系统真值
    gE5BenchMSE          float[1500]   5 变体 x 300 块学习曲线（dB）
    gE5RtMSE             float[512]    实时学习曲线（dB）
    gE5Timing            float[16]     cycles / 失配 / 模式
    gE5Debug             uint32[16]    阶段计数 / AdcCount / 采集计数
    gRawCapture          int16[12000]  250 ms 内部 x(n)
    gProcessedCapture    int16[12000]  250 ms 内部 e(n)
    gProcessed1024Capture int16[12000] 250 ms 内部 d(n)

平台：48 kHz，SC589 Core1 450 MHz；1 块 = 256 点 = 5.333 ms；
DMA ping-pong 单次回调期限约 2.66 ms。

只依赖 numpy + PIL（不 import matplotlib）。
"""
import math
import os
import re
import sys
import traceback

import numpy as np
from PIL import Image, ImageDraw, ImageFont

# ---------------------------------------------------------------------------
# 路径与常量
# ---------------------------------------------------------------------------
HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else os.path.join(HERE, "data")
# 每个数据目录单独一个图目录（data_part -> analysis_figs/data_part），
# 这样基础模式/进阶模式两次采集的图不会互相覆盖。
FIG = os.path.normpath(os.path.join(os.path.dirname(DATA), "analysis_figs", os.path.basename(DATA)))
os.makedirs(FIG, exist_ok=True)

FS = 48000.0
CLK = 450e6
BLOCK = 256
BLOCK_MS = 1000.0 * BLOCK / FS          # 5.3333 ms
RT_BUDGET_MS = BLOCK_MS                 # 实时块预算 5.333 ms
DMA_MS = BLOCK_MS / 2.0                 # 2.6667 ms 单次回调 DMA 期限
NF_SPEC = 4096                          # Welch FFT 点数
N_VAR, N_BENCH = 5, 300

VARIANT_NAMES = [
    "标准256-约束",
    "标准256-不约束",
    "分区1024-约束",
    "分区1024-不约束",
    "标准1024(N=2048)-约束",
]
VARIANT_COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]
VARIANT_POINTS = [256, 256, 256, 256, 1024]   # 每个块覆盖的新样点数（hop）

FONT_REG = r"C:\Windows\Fonts\msyh.ttc"
FONT_BOLD = r"C:\Windows\Fonts\msyhbd.ttc"
S = 2          # 绘图超采样倍数

MISSING = []   # 记录跳过的图/数据，写进 summary


# ---------------------------------------------------------------------------
# 数据读取（沿用 E4 脚本风格：每行一个字节）
# ---------------------------------------------------------------------------
def read_bytes(path):
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        txt = f.read()
    toks = re.findall(r"0[xX][0-9a-fA-F]+|[-+]?\d+", txt)
    b = bytearray()
    for s in toks:
        v = int(s, 16) if s.lower().startswith("0x") else int(s)
        b.append(v & 0xFF)
    return bytes(b)


def read_mem(path, typ, count):
    b = read_bytes(path)
    w = {"int16": 2, "single": 4, "uint32": 4}[typ]
    b = b[: len(b) // w * w]
    dt = {"int16": "<i2", "single": "<f4", "uint32": "<u4"}[typ]
    return np.frombuffer(b, dtype=dt).astype(np.float64)[:count]


def load(name, typ, count):
    p = os.path.join(DATA, name + ".dat")
    if not os.path.exists(p):
        print(f"[跳过] 缺少 {name}.dat")
        MISSING.append(f"{name}.dat 缺失")
        return None
    try:
        v = read_mem(p, typ, count)
    except Exception as e:                                    # noqa: BLE001
        print(f"[跳过] 读取 {name}.dat 失败: {e}")
        MISSING.append(f"{name}.dat 读取失败")
        return None
    if len(v) < count:
        print(f"[警告] {name}.dat 只有 {len(v)} 点（应为 {count}），按实际长度使用")
        MISSING.append(f"{name}.dat 长度不足（{len(v)}/{count}）")
    print(f"[读取] {name:<24s} {len(v):6d} 点")
    return v


# ---------------------------------------------------------------------------
# 绘图工具（PIL，白底，中文，网格）
# ---------------------------------------------------------------------------
def get_font(size, bold=False):
    return ImageFont.truetype(FONT_BOLD if bold else FONT_REG, size * S)


def canvas(w, h, title, subtitle=None):
    img = Image.new("RGB", (w * S, h * S), "white")
    d = ImageDraw.Draw(img)
    d.text((w * S // 2, 23 * S), title, font=get_font(23, True),
           fill="#111111", anchor="mm")
    if subtitle:
        d.text((w * S // 2, 50 * S), subtitle, font=get_font(13),
               fill="#444444", anchor="mm")
    return img, d


def panel(d, box, xlim, ylim, xticks, yticks, title, xlabel, ylabel,
          xtick_fmt="{:.0f}", ytick_fmt="{:.0f}"):
    x0, y0, x1, y1 = [v * S for v in box]
    d.rectangle([x0, y0, x1, y1], fill="white", outline="#333333", width=S)
    if title:
        d.text(((x0 + x1) // 2, y0 - 24 * S), title, font=get_font(17, True),
               fill="#111111", anchor="mm")

    def X(x):
        return x0 + (x - xlim[0]) / (xlim[1] - xlim[0]) * (x1 - x0)

    def Y(y):
        return y1 - (y - ylim[0]) / (ylim[1] - ylim[0]) * (y1 - y0)

    for xt in xticks:
        if xt < xlim[0] or xt > xlim[1]:
            continue
        xx = X(xt)
        d.line([xx, y0, xx, y1], fill="#dddddd", width=S)
        d.text((xx, y1 + 8 * S), xtick_fmt.format(xt), font=get_font(13),
               fill="#222222", anchor="ma")
    for yt in yticks:
        if yt < ylim[0] or yt > ylim[1]:
            continue
        yy = Y(yt)
        d.line([x0, yy, x1, yy], fill="#dddddd", width=S)
        d.text((x0 - 8 * S, yy), ytick_fmt.format(yt), font=get_font(13),
               fill="#222222", anchor="rm")
    d.text(((x0 + x1) // 2, y1 + 42 * S), xlabel, font=get_font(14),
           fill="#111111", anchor="mm")
    d.text((x0, y0 - 8 * S), ylabel, font=get_font(14), fill="#111111", anchor="lb")
    return X, Y


def line(d, xs, ys, X, Y, color, width=2):
    pts = [(X(x), Y(y)) for x, y in zip(xs, ys) if np.isfinite(y)]
    if len(pts) > 1:
        d.line(pts, fill=color, width=width * S, joint="curve")


def legend(d, box, items, font_size=13):
    x0, y0, x1, y1 = [v * S for v in box]
    pad = 8 * S
    line_h = 23 * S
    d.rectangle([x0, y0, x1, y1], fill="white", outline="#999999", width=S)
    for i, (label, color) in enumerate(items):
        yy = y0 + pad + i * line_h + line_h // 2
        d.line([x0 + pad, yy, x0 + pad + 34 * S, yy], fill=color, width=3 * S)
        d.text((x0 + pad + 44 * S, yy), label, font=get_font(font_size),
               fill="#111111", anchor="lm")


def vline(d, x, X, Y, y0, y1, color="#666666", dash=6):
    xx = X(x)
    yy0, yy1 = Y(y0), Y(y1)
    step = dash * S
    y = yy0
    while y < yy1:
        d.line([xx, y, xx, min(y + step, yy1)], fill=color, width=S)
        y += 2 * step


def hline(d, y, X, Y, x0, x1, color="#666666", dash=6):
    yy = Y(y)
    xx0, xx1 = X(x0), X(x1)
    step = dash * S
    x = xx0
    while x < xx1:
        d.line([x, yy, min(x + step, xx1), yy], fill=color, width=S)
        x += 2 * step


def textbox(d, xy, lines, fs=14, color="#111111"):
    f = get_font(fs)
    pad = 8 * S
    lh = 22 * S
    w = max(f.getbbox(s)[2] for s in lines) + 2 * pad
    x, y = xy[0] * S, xy[1] * S
    d.rectangle([x, y, x + w, y + len(lines) * lh + pad], fill="white",
                outline="#888888", width=S)
    for i, s in enumerate(lines):
        d.text((x + pad, y + pad // 2 + i * lh), s, font=f, fill=color)


def save(img, name):
    p = os.path.join(FIG, name)
    img.save(p)
    print(f"[图] {p}")


# ---------------------------------------------------------------------------
# 信号处理工具
# ---------------------------------------------------------------------------
def welch_psd(x, nfft=NF_SPEC, fs=FS):
    x = np.asarray(x, dtype=float)
    x = x - x.mean()
    step = nfft // 2
    win = 0.5 - 0.5 * np.cos(2 * np.pi * np.arange(nfft) / (nfft - 1))
    P = np.zeros(nfft // 2 + 1)
    used = 0
    for s in range(0, len(x) - nfft + 1, step):
        X = np.fft.rfft(x[s:s + nfft] * win)
        P += np.abs(X) ** 2
        used += 1
    P /= max(1, used)
    P[1:-1] *= 2
    P /= fs * np.sum(win ** 2)
    f = np.arange(nfft // 2 + 1) * fs / nfft
    return f, P


def welch_cross(x, y, nfft=NF_SPEC, fs=FS):
    x = np.asarray(x, dtype=float) - np.mean(x)
    y = np.asarray(y, dtype=float) - np.mean(y)
    step = nfft // 2
    win = 0.5 - 0.5 * np.cos(2 * np.pi * np.arange(nfft) / (nfft - 1))
    Pxx = np.zeros(nfft // 2 + 1)
    Pxy = np.zeros(nfft // 2 + 1, dtype=complex)
    used = 0
    for s in range(0, len(x) - nfft + 1, step):
        X = np.fft.rfft(x[s:s + nfft] * win)
        Y = np.fft.rfft(y[s:s + nfft] * win)
        Pxx += np.abs(X) ** 2
        Pxy += np.conj(X) * Y
        used += 1
    Pxx /= max(1, used)
    Pxy /= max(1, used)
    f = np.arange(nfft // 2 + 1) * fs / nfft
    return f, Pxy, Pxx


def resp_db(h, nfft):
    H = np.fft.rfft(np.asarray(h, dtype=float), nfft)
    f = np.arange(nfft // 2 + 1) * FS / nfft
    return f, 20 * np.log10(np.abs(H) + 1e-12)


def resp_complex_on(f, h, nfft=16384):
    H = np.fft.rfft(np.asarray(h, dtype=float), nfft)
    fh = np.arange(nfft // 2 + 1) * FS / nfft
    return np.interp(f, fh, H.real) + 1j * np.interp(f, fh, H.imag)


def valid_len(v):
    """学习曲线有效长度：遇到第一个 0 即认为之后无效（变体 4 只有前 150 块）。"""
    v = np.asarray(v, dtype=float)
    z = np.where(v == 0.0)[0]
    return int(z[0]) if len(z) else len(v)


def conv_block(v, thresh=-20.0, win=10):
    """10 块滑动平均首次 <= thresh 的块序号（0 基）；达不到返回 None。"""
    n = valid_len(v)
    vv = np.asarray(v, dtype=float)[:n]
    if n < win:
        return None
    sm = np.convolve(vv, np.ones(win) / win, "valid")
    idx = np.where(sm <= thresh)[0]
    return int(idx[0]) + win - 1 if len(idx) else None


def align_pair(hhat, htrue, maxlag=32):
    """在 +-maxlag 内找出使 hhat 与 htrue 最对齐的整数延迟。"""
    n = min(len(hhat), len(htrue))
    h = np.asarray(hhat[:n], dtype=float)
    t = np.asarray(htrue[:n], dtype=float)
    best = (0, float("inf"))
    for lag in range(-maxlag, maxlag + 1):
        a = np.zeros(n)
        if lag >= 0:
            a[lag:] = h[:n - lag]
        else:
            a[:n + lag] = h[-lag:]
        v = float(np.sum((a - t) ** 2))
        if v < best[1]:
            best = (lag, v)
    lag = best[0]
    a = np.zeros(n)
    if lag >= 0:
        a[lag:] = h[:n - lag]
    else:
        a[:n + lag] = h[-lag:]
    return lag, a, t


def ir_metrics(hhat, htrue, n):
    """前 n 点的失配 / 相对误差（先做整数延迟对齐）。"""
    n = int(min(n, len(hhat), len(htrue)))
    lag, a, t = align_pair(hhat[:n], htrue[:n])
    diff = a - t
    ref = float(np.sum(t * t))
    mis = 10 * np.log10(max(float(np.sum(diff * diff)), 1e-30) / max(ref, 1e-30))
    peak = max(float(np.max(np.abs(t))), 1e-30)
    max_rel = float(np.max(np.abs(diff))) / peak * 100.0
    mask = np.abs(t) > 0.01 * peak
    max_pt = (float(np.max(np.abs(diff[mask]) / np.abs(t[mask]))) * 100.0
              if np.any(mask) else float("nan"))
    return dict(n=n, lag=lag, mis=mis, max_rel=max_rel, max_pt=max_pt,
                peak=peak, hhat=a, htrue=t)


def is_sys_b(htrue):
    tail = np.max(np.abs(htrue[256:])) if len(htrue) > 256 else 0.0
    return tail > 1e-3 * max(float(np.max(np.abs(htrue))), 1e-9)


def tone_db(f, Hdb, f0):
    i = int(np.argmin(np.abs(f - f0)))
    return float(Hdb[i]), float(f[i])


# ---------------------------------------------------------------------------
# 读数据
# ---------------------------------------------------------------------------
print(f"数据目录: {DATA}")
print(f"图目录  : {FIG}")
Hhat = load("gE5Hhat", "single", 1024)
Htrue = load("gE5HtrueT", "single", 1024)
Timing = load("gE5Timing", "single", 16)
Debug = load("gE5Debug", "uint32", 16)
Bench = load("gE5BenchMSE", "single", N_VAR * N_BENCH)
Rt = load("gE5RtMSE", "single", 512)
raw = load("gRawCapture", "int16", 12000)
e_cap = load("gProcessedCapture", "int16", 12000)
d_cap = load("gProcessed1024Capture", "int16", 12000)

# 实时模式：gE5Timing[11] = 256 基础 / 1024 进阶；[12] = 约束标志
RT_MODE = None
CONSTRAINT = None
if Timing is not None and len(Timing) >= 13 and 100.0 <= Timing[11] <= 8192.0:
    RT_MODE = int(round(float(Timing[11])))
    CONSTRAINT = int(round(float(Timing[12]))) == 1
MODE_LABEL = {256: "基础（标准 256 点）",
              1024: "进阶（分区 1024 点）"}.get(RT_MODE, "未知")
CONSTRAINT_LABEL = ("约束" if CONSTRAINT else "不约束") if CONSTRAINT is not None else "未知"

# ---------------------------------------------------------------------------
# 基础分析（供各图与 summary 共用）
# ---------------------------------------------------------------------------
bench = None
if Bench is not None and len(Bench) >= N_VAR * N_BENCH:
    bench = Bench[:N_VAR * N_BENCH].reshape(N_VAR, N_BENCH)
bench_n = [valid_len(bench[i]) if bench is not None else 0 for i in range(N_VAR)]
bench_last = [float(bench[i][bench_n[i] - 1]) if bench_n[i] else float("nan")
              for i in range(N_VAR)]
bench_conv = [conv_block(bench[i]) if bench is not None else None for i in range(N_VAR)]
bench_min = [float(np.min(bench[i][:bench_n[i]])) if bench_n[i] else float("nan")
             for i in range(N_VAR)]

rt_n = valid_len(Rt) if Rt is not None else 0
rt_conv = conv_block(Rt) if rt_n >= 10 else None
rt_last = float(Rt[rt_n - 1]) if rt_n else float("nan")

SYS_B = is_sys_b(Htrue) if Htrue is not None else None
IR = {}
if Hhat is not None and Htrue is not None:
    n_tot = min(len(Hhat), len(Htrue))
    IR["all"] = ir_metrics(Hhat, Htrue, n_tot)
    IR["256"] = ir_metrics(Hhat, Htrue, min(256, n_tot))
    if SYS_B:
        IR["512"] = ir_metrics(Hhat, Htrue, min(512, n_tot))
    IR["sys"] = "B" if SYS_B else "A"

CAP = {}
if raw is not None and e_cap is not None and d_cap is not None:
    xf = np.asarray(raw, float) / 32768.0
    df = np.asarray(d_cap, float) / 32768.0
    ef = np.asarray(e_cap, float) / 32768.0
    rms = lambda v: float(np.sqrt(np.mean(v * v)))             # noqa: E731
    CAP = dict(rms_x=rms(xf), rms_d=rms(df), rms_e=rms(ef),
               peak_d=float(np.max(np.abs(df))),
               peak_e=float(np.max(np.abs(ef))),
               ratio_db=20 * np.log10(max(rms(ef), 1e-15) / max(rms(df), 1e-15)),
               ratio_pct=100.0 * rms(ef) / max(rms(df), 1e-15),
               gain=max(float(np.max(np.abs(df))), 1e-9) /
                    max(float(np.max(np.abs(ef))), 1e-15))

TIM = {}
if Timing is not None and len(Timing) >= 5:
    TIM["cyc"] = np.array([float(Timing[i]) for i in range(5)])
    TIM["rt_avg"] = float(Timing[8]) if len(Timing) > 8 else 0.0
    TIM["mu"] = float(Timing[13]) if len(Timing) > 13 else float("nan")
    TIM["rt_first"] = float(Timing[14]) if len(Timing) > 14 else 0.0
    TIM["rt_max"] = float(Timing[15]) if len(Timing) > 15 else 0.0
    # 失配：变体 0 -> Timing[5]，变体 2 -> [6]，变体 4 -> [7]
    TIM["mis"] = {0: float(Timing[5]) if len(Timing) > 5 else float("nan"),
                  2: float(Timing[6]) if len(Timing) > 6 else float("nan"),
                  4: float(Timing[7]) if len(Timing) > 7 else float("nan")}

DBG = {}
if Debug is not None:
    for i, key in enumerate(["enter", "blocks", "s0", "s1", "s2", "s3", "adc",
                             "dac", "err", "fifo_rd", "cap_active", "cap_frames",
                             "w_energy", "e_energy", "last_cyc", "bench_done"]):
        if i < len(Debug):
            DBG[key] = float(Debug[i])


# ---------------------------------------------------------------------------
# 图 1：学习曲线
# ---------------------------------------------------------------------------
def fig_learning():
    if bench is None:
        print("[跳过] e5_learning_curves.png：缺少 gE5BenchMSE.dat 或长度不足")
        return
    img, d = canvas(1500, 900,
                    "E5 FDAF 启动 benchmark：5 个变体的学习曲线")
    d.text((1500 * S // 2, 52 * S),
           "误差 MSE = 10·log10( Σe² / Σd² )，每块一个点；变体 4 仅前 150 块有效",
           font=get_font(13), fill="#444444", anchor="mm")
    ns = bench_n
    data = [bench[i][:ns[i]] for i in range(N_VAR)]
    flats = [v for v in data if len(v)]
    if not flats:
        print("[跳过] e5_learning_curves.png：学习曲线全 0")
        return
    lo = min(float(np.min(v)) for v in flats)
    hi = max(float(np.max(v)) for v in flats)
    lo = max(-70.0, math.floor((lo - 3) / 10) * 10)
    hi = min(10.0, math.ceil((hi + 3) / 10) * 10)
    if hi - lo < 20:
        hi = lo + 20
    yt = np.arange(lo, hi + 1e-9, 10.0)
    X, Y = panel(d, (110, 100, 1460, 800), (0, N_BENCH), (lo, hi),
                 np.arange(0, 301, 50), yt, "",
                 "块序号 m（1 块 = 256 点 = 5.33 ms）", "误差 MSE / dB")
    hline(d, -20, X, Y, 0, N_BENCH, "#bbbbbb")
    d.text((X(N_BENCH) - 5 * S, Y(-20) + 4 * S), "-20 dB",
           font=get_font(12), fill="#666666", anchor="rt")
    for i in range(N_VAR):
        if ns[i] < 2:
            continue
        line(d, np.arange(ns[i]), data[i], X, Y, VARIANT_COLORS[i], 2)
    items = []
    for i in range(N_VAR):
        nm = VARIANT_NAMES[i] if ns[i] >= N_BENCH else f"{VARIANT_NAMES[i]}（前 {ns[i]} 块）"
        items.append((nm, VARIANT_COLORS[i]))
    legend(d, (1010, 110, 1450, 280), items, font_size=13)
    save(img, "e5_learning_curves.png")


# ---------------------------------------------------------------------------
# 图 2：256 点冲激响应
# ---------------------------------------------------------------------------
def fig_ir256():
    if Hhat is None or Htrue is None:
        print("[跳过] e5_ir_256.png：缺少 gE5Hhat/gE5HtrueT")
        return
    n = min(256, len(Hhat), len(Htrue))
    m = ir_metrics(Hhat, Htrue, n)
    sys_name = ("系统 B 真值（前 256 点）" if SYS_B
                else "系统 A 真值（256 点）")
    img, d = canvas(1500, 800,
                    f"E5 系统辨识：冲激响应真值 vs 辨识 hhat（前 {n} 点）")
    d.text((1500 * S // 2, 52 * S),
           f"未知系统：{sys_name}；1 tap = 1/48 kHz = 20.83 µs",
           font=get_font(13), fill="#444444", anchor="mm")
    ymax = max(float(np.max(np.abs(m["htrue"]))),
               float(np.max(np.abs(m["hhat"]))), 1e-9) * 1.18
    X, Y = panel(d, (120, 115, 1460, 660), (0, max(n - 1, 1)), (-ymax, ymax),
                 np.arange(0, n + 1, 32), np.linspace(-ymax, ymax, 9), "",
                 "抽头序号 n", "冲激响应 h(n)", ytick_fmt="{:.3f}")
    line(d, np.arange(n), m["hhat"], X, Y, "#d62728", 2)
    line(d, np.arange(n), m["htrue"], X, Y, "#111111", 4)
    legend(d, (1130, 130, 1445, 205),
           [("真值 h_true", "#111111"), ("辨识 hhat", "#d62728")])
    textbox(d, (145, 140), [
        f"失配 = {m['mis']:.1f} dB",
        f"峰值相对误差 = {m['max_rel']:.2f} %",
        f"峰值处逐点最大相对误差 = {m['max_pt']:.1f} %",
        f"整数延迟对齐 lag = {m['lag']} 点",
    ])
    save(img, "e5_ir_256.png")


# ---------------------------------------------------------------------------
# 图 3：1024 点冲激响应
# ---------------------------------------------------------------------------
def fig_ir1024():
    if Hhat is None or Htrue is None:
        print("[跳过] e5_ir_1024.png：缺少 gE5Hhat/gE5HtrueT")
        return
    n_tot = min(len(Hhat), len(Htrue))
    if n_tot < 300:
        print("[跳过] e5_ir_1024.png：数据长度不足")
        return
    sys_name = "系统 B（1024 点混响）" if SYS_B else "系统 A（256 点，后段为 0）"
    img, d = canvas(1500, 950,
                    f"E5 系统辨识：冲激响应真值 vs 辨识 hhat（{sys_name}）")
    for k, (n, box, tag) in enumerate([
            (min(512, n_tot), (115, 110, 1460, 455), "前 512 点"),
            (n_tot, (115, 600, 1460, 930), f"全 {n_tot} 点")]):
        m = ir_metrics(Hhat, Htrue, n)
        ymax = max(float(np.max(np.abs(m["htrue"]))),
                   float(np.max(np.abs(m["hhat"]))), 1e-9) * 1.18
        X, Y = panel(d, box, (0, max(n - 1, 1)), (-ymax, ymax),
                     np.arange(0, n + 1, max(32, n // 16)),
                     np.linspace(-ymax, ymax, 9),
                     f"{tag}（失配 {m['mis']:.1f} dB，lag = {m['lag']} 点）",
                     "抽头序号 n", "h(n)", ytick_fmt="{:.3f}")
        line(d, np.arange(n), m["hhat"], X, Y, "#d62728", 2)
        line(d, np.arange(n), m["htrue"], X, Y, "#111111", 4)
        legend(d, (1130, box[1] + 15, 1445, box[1] + 90),
               [("真值 h_true", "#111111"), ("辨识 hhat", "#d62728")])
        textbox(d, (box[0] + 25, box[1] + 20), [
            f"峰值相对误差 = {m['max_rel']:.2f} %",
            f"峰值处逐点最大相对误差 = {m['max_pt']:.1f} %",
        ], fs=13)
    save(img, "e5_ir_1024.png")


# ---------------------------------------------------------------------------
# 图 4：幅频响应
# ---------------------------------------------------------------------------
def fig_freq():
    if Hhat is None or Htrue is None:
        print("[跳过] e5_freq_response.png：缺少 gE5Hhat/gE5HtrueT")
        return
    n_tot = min(len(Hhat), len(Htrue))
    if SYS_B:
        segs = [(512, min(256, n_tot), "前 256 点（512 点 FFT，对照系统 A 长度）"),
                (1024, n_tot, "系统 B：全 1024 点（1024 点 FFT）")]
    else:
        segs = [(512, min(256, n_tot), "系统 A：256 点（512 点 FFT）")]
    h = 900 if len(segs) > 1 else 600
    img, d = canvas(1500, h, "E5 幅频响应：未知系统真值 vs 辨识结果")
    boxes = [(110, 110, 1460, 480), (110, 600, 1460, 850)]
    freq_ann = {}
    for (nfft, n, tag), box in zip(segs, boxes):
        m = ir_metrics(Hhat, Htrue, n)
        f, Ht = resp_db(m["htrue"], nfft)
        _, Hh = resp_db(m["hhat"], nfft)
        keep = f <= 24000 + 1e-9
        fk, tk, hk = f[keep], Ht[keep], Hh[keep]
        lo = max(-80.0, math.floor(min(float(np.min(tk)), float(np.min(hk))) / 10) * 10 - 5)
        hi = math.ceil(max(float(np.max(tk)), float(np.max(hk))) / 10) * 10 + 5
        if hi - lo < 20:
            hi = lo + 20
        X, Y = panel(d, box, (0, 24000), (lo, hi),
                     np.arange(0, 24001, 4000), np.arange(lo, hi + 1e-9, 10),
                     tag, "频率 / Hz", "幅度 / dB")
        line(d, fk, tk, X, Y, "#111111", 2)
        line(d, fk, hk, X, Y, "#d62728", 2)
        legend(d, (1150, box[1] + 15, 1445, box[1] + 90),
               [("真值 |H_true(f)|", "#111111"), ("辨识 |Hhat(f)|", "#d62728")])
        ann = []
        for f0, f0m in [(6000, "6 kHz"), (12000, "12 kHz")]:
            tdb, ff = tone_db(fk, tk, f0)
            hdb, _ = tone_db(fk, hk, f0)
            vline(d, ff, X, Y, lo, hi, "#999999", 4)
            d.text((X(ff) + 4 * S, Y(lo) + 8 * S), f0m, font=get_font(12),
                   fill="#444444", anchor="lb")
            ann.append(f"{f0m}：真值 {tdb:.1f} dB / 辨识 {hdb:.1f} dB")
        textbox(d, (box[0] + 20, box[1] + 15), ann, fs=13)
        freq_ann[tag] = ann
    if not SYS_B:
        d.text((1500 * S // 2, 560 * S),
               "本导出数据为 256 点系统 A：gE5HtrueT 后 768 点为 0，故不画系统 B 子图",
               font=get_font(13), fill="#666666", anchor="mm")
    IR["freq_ann"] = freq_ann
    save(img, "e5_freq_response.png")


# ---------------------------------------------------------------------------
# 图 5：实时学习曲线
# ---------------------------------------------------------------------------
def fig_rt():
    if Rt is None:
        print("[跳过] e5_rt_learning.png：缺少 gE5RtMSE.dat")
        return
    n = valid_len(Rt)
    if n < 5:
        print("[跳过] e5_rt_learning.png：gE5RtMSE 无有效数据（全 0）")
        return
    y = np.asarray(Rt[:n], float)
    x = np.arange(n) * BLOCK_MS
    img, d = canvas(1500, 820,
                    f"E5 实时学习曲线（{MODE_LABEL}，{CONSTRAINT_LABEL}）")
    d.text((1500 * S // 2, 52 * S),
           f"每块一个 dB 点；1 块 = 256 点 = {BLOCK_MS:.3f} ms；有效 {n} 块，"
           f"共 {x[-1]:.0f} ms",
           font=get_font(13), fill="#444444", anchor="mm")
    lo = max(-70.0, math.floor((float(np.min(y)) - 3) / 10) * 10)
    hi = min(10.0, math.ceil((float(np.max(y)) + 3) / 10) * 10)
    if hi - lo < 20:
        hi = lo + 20
    X, Y = panel(d, (120, 110, 1460, 690), (0, max(x[-1], 1.0)), (lo, hi),
                 np.linspace(0, x[-1], 7) if x[-1] > 0 else [0, 1],
                 np.arange(lo, hi + 1e-9, 10), "",
                 "时间 / ms（块序号 × 5.33 ms）", "误差 MSE / dB")
    hline(d, -20, X, Y, 0, x[-1], "#bbbbbb")
    d.text((X(x[-1]) - 5 * S, Y(-20) + 4 * S), "-20 dB",
           font=get_font(12), fill="#666666", anchor="rt")
    line(d, x, y, X, Y, "#1f77b4", 2)
    conv_ms = (rt_conv * BLOCK_MS) if rt_conv is not None else None
    textbox(d, (150, 140), [
        f"有效块数 = {n}（{x[-1]:.0f} ms）",
        f"末块 MSE = {rt_last:.1f} dB",
        ("收敛到 -20 dB：块 " + f"{rt_conv}（{conv_ms:.0f} ms）"
         if rt_conv is not None else "收敛到 -20 dB：未达到"),
    ])
    save(img, "e5_rt_learning.png")


# ---------------------------------------------------------------------------
# 图 6：250 ms 采集时域波形
# ---------------------------------------------------------------------------
def fig_capture_time():
    if raw is None or e_cap is None or d_cap is None:
        print("[跳过] e5_capture_time.png：缺少采集数组")
        return
    xf = np.asarray(raw, float) / 32768.0
    df = np.asarray(d_cap, float) / 32768.0
    ef = np.asarray(e_cap, float) / 32768.0
    n = min(len(xf), len(df), len(ef))
    xf, df, ef = xf[:n], df[:n], ef[:n]
    gain = max(float(np.max(np.abs(df))), 1e-9) / max(float(np.max(np.abs(ef))), 1e-15)
    gain = max(gain, 1.0)
    img, d = canvas(1500, 950,
                    "E5 250 ms 内部采集：激励 x(n) / 期望 d(n) / 误差 e(n)")
    d.text((1500 * S // 2, 52 * S),
           "采集为 DSP 内部信号（不经 DAC/ADC 模拟链路）；e 在图中按比例放大",
           font=get_font(13), fill="#444444", anchor="mm")
    # 上：前 20 ms
    t20 = 0.020
    n20 = int(t20 * FS)
    n20 = min(n20, n)
    tt = np.arange(n20) / FS * 1000.0
    ymax = max(float(np.max(np.abs(xf[:n20]))), float(np.max(np.abs(df[:n20]))),
               float(np.max(np.abs(ef[:n20]))) * gain, 1e-9) * 1.15
    X, Y = panel(d, (115, 110, 1460, 470), (0, t20 * 1000), (-ymax, ymax),
                 np.arange(0, 21, 2), np.linspace(-ymax, ymax, 7),
                 "前 20 ms 细节", "时间 / ms", "幅度（归一化 / 32768）")
    line(d, tt, xf[:n20], X, Y, "#111111", 1)
    line(d, tt, df[:n20], X, Y, "#1f77b4", 1)
    line(d, tt, ef[:n20] * gain, X, Y, "#d62728", 1)
    legend(d, (1080, 130, 1440, 235),
           [("x(n) 激励", "#111111"), ("d(n) 期望", "#1f77b4"),
            (f"e(n) ×{gain:.0f} 误差", "#d62728")], font_size=13)
    # 下：整段 250 ms
    tfull = n / FS * 1000.0
    step = max(1, n // 6000)
    xs = np.arange(0, n, step) / FS * 1000.0
    ymax2 = max(float(np.max(np.abs(df))), float(np.max(np.abs(ef))) * gain, 1e-9) * 1.12
    X, Y = panel(d, (115, 620, 1460, 920), (0, tfull), (-ymax2, ymax2),
                 np.arange(0, tfull + 1e-9, 50), np.linspace(-ymax2, ymax2, 7),
                 f"整段 {tfull:.0f} ms：d(n) 与 e(n)×{gain:.0f}",
                 "时间 / ms", "幅度（归一化 / 32768）")
    line(d, xs, df[::step], X, Y, "#1f77b4", 1)
    line(d, xs, ef[::step] * gain, X, Y, "#d62728", 1)
    legend(d, (1080, 640, 1440, 715),
           [("d(n) 期望", "#1f77b4"), (f"e(n) ×{gain:.0f}", "#d62728")],
           font_size=13)
    textbox(d, (150, 640), [
        f"rms(e)/rms(d) = {CAP.get('ratio_pct', float('nan')):.3f} % "
        f"（{CAP.get('ratio_db', float('nan')):.1f} dB）",
        f"rms(d) = {CAP.get('rms_d', float('nan')):.4f}（int16 归一化）",
    ], fs=13)
    save(img, "e5_capture_time.png")


# ---------------------------------------------------------------------------
# 图 7：采集频谱（x / d Welch + 实测系统频响）
# ---------------------------------------------------------------------------
def fig_capture_spectrum():
    if raw is None or d_cap is None:
        print("[跳过] e5_capture_spectrum.png：缺少 gRawCapture/gProcessed1024Capture")
        return
    f, Px = welch_psd(raw)
    _, Pd = welch_psd(d_cap)
    ref = max(float(np.max(Px)), 1e-30)
    dbx = 10 * np.log10(Px / ref + 1e-30)
    dbd = 10 * np.log10(Pd / ref + 1e-30)
    keep = f <= 24000 + 1e-9
    fk, bx, bd = f[keep], dbx[keep], dbd[keep]
    img, d = canvas(1500, 950,
                    "E5 250 ms 采集频谱：x(n) / d(n) Welch 谱与实测系统频响")
    d.text((1500 * S // 2, 52 * S),
           "Welch：4096 点、Hann 窗、50% 重叠；谱相对 x 峰值归一化",
           font=get_font(13), fill="#444444", anchor="mm")
    lo = max(-100.0, math.floor((min(float(np.min(bx)), float(np.min(bd))) - 5) / 10) * 10)
    hi = 8.0
    X, Y = panel(d, (115, 110, 1460, 480), (0, 24000), (lo, hi),
                 np.arange(0, 24001, 4000), np.arange(lo, hi + 1e-9, 20),
                 "Welch 功率谱（0–24 kHz）", "频率 / Hz", "相对功率 / dB")
    line(d, fk, bx, X, Y, "#111111", 1)
    line(d, fk, bd, X, Y, "#1f77b4", 1)
    legend(d, (1080, 130, 1445, 205),
           [("x(n) 激励谱", "#111111"), ("d(n) 期望谱", "#1f77b4")])
    # 峰值标注（d 谱，间隔 >= 500 Hz，取前 3）
    Pk = Pd[keep]
    idx = np.where((Pk[1:-1] > Pk[:-2]) & (Pk[1:-1] > Pk[2:]))[0] + 1
    idx = idx[fk[idx] >= 200]
    order = idx[np.argsort(Pk[idx])[::-1]]
    peaks = []
    for i in order:
        if all(abs(fk[i] - fp) > 500 for fp, _ in peaks):
            peaks.append((float(fk[i]), float(10 * np.log10(Pk[i] / ref + 1e-30))))
        if len(peaks) >= 3:
            break
    lines = []
    for fp, dbv in peaks:
        vline(d, fp, X, Y, lo, hi, "#cccccc", 4)
        d.text((X(fp) + 4 * S, Y(hi) - 7 * S), f"{fp:.0f} Hz",
               font=get_font(11), fill="#555555", anchor="lm")
        lines.append(f"峰值 {fp:6.0f} Hz：{dbv:6.1f} dB")
    if lines:
        textbox(d, (150, 135), lines, fs=13)
    # 下：实测频响 Pxy/Pxx vs 真值 / 辨识
    f2, Pxy, Pxx = welch_cross(raw, d_cap)
    Hm = np.abs(Pxy / np.maximum(Pxx, 1e-30))
    Hmdb = 20 * np.log10(Hm + 1e-12)
    keep2 = f2 <= 24000 + 1e-9
    f2k = f2[keep2]
    Hm2 = Hmdb[keep2]
    curves = [("实测 d/x（Welch 交谱）", Hm2, "#ff7f0e")]
    if Htrue is not None:
        Ht = 20 * np.log10(np.abs(resp_complex_on(f2k, Htrue)) + 1e-12)
        curves.append(("真值 H_true(f)", Ht, "#111111"))
    if Hhat is not None:
        Hh = 20 * np.log10(np.abs(resp_complex_on(f2k, Hhat)) + 1e-12)
        curves.append(("辨识 Hhat(f)", Hh, "#d62728"))
    allv = np.concatenate([c[1] for c in curves])
    lo2 = max(-80.0, math.floor((float(np.min(allv)) - 5) / 10) * 10)
    hi2 = math.ceil((float(np.max(allv)) + 5) / 10) * 10
    if hi2 - lo2 < 20:
        hi2 = lo2 + 20
    X, Y = panel(d, (115, 620, 1460, 915), (0, 24000), (lo2, hi2),
                 np.arange(0, 24001, 4000), np.arange(lo2, hi2 + 1e-9, 10),
                 "实测系统频响 |D(f)/X(f)| vs 真值（0–24 kHz）",
                 "频率 / Hz", "幅度 / dB")
    for lab, cv, col in curves:
        line(d, f2k, cv, X, Y, col, 2)
    legend(d, (1030, 640, 1450, 645 + 25 * len(curves)),
           [(lab, col) for lab, _, col in curves], font_size=13)
    IR["capture_freq"] = [(lab, float(np.mean(cv[(f2k > 5500) & (f2k < 6500)])))
                          for lab, cv, _ in curves]
    save(img, "e5_capture_spectrum.png")


# ---------------------------------------------------------------------------
# 图 8：耗时柱状图
# ---------------------------------------------------------------------------
def fig_timing():
    if Timing is None or len(Timing) < 5:
        print("[跳过] e5_timing.png：缺少 gE5Timing.dat")
        return
    cyc = np.array([float(Timing[i]) for i in range(5)])
    if not np.any(cyc > 0):
        print("[跳过] e5_timing.png：gE5Timing[0..4] 全为 0")
        return
    img, d = canvas(1400, 1000,
                    "E5 运算耗时：5 个 benchmark 变体 cycles/块（SC589 450 MHz）")
    d.text((1400 * S // 2, 52 * S),
           f"红虚线 {RT_BUDGET_MS:.2f} ms = 实时块预算；"
           f"橙虚线 {DMA_MS:.2f} ms = DMA 单次回调期限",
           font=get_font(13), fill="#444444", anchor="mm")
    x0, y0, x1, y1 = 135 * S, 105 * S, 1330 * S, 560 * S
    ymax = max(float(np.max(cyc)) * 1.38, 2.65e6)
    d.rectangle([x0, y0, x1, y1], fill="white", outline="#333333", width=S)
    for k in range(6):
        c = ymax * k / 5
        yy = y1 - (y1 - y0) * k / 5
        d.line([x0, yy, x1, yy], fill="#dddddd", width=S)
        d.text((x0 - 8 * S, yy), f"{c / 1e6:.1f}M", font=get_font(12),
               fill="#222222", anchor="rm")
        d.text((x1 + 8 * S, yy), f"{c / CLK * 1000:.2f}", font=get_font(12),
               fill="#222222", anchor="lm")
    d.text((x0, y0 - 26 * S), "cycles / 块（左轴）", font=get_font(14),
           fill="#111111", anchor="lb")
    d.text((x1, y0 - 26 * S), "ms / 块（右轴）", font=get_font(14),
           fill="#111111", anchor="rb")
    d.text(((x0 + x1) // 2, y1 + 82 * S), "benchmark 变体（每块 256 点；变体 4 为 1024 点块）",
           font=get_font(13), fill="#111111", anchor="mm")
    # 柱
    slot = (x1 - x0) / 5.0
    bw = slot * 0.50
    for i in range(5):
        cx = x0 + slot * (i + 0.5)
        hh = (y1 - y0) * float(cyc[i]) / ymax
        d.rectangle([cx - bw / 2, y1 - hh, cx + bw / 2, y1],
                    fill=VARIANT_COLORS[i])
        d.text((cx, y1 - hh - 7 * S),
               f"{cyc[i] / 1e6:.2f}M / {cyc[i] / CLK * 1000:.2f} ms",
               font=get_font(12, True), fill="#111111", anchor="mb")
        d.text((cx, y1 + 10 * S), VARIANT_NAMES[i], font=get_font(12),
               fill="#111111", anchor="ma")
    # 预算线（画在柱子之后，避免被柱子盖住标签）
    for ms_v, col, lab in [(RT_BUDGET_MS, "#d62728", "5.33 ms 实时块预算"),
                           (DMA_MS, "#ff7f0e", "2.66 ms DMA 回调期限")]:
        c = ms_v / 1000.0 * CLK
        yy = y1 - (y1 - y0) * c / ymax
        xx = x0
        while xx < x1:
            d.line([xx, yy, min(xx + 12 * S, x1), yy], fill=col, width=2 * S)
            xx += 24 * S
        lab_txt = f"{lab}（{c / 1e6:.2f} M cycles）"
        tfont = get_font(12, True)
        tb = d.textbbox((0, 0), lab_txt, font=tfont)
        tw, th = tb[2] - tb[0], tb[3] - tb[1]
        d.rectangle([x1 - 6 * S - tw - 5 * S, yy - 6 * S - th - 4 * S,
                     x1 - 2 * S, yy - 4 * S], fill="white")
        d.text((x1 - 6 * S, yy - 6 * S), lab_txt,
               font=tfont, fill=col, anchor="rb")
    # 下：实时单阶段估算
    avg = TIM.get("rt_avg", 0.0)
    if avg > 0:
        if RT_MODE == 1024 and CONSTRAINT is not None and int(CONSTRAINT) == 0:
            w = np.array([1.0, 2.2, 0.5])      # 分区不约束：S2 只有频域乘加
        else:
            w = np.array([1.0, 2.0, 2.0])      # 变换数 S0/S1/S2 = 1/2/2
        stage = avg * w / w.sum()
        sx0, sy0, sx1, sy1 = 135 * S, 730 * S, 900 * S, 965 * S
        smax = max(float(np.max(stage)) * 1.45, 1.4e6)
        d.rectangle([sx0, sy0, sx1, sy1], fill="white", outline="#333333", width=S)
        for k in range(5):
            c = smax * k / 4
            yy = sy1 - (sy1 - sy0) * k / 4
            d.line([sx0, yy, sx1, yy], fill="#dddddd", width=S)
            d.text((sx0 - 8 * S, yy), f"{c / 1e6:.2f}M", font=get_font(12),
                   fill="#222222", anchor="rm")
        c = DMA_MS / 1000.0 * CLK
        yy = sy1 - (sy1 - sy0) * c / smax
        xx = sx0
        while xx < sx1:
            d.line([xx, yy, min(xx + 12 * S, sx1), yy], fill="#ff7f0e", width=2 * S)
            xx += 24 * S
        d.text((sx1 - 6 * S, yy - 6 * S), f"2.66 ms（{c / 1e6:.2f} M）",
               font=get_font(12, True), fill="#ff7f0e", anchor="rb")
        sslot = (sx1 - sx0) / 3.0
        sbw = sslot * 0.42
        for j, (nm, col) in enumerate(zip(["S0 收集+FFT X", "S1 IFFT+FFT+误差",
                                           "S2 梯度/更新"], ["#1f77b4", "#2ca02c",
                                                             "#9467bd"])):
            cx = sx0 + sslot * (j + 0.5)
            hh = (sy1 - sy0) * float(stage[j]) / smax
            d.rectangle([cx - sbw / 2, sy1 - hh, cx + sbw / 2, sy1], fill=col)
            d.text((cx, sy1 - hh - 6 * S),
                   f"{stage[j] / 1e6:.2f}M / {stage[j] / CLK * 1000:.2f} ms",
                   font=get_font(11, True), fill="#111111", anchor="mb")
            d.text((cx, sy1 + 8 * S), nm, font=get_font(11),
                   fill="#111111", anchor="ma")
        d.text(((sx0 + sx1) // 2, sy0 - 26 * S),
               "实时单阶段 cycles（估算，按变换数分摊自 gE5Timing[8]）",
               font=get_font(13, True), fill="#111111", anchor="mm")
        textbox(d, (930, 745), [
            f"实时块平均：{avg / 1e6:.2f} M cycles = {avg / CLK * 1000:.2f} ms",
            f"实时块预算 5.33 ms：{'满足' if avg / CLK * 1000 <= RT_BUDGET_MS else '超出'}",
            f"单次回调 2.66 ms：{'满足' if max(stage) / CLK * 1000 <= DMA_MS else '超出'}",
            f"首块 {TIM.get('rt_first', 0) / 1e6:.2f} M = "
            f"{TIM.get('rt_first', 0) / CLK * 1000:.2f} ms；"
            f"最大 {TIM.get('rt_max', 0) / 1e6:.2f} M = "
            f"{TIM.get('rt_max', 0) / CLK * 1000:.2f} ms",
        ], fs=12)
    else:
        d.text((1400 * S // 2, 780 * S),
               "gE5Timing[8] 为 0：本次导出无实时链路耗时数据，未画单阶段柱状图",
               font=get_font(14), fill="#666666", anchor="mm")
    save(img, "e5_timing.png")


# ---------------------------------------------------------------------------
# summary
# ---------------------------------------------------------------------------
def fmt_ms(c):
    return c / CLK * 1000.0


def build_summary():
    L = []
    L.append("# E5 FDAF 系统辨识：导出数据分析\n")
    L.append(f"> 数据目录：`{DATA}`  ")
    L.append(f"> 图目录：`{FIG}`  ")
    L.append(f"> 平台：fs = {FS:.0f} Hz，SC589 Core1 450 MHz；"
             f"1 块 = 256 点 = {BLOCK_MS:.3f} ms；"
             f"实时块预算 {RT_BUDGET_MS:.2f} ms，DMA 单次回调期限 {DMA_MS:.2f} ms。\n")
    L.append("## 1 导出数据与实时模式\n")
    L.append("| 变量 | 状态 |")
    L.append("|---|---|")
    for nm in ["gE5Hhat", "gE5HtrueT", "gE5Timing", "gE5Debug", "gE5BenchMSE",
               "gE5RtMSE", "gRawCapture", "gProcessedCapture",
               "gProcessed1024Capture"]:
        p = os.path.join(DATA, nm + ".dat")
        L.append(f"| `{nm}.dat` | {'已读取' if os.path.exists(p) else '**缺失**'} |")
    L.append("")
    L.append(f"- 实时模式：**{MODE_LABEL}**（`gE5Timing[11]` = "
             f"{RT_MODE if RT_MODE is not None else '—'}）")
    L.append(f"- 约束标志：**{CONSTRAINT_LABEL}**（`gE5Timing[12]` = "
             f"{int(CONSTRAINT) if CONSTRAINT is not None else '—'}）")
    if TIM:
        L.append(f"- μ = {TIM.get('mu', float('nan')):.2e}；"
                 f"实时块数 = {int(DBG.get('blocks', 0))}")
    L.append("")

    L.append("## 2 benchmark 变体：耗时与精度\n")
    L.append("| 变体 | cycles/块 | cycles/点 | ms/块 | 折合 256 点 ms | "
             "满足 5.33 ms | 失配 / dB | 收敛 -20 dB（块 / ms） |")
    L.append("|---|---:|---:|---:|---:|:---:|---:|---:|")
    if TIM:
        for i in range(5):
            c = float(TIM["cyc"][i])
            pts = VARIANT_POINTS[i]
            per_pt = c / pts
            ms = fmt_ms(c)
            eq256 = ms * 256.0 / pts
            period_ms = pts * 1000.0 / FS      # 该变体一个块的周期
            ok = ("是" if ms <= period_ms + 1e-9
                  else f"**否（{ms / period_ms:.2f}× 超时）**")
            note4 = ""
            if i == 4:
                ok = ("是*" if ms <= period_ms else "**否**")
                note4 = "（块周期 21.33 ms；单次回调算完 18.5 ms > DMA 2.66 ms，不能真机实时）"
            mis = TIM["mis"].get(i)
            mis_txt = f"{mis:.1f}" if mis is not None and np.isfinite(mis) else "—"
            cb = bench_conv[i]
            conv_txt = (f"{cb} / {cb * BLOCK_MS:.0f}" if cb is not None
                        else ("未达到" if TIM else "—"))
            L.append(f"| {VARIANT_NAMES[i]} | {c:,.0f} | {per_pt:,.0f} | "
                     f"{ms:.2f} | {eq256:.2f} | {ok} | {mis_txt} | {conv_txt} |")
            if note4:
                L.append(f"| ↳ 说明 |  |  |  |  | {note4} |  |  |")
    else:
        L.append("| — | — | — | — | — | — | — | — |")
    L.append("")
    L.append("说明：变体 4（标准 1024 点，N=2048）按 1024 点块统计，"
             "与 hop=256 的变体不能直接比 cycles/块，故给“折合 256 点 ms”；"
             "收敛到 -20 dB 取 10 块滑动平均首次 ≤ -20 dB 的块序号。\n")

    L.append("## 3 实时链路（S0/S1/S2 分阶段）\n")
    if TIM and TIM.get("rt_avg", 0) > 0:
        avg = TIM["rt_avg"]
        if RT_MODE == 1024 and CONSTRAINT is not None and int(CONSTRAINT) == 0:
            # 分区不约束版：S0 一次 FFT、S1 一次 IFFT+一次 FFT+4 分区乘加、
            # S2 只有 4 分区的频域乘加（没有 IFFT/FFT），按实测块均值 1:2.2:0.5 分摊估算。
            ratio = np.array([1.0, 2.2, 0.5])
            stage_note = ("阶段 cycles 无单独导出变量；分区不约束版 S2 只有频域乘加，"
                          "这里按实测块均值以 1:2.2:0.5 分摊估算（仅作量级参考）。\n")
        else:
            ratio = np.array([1.0, 2.0, 2.0])
            stage_note = ("阶段 cycles 无单独导出变量，按变换数 1:2:2（S0:S1:S2）"
                          "从 `gE5Timing[8]` 分摊估算；阶段实测请以 `gE5Debug` 计数为佐证。\n")
        stage = avg * ratio / ratio.sum()
        L.append("| 阶段 | cycles（估算） | ms | 满足 2.66 ms |")
        L.append("|---|---:|---:|:---:|")
        for nm, c in zip(["S0（收集 + FFT X）", "S1（IFFT + FFT，出 y/d/e）",
                          "S2（梯度/更新）"], stage):
            L.append(f"| {nm} | {c:,.0f} | {fmt_ms(c):.2f} | "
                     f"{'是' if fmt_ms(c) <= DMA_MS else '**否**'} |")
        L.append("")
        L.append(f"- 实时块平均：{avg:,.0f} cycles = {fmt_ms(avg):.2f} ms "
                 f"（预算 {RT_BUDGET_MS:.2f} ms："
                 f"{'满足' if fmt_ms(avg) <= RT_BUDGET_MS else '**超出**'}）")
        L.append(f"- 首块：{TIM['rt_first']:,.0f} cycles = "
                 f"{fmt_ms(TIM['rt_first']):.2f} ms；"
                 f"最大：{TIM['rt_max']:,.0f} cycles = "
                 f"{fmt_ms(TIM['rt_max']):.2f} ms")
        L.append("- " + stage_note)
    else:
        L.append("`gE5Timing[8]` 为 0，未导出实时链路耗时。\n")
    if DBG:
        L.append("| gE5Debug | 值 | 含义 |")
        L.append("|---|---:|---|")
        for key, desc in [("enter", "ProcessBuffers 进入次数"), ("blocks", "块数"),
                          ("s0", "S0 次数"), ("s1", "S1 次数"), ("s2", "S2 次数"),
                          ("s3", "S3 次数"), ("adc", "AdcCount"),
                          ("err", "bEventError"), ("cap_active", "gCaptureActive"),
                          ("cap_frames", "采集帧数")]:
            if key in DBG:
                L.append(f"| `gE5Debug[...]` {desc} | {int(DBG[key])} | |")
        L.append("")
    L.append("### 实时学习曲线\n")
    if Rt is not None and rt_n > 0:
        L.append(f"- 有效块数 {rt_n}（{rt_n * BLOCK_MS:.0f} ms），"
                 f"末块 MSE = {rt_last:.1f} dB")
        if rt_conv is not None:
            L.append(f"- 收敛到 -20 dB：第 {rt_conv} 块（{rt_conv * BLOCK_MS:.0f} ms）")
        else:
            L.append("- 收敛到 -20 dB：未达到")
    else:
        L.append("- `gE5RtMSE` 无有效数据。")
    L.append("")

    L.append("## 4 250 ms 内部采集\n")
    if CAP:
        L.append("| 量 | 值 |")
        L.append("|---|---:|")
        L.append(f"| rms(x) | {CAP['rms_x']:.5f} |")
        L.append(f"| rms(d) | {CAP['rms_d']:.5f} |")
        L.append(f"| rms(e) | {CAP['rms_e']:.6f} |")
        L.append(f"| e 相对 d 的 RMS 比例 | {CAP['ratio_pct']:.3f} % "
                 f"（{CAP['ratio_db']:.1f} dB） |")
        L.append(f"| 峰值 \\|d\\| / 峰值 \\|e\\| | {CAP['peak_d']:.4f} / "
                 f"{CAP['peak_e']:.5f} |")
        L.append("")
        L.append("说明：三个采集数组均为 DSP 内部 float 信号乘同一缩放系数后存 int16；"
                 "e 相对 d 的 RMS 比例直接反映实时自适应收敛后的残余误差。\n")
    else:
        L.append("采集数组不全，未统计。\n")

    L.append("## 5 辨识精度\n")
    if IR:
        sys_name = "系统 B（1024 点混响）" if SYS_B else "系统 A（256 点共振）"
        L.append(f"导出真值对应未知系统：**{sys_name}**。\n")
        L.append("| 比较范围 | 失配（10log10 Σe²/Σh²） | 峰值相对误差 | "
                 "峰值处逐点最大相对误差 | 对齐 lag |")
        L.append("|---|---:|---:|---:|---:|")
        for key, nm in [("256", "前 256 点"), ("512", "前 512 点"),
                        ("all", f"全 {IR['all']['n']} 点")]:
            if key not in IR:
                continue
            m = IR[key]
            L.append(f"| {nm} | {m['mis']:.2f} dB | {m['max_rel']:.2f} % | "
                     f"{m['max_pt']:.1f} % | {m['lag']} 点 |")
        L.append("")
        if "freq_ann" in IR:
            L.append("幅频响应（`e5_freq_response.png`）：\n")
            for tag, ann in IR["freq_ann"].items():
                L.append(f"- {tag}：" + "；".join(ann))
            L.append("")
    else:
        L.append("缺少 `gE5Hhat` / `gE5HtrueT`，未计算失配。\n")

    L.append("## 6 图\n")
    figs = [("e5_learning_curves.png", "5 变体学习曲线"),
            ("e5_ir_256.png", "系统前 256 点冲激响应"),
            ("e5_ir_1024.png", "系统 512/1024 点冲激响应"),
            ("e5_freq_response.png", "真值 vs 辨识幅频响应"),
            ("e5_rt_learning.png", "实时学习曲线"),
            ("e5_capture_time.png", "250 ms 时域波形"),
            ("e5_capture_spectrum.png", "250 ms 采集频谱与系统频响"),
            ("e5_timing.png", "耗时柱状图")]
    rel = os.path.relpath(FIG, DATA).replace("\\", "/")
    for fn, desc in figs:
        if os.path.exists(os.path.join(FIG, fn)):
            L.append(f"![{desc}]({rel}/{fn})\n")
        else:
            L.append(f"- {desc}：`{fn}` 未生成\n")
    if MISSING:
        L.append("## 7 数据缺失说明\n")
        for s in MISSING:
            L.append(f"- {s}")
        L.append("")
    return "\n".join(L)


# ---------------------------------------------------------------------------
# 生成全部图 + summary
# ---------------------------------------------------------------------------
print("\n===== 生成图片 =====")
for fn, name in [(fig_learning, "e5_learning_curves.png"),
                 (fig_ir256, "e5_ir_256.png"),
                 (fig_ir1024, "e5_ir_1024.png"),
                 (fig_freq, "e5_freq_response.png"),
                 (fig_rt, "e5_rt_learning.png"),
                 (fig_capture_time, "e5_capture_time.png"),
                 (fig_capture_spectrum, "e5_capture_spectrum.png"),
                 (fig_timing, "e5_timing.png")]:
    try:
        fn()
    except Exception:                                          # noqa: BLE001
        print(f"[警告] 生成 {name} 失败：")
        traceback.print_exc()
        MISSING.append(f"{name} 生成失败")

summary = build_summary()
sp = os.path.join(DATA, "E5_analysis_summary.md")
os.makedirs(DATA, exist_ok=True)
with open(sp, "w", encoding="utf-8") as f:
    f.write(summary + "\n")
print(f"\n[summary] {sp}")

# ---------------------------------------------------------------------------
# stdout 关键数字表
# ---------------------------------------------------------------------------
print("\n===== E5 关键数字 =====")
print(f"数据目录: {DATA}")
print(f"实时模式: {MODE_LABEL}   约束标志: {CONSTRAINT_LABEL}")
if TIM:
    print(f"\n{'变体':<22}{'cycles/块':>13}{'cycles/点':>10}{'ms/块':>9}"
          f"{'折合256点ms':>12}{'失配dB':>9}{'-20dB块':>9}")
    for i in range(5):
        c = float(TIM["cyc"][i])
        mis = TIM["mis"].get(i)
        mt = f"{mis:>8.1f}" if mis is not None and np.isfinite(mis) else "       —"
        cb = bench_conv[i]
        ct = f"{cb:>8d}" if cb is not None else "      未达"
        print(f"{VARIANT_NAMES[i]:<22}{c:>13,.0f}{c / VARIANT_POINTS[i]:>10,.0f}"
              f"{fmt_ms(c):>9.2f}{fmt_ms(c) * 256 / VARIANT_POINTS[i]:>12.2f}"
              f"{mt}{ct}")
    ert = TIM.get("rt_avg", 0.0)
    print(f"\n实时块: {ert:,.0f} cycles = {fmt_ms(ert):.2f} ms  "
          f"({'满足' if fmt_ms(ert) <= RT_BUDGET_MS else '超出'} 5.33 ms 预算)；"
          f"首块 {fmt_ms(TIM['rt_first']):.2f} ms / 最大 {fmt_ms(TIM['rt_max']):.2f} ms")
    if RT_MODE == 1024 and CONSTRAINT is not None and int(CONSTRAINT) == 0:
        sw = np.array([1.0, 2.2, 0.5])
    else:
        sw = np.array([1.0, 2.0, 2.0])
    st = ert * sw / sw.sum()
    print("单阶段(估): " + "  ".join(
        f"{n}={c:,.0f}({fmt_ms(c):.2f} ms)"
        for n, c in zip(["S0", "S1", "S2"], st)))
if CAP:
    print(f"\n250 ms 采集: rms(d)={CAP['rms_d']:.5f}  rms(e)={CAP['rms_e']:.6f}  "
          f"e/d = {CAP['ratio_pct']:.3f} % ({CAP['ratio_db']:.1f} dB)")
if IR:
    print(f"辨识失配: 前256点 {IR['256']['mis']:.1f} dB / 全段 "
          f"{IR['all']['mis']:.1f} dB（lag={IR['all']['lag']} 点）")
if Rt is not None and rt_n:
    print(f"实时学习曲线: {rt_n} 块有效，末值 {rt_last:.1f} dB，"
          f"收敛 -20 dB 于 " + (f"第 {rt_conv} 块" if rt_conv is not None else "未达到"))
print(f"\n完成。图：{len([f for f in os.listdir(FIG) if f.endswith('.png')])} 张，"
      f"目录 {FIG}")
