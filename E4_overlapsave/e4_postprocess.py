# -*- coding: utf-8 -*-
"""E4 overlap-save FIR: post-process CCES exported data and generate report figures."""
import os
import re
import math
import numpy as np
from PIL import Image, ImageDraw, ImageFont

BASE = r"D:\DSP_experiment\E4_overlapsave"
FIG = os.path.join(BASE, "analysis_figs")
os.makedirs(FIG, exist_ok=True)

FS = 48000.0
CLK = 450e6
TAPS1, TAPS2, FC = 256, 1024, 2000.0
NFFT = 4096

FONT_REG = r"C:\Windows\Fonts\msyh.ttc"
FONT_BOLD = r"C:\Windows\Fonts\msyhbd.ttc"
S = 2  # supersampling


# --------------------------------------------------------------------------
# data
# --------------------------------------------------------------------------
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
    w = {"int16": 2, "single": 4}[typ]
    b = b[: len(b) // w * w]
    dt = "<i2" if typ == "int16" else "<f4"
    return np.frombuffer(b, dtype=dt).astype(np.float64)[:count]


def load_mode(folder):
    p = os.path.join(BASE, folder)
    raw = read_mem(os.path.join(p, "gRawCapture.dat"), "int16", 12000)
    p256 = read_mem(os.path.join(p, "gProcessedCapture.dat"), "int16", 12000)
    p1024 = read_mem(os.path.join(p, "gProcessed1024Capture.dat"), "int16", 12000)
    tim = read_mem(os.path.join(p, "gE4Timing.dat"), "single", 13)
    mode = int(round(tim[12]))
    proc = p256 if mode == 256 else p1024
    return raw, proc, tim, mode


# --------------------------------------------------------------------------
# DSP
# --------------------------------------------------------------------------
def welch_psd(x, nfft=NFFT, fs=FS):
    x = np.asarray(x, dtype=float)
    x = x - x.mean()
    noverlap = nfft // 2
    step = nfft - noverlap
    win = 0.5 - 0.5 * np.cos(2 * np.pi * np.arange(nfft) / (nfft - 1))
    P = np.zeros(nfft // 2 + 1)
    used = 0
    for s in range(0, len(x) - nfft + 1, step):
        seg = x[s:s + nfft] * win
        X = np.fft.rfft(seg)
        P += np.abs(X) ** 2
        used += 1
    P /= max(1, used)
    P[1:-1] *= 2
    P /= fs * np.sum(win ** 2)
    f = np.arange(nfft // 2 + 1) * fs / nfft
    return f, P


def welch_cross(x, y, nfft=NFFT, fs=FS):
    x = np.asarray(x, dtype=float); y = np.asarray(y, dtype=float)
    x = x - x.mean(); y = y - y.mean()
    noverlap = nfft // 2
    step = nfft - noverlap
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
    Pxx /= max(1, used); Pxy /= max(1, used)
    f = np.arange(nfft // 2 + 1) * fs / nfft
    return f, Pxy, Pxx


def design_lpf(taps, fc=FC, fs=FS):
    M = taps - 1
    n = np.arange(taps)
    x = 2 * fc / fs * (n - M / 2)
    s = np.ones(taps)
    idx = np.abs(x) > 1e-12
    s[idx] = np.sin(np.pi * x[idx]) / (np.pi * x[idx])
    w = 0.54 - 0.46 * np.cos(2 * np.pi * n / M)
    h = s * w
    return h / h.sum()


def freq_resp(h, n=16384, fs=FS):
    H = np.fft.rfft(h, n)
    f = np.arange(n // 2 + 1) * fs / n
    return f, H


def tone_level(f, P, f0, half=2):
    idx = np.where(np.abs(f - f0) <= half * (f[1] - f[0]))[0]
    i = idx[np.argmax(P[idx])]
    return P[i], f[i]


def h_at(f, H, f0, half=2):
    idx = np.where(np.abs(f - f0) <= half * (f[1] - f[0]))[0]
    i = idx[np.argmax(np.abs(H[idx]))]
    return abs(H[i]), f[i]


def resp_exact(h, f0):
    n = np.arange(len(h))
    return abs(np.sum(h * np.exp(-1j * 2 * np.pi * f0 * n / FS)))


# --------------------------------------------------------------------------
# plotting
# --------------------------------------------------------------------------
def get_font(size, bold=False):
    return ImageFont.truetype(FONT_BOLD if bold else FONT_REG, size * S)


def canvas(w, h, title):
    img = Image.new("RGB", (w * S, h * S), "white")
    d = ImageDraw.Draw(img)
    d.text((w * S // 2, 22 * S), title, font=get_font(24, True),
           fill="#111111", anchor="mm")
    return img, d


def panel(d, box, xlim, ylim, xticks, yticks, title, xlabel, ylabel,
          xtick_fmt="{:.0f}", ytick_fmt="{:.0f}"):
    x0, y0, x1, y1 = [v * S for v in box]
    d.rectangle([x0, y0, x1, y1], fill="white", outline="#333333", width=S)
    d.text(((x0 + x1) // 2, y0 - 28 * S), title, font=get_font(18, True),
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
    d.text(((x0 + x1) // 2, y1 + 42 * S), xlabel, font=get_font(15),
           fill="#111111", anchor="mm")
    d.text((x0, y0 - 10 * S), ylabel, font=get_font(15), fill="#111111", anchor="lb")
    return X, Y


def line(d, xs, ys, X, Y, color, width=2):
    pts = [(X(x), Y(y)) for x, y in zip(xs, ys)]
    if len(pts) > 1:
        d.line(pts, fill=color, width=width * S, joint="curve")


def legend(d, box, items, font_size=13):
    x0, y0, x1, y1 = [v * S for v in box]
    pad = 8 * S
    line_h = 24 * S
    w = x1 - x0
    h = y1 - y0
    d.rectangle([x0, y0, x1, y1], fill="white", outline="#999999", width=S)
    for i, (label, color) in enumerate(items):
        yy = y0 + pad + i * line_h + line_h // 2
        d.line([x0 + pad, yy, x0 + pad + 38 * S, yy], fill=color, width=3 * S)
        d.text((x0 + pad + 48 * S, yy), label, font=get_font(font_size),
               fill="#111111", anchor="lm")


def vline(d, x, X, Y, y0, y1, color="#666666", dash=6):
    xx = X(x)
    yy0, yy1 = Y(y0), Y(y1)
    step = dash * S
    y = yy0
    while y < yy1:
        d.line([xx, y, xx, min(y + step, yy1)], fill=color, width=S)
        y += 2 * step


# --------------------------------------------------------------------------
# load
# --------------------------------------------------------------------------
D256 = load_mode("data_256")
D1024 = load_mode("data_1024")

f_raw256, P_raw256 = welch_psd(D256[0])
f_p256, P_p256 = welch_psd(D256[1])
f_raw1024, P_raw1024 = welch_psd(D1024[0])
f_p1024, P_p1024 = welch_psd(D1024[1])

f_h, H256 = freq_resp(design_lpf(TAPS1))
_, H1024 = freq_resp(design_lpf(TAPS2))

# keep frequency grids identical
assert np.allclose(f_raw256, f_p256) and np.allclose(f_raw1024, f_p1024)

# transfer function
f_hx256, Pxy256, Pxx256 = welch_cross(D256[0], D256[1])
f_hx1024, Pxy1024, Pxx1024 = welch_cross(D1024[0], D1024[1])
Hm256 = Pxy256 / Pxx256
Hm1024 = Pxy1024 / Pxx1024

# reference: raw max
ref256 = P_raw256.max()
ref1024 = P_raw1024.max()
raw256_db = 10 * np.log10(P_raw256 / ref256 + 1e-30)
proc256_db = 10 * np.log10(P_p256 / ref256 + 1e-30)
raw1024_db = 10 * np.log10(P_raw1024 / ref1024 + 1e-30)
proc1024_db = 10 * np.log10(P_p1024 / ref1024 + 1e-30)

# timing
t = D256[2]
tim_rows = [
    ("OS 255", t[0], t[4]),
    ("Direct 255", t[1], t[4]),
    ("OS 1023", t[2], t[5]),
    ("Direct 1023", t[3], t[5]),
]

# tone table
tone_rows = []
for f0 in [1000, 2000, 6000]:
    r256, _ = tone_level(f_raw256, P_raw256, f0)
    p256v, _ = tone_level(f_p256, P_p256, f0)
    r1024, _ = tone_level(f_raw1024, P_raw1024, f0)
    p1024v, _ = tone_level(f_p1024, P_p1024, f0)
    hm256, _ = h_at(f_hx256, Hm256, f0)
    hm1024, _ = h_at(f_hx1024, Hm1024, f0)
    hth256 = resp_exact(design_lpf(TAPS1), f0)
    hth1024 = resp_exact(design_lpf(TAPS2), f0)
    tone_rows.append((f0,
                      10 * np.log10(r256 + 1e-30), 10 * np.log10(p256v + 1e-30),
                      10 * np.log10(p256v / r256 + 1e-30),
                      10 * np.log10(r1024 + 1e-30), 10 * np.log10(p1024v + 1e-30),
                      10 * np.log10(p1024v / r1024 + 1e-30),
                      20 * np.log10(hm256 + 1e-30), 20 * np.log10(hm1024 + 1e-30),
                      20 * np.log10(hth256 + 1e-30), 20 * np.log10(hth1024 + 1e-30)))

# --------------------------------------------------------------------------
# figure 1: time
# --------------------------------------------------------------------------
def aligned(raw, proc, delay):
    proc2 = proc[delay:]
    raw2 = raw[:len(proc2)]
    tt = np.arange(len(proc2)) / FS * 1000
    return tt, raw2, proc2


for mode, raw, proc, delay, tag in [
    (256, D256[0], D256[1], 384, "255 阶"),
    (1024, D1024[0], D1024[1], 768, "1023 阶 4 分区"),
]:
    img, d = canvas(1500, 920, "E4 重叠保留法 FIR：时域（raw vs processed）")
    tt, rr, pp = aligned(raw, proc, delay)
    ymax = max(np.max(np.abs(rr)), np.max(np.abs(pp))) * 1.08
    ylim = (-ymax, ymax)
    n = min(4000, len(tt))
    step = max(1, len(tt) // n)
    xs = tt[::step]
    r = rr[::step]; p = pp[::step]

    # top: 20 ms
    box = (100, 80, 1460, 430)
    X, Y = panel(d, box, (0, 20), ylim,
                 np.arange(0, 21, 2), np.linspace(ylim[0], ylim[1], 7),
                 f"{tag}：前 20 ms（已按延迟对齐）", "时间 / ms", "幅度 / LSB")
    line(d, xs, r, X, Y, "#111111", 1)
    line(d, xs, p, X, Y, "#1f77b4", 1)
    legend(d, (1190, 100, 1440, 160), [("raw", "#111111"), (tag, "#1f77b4")])

    # bottom: full
    box = (100, 540, 1460, 870)
    X, Y = panel(d, box, (0, 250), ylim,
                 np.arange(0, 251, 50), np.linspace(ylim[0], ylim[1], 6),
                 f"{tag}：整段 250 ms", "时间 / ms", "幅度 / LSB")
    line(d, xs, r, X, Y, "#111111", 1)
    line(d, xs, p, X, Y, "#1f77b4", 1)
    legend(d, (1190, 560, 1440, 620), [("raw", "#111111"), (tag, "#1f77b4")])
    img.save(os.path.join(FIG, f"E4_time_{mode}.png"))

# --------------------------------------------------------------------------
# figure 2: spectrum
# --------------------------------------------------------------------------
for mode, f, rawdb, procdb, tag in [
    (256, f_raw256, raw256_db, proc256_db, "255 阶"),
    (1024, f_raw1024, raw1024_db, proc1024_db, "1023 阶 4 分区"),
]:
    img, d = canvas(1500, 920, f"E4 频谱（Welch，{tag}）")
    # full
    X, Y = panel(d, (100, 80, 1460, 430), (0, 24000), (-100, 10),
                 np.arange(0, 24001, 4000), np.arange(-100, 11, 20),
                 "全频段 0–24 kHz", "频率 / Hz", "相对幅度 / dB")
    line(d, f, rawdb, X, Y, "#111111", 1)
    line(d, f, procdb, X, Y, "#1f77b4", 1)
    legend(d, (1190, 100, 1440, 160), [("raw", "#111111"), (tag, "#1f77b4")])
    # zoom
    m = (f >= 200) & (f <= 8000)
    X, Y = panel(d, (100, 540, 1460, 870), (200, 8000), (-100, 10),
                 np.arange(1000, 8001, 1000), np.arange(-100, 11, 20),
                 "放大 200–8000 Hz", "频率 / Hz", "相对幅度 / dB")
    line(d, f[m], rawdb[m], X, Y, "#111111", 1)
    line(d, f[m], procdb[m], X, Y, "#1f77b4", 1)
    for f0, lab in [(1000, "1k"), (2000, "2k"), (6000, "6k")]:
        vline(d, f0, X, Y, -100, 10)
        d.text((X(f0) + 4 * S, Y(8)), lab, font=get_font(13), fill="#444444", anchor="lm")
    legend(d, (1190, 560, 1440, 620), [("raw", "#111111"), (tag, "#1f77b4")])
    img.save(os.path.join(FIG, f"E4_spectrum_{mode}.png"))

# --------------------------------------------------------------------------
# figure 3: filter response
# --------------------------------------------------------------------------
img, d = canvas(1500, 920, "E4 低通滤波器幅频响应（fc = 2 kHz）")

mag256 = 20 * np.log10(np.abs(H256) + 1e-30)
mag1024 = 20 * np.log10(np.abs(H1024) + 1e-30)

# upper: 0-4 kHz detail
m = f_h <= 4000
X, Y = panel(d, (100, 80, 1460, 430), (0, 4000), (-100, 5),
             np.arange(0, 4001, 500), np.arange(-100, 1, 20),
             "0–4 kHz 细节", "频率 / Hz", "幅度 / dB")
line(d, f_h[m], mag256[m], X, Y, "#1f77b4", 2)
line(d, f_h[m], mag1024[m], X, Y, "#d62728", 2)
vline(d, 2000, X, Y, -100, 5)
d.text((X(2000) + 4 * S, Y(-5)), "fc=2k", font=get_font(13), fill="#444444", anchor="lm")
legend(d, (1120, 100, 1440, 170),
       [("255 阶（256 taps）", "#1f77b4"), ("1023 阶（1024 taps）", "#d62728")])

# lower: whole band, max envelope to avoid dense ripple lines
def envelope(f, mag, block=64):
    n = len(f) // block * block
    return (f[:n].reshape(-1, block).mean(axis=1),
            mag[:n].reshape(-1, block).max(axis=1))

fe, me256 = envelope(f_h, mag256)
_, me1024 = envelope(f_h, mag1024)
X, Y = panel(d, (100, 540, 1460, 880), (0, 24000), (-120, 5),
             np.arange(0, 24001, 4000), np.arange(-120, 1, 20),
             "全频段（每 64 个 bin 取最大包络）", "频率 / Hz", "幅度 / dB")
line(d, fe, me256, X, Y, "#1f77b4", 2)
line(d, fe, me1024, X, Y, "#d62728", 2)
vline(d, 6000, X, Y, -120, 5)
d.text((X(6000) + 4 * S, Y(-8)), "6k", font=get_font(13), fill="#444444", anchor="lm")
legend(d, (1120, 560, 1440, 630),
       [("255 阶（256 taps）", "#1f77b4"), ("1023 阶（1024 taps）", "#d62728")])
img.save(os.path.join(FIG, "E4_filter_response.png"))

# --------------------------------------------------------------------------
# figure 4: timing
# --------------------------------------------------------------------------
img, d = canvas(1400, 760, "E4 运算耗时：重叠保留 vs 逐点 FIR")
vals = [v[1] for v in tim_rows]
labels = [v[0] for v in tim_rows]
colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]
x0, y0, x1, y1 = 110 * S, 90 * S, 1360 * S, 650 * S
ymax = max(vals) * 1.20
d.rectangle([x0, y0, x1, y1], fill="white", outline="#333333", width=S)
for k in range(6):
    yy = y1 - (y1 - y0) * k / 5
    d.line([x0, yy, x1, yy], fill="#dddddd", width=S)
    d.text((x0 - 10 * S, yy), f"{ymax * k / 5 / 1e6:.1f}M", font=get_font(13),
           fill="#222222", anchor="rm")
bw = (x1 - x0) / (len(vals) * 1.6)
for i, (v, lab, col) in enumerate(zip(vals, labels, colors)):
    cx = x0 + (i + 0.8) * (x1 - x0) / len(vals)
    h = (y1 - y0) * v / ymax
    d.rectangle([cx - bw / 2, y1 - h, cx + bw / 2, y1], fill=col)
    d.text((cx, y1 - h - 6 * S), f"{v/1e6:.2f}M", font=get_font(13, True),
           fill="#111111", anchor="mb")
    d.text((cx, y1 + 10 * S), lab, font=get_font(14), fill="#111111", anchor="ma")
d.text(((x0 + x1) // 2, y1 + 50 * S), "算法", font=get_font(15), fill="#111111", anchor="mm")
d.text((x0, y0 - 30 * S), "cycles / 256 点块", font=get_font(15), fill="#111111", anchor="lm")
d.text(((x0 + x1) // 2, y1 + 92 * S),
       f"加速比：255 阶 {t[4]:.2f}×，1023 阶 {t[5]:.2f}×（450 MHz）",
       font=get_font(16, True), fill="#111111", anchor="mm")
img.save(os.path.join(FIG, "E4_timing.png"))

# --------------------------------------------------------------------------
# report
# --------------------------------------------------------------------------
def fmt_us(c):
    return c / CLK * 1e6

lines = []
lines.append("# E4 重叠保留法 FIR：导出数据分析\n")
lines.append("> 数据：`data_256` / `data_1024`，48 kHz、250 ms（12000 点），"
             "CCES 导出，每行一个字节（小端）。\n")
lines.append("## 1. 采集模式与数据规模\n")
lines.append("| 目录 | 实时模式 | 采样点 | 说明 |")
lines.append("|---|---:|---:|---|")
lines.append(f"| data_256 | 255 阶（256 taps） | {len(D256[0])} | `gProcessed1024Capture` 为 0 |")
lines.append(f"| data_1024 | 1023 阶（4×256 分区） | {len(D1024[0])} | `gProcessedCapture` 与 `gProcessed1024Capture` 相同 |")
lines.append("")
lines.append("## 2. 运算耗时（启动时 5 块平均，256 点/块）\n")
lines.append("| 算法 | cycles/块 | cycles/点 | μs/块（450 MHz） | 加速比 |")
lines.append("|---|---:|---:|---:|---:|")
for lab, cyc, spd in tim_rows:
    spd_txt = "—" if lab.startswith("OS") else f"{spd:.2f}×"
    lines.append(f"| {lab} | {cyc:,.0f} | {cyc/256:.1f} | {fmt_us(cyc):.1f} | {spd_txt} |")
lines.append("")
lines.append(f"处理块数：{int(t[6])}；采样率：{int(t[11])} Hz。\n")
lines.append("## 3. 精度（重叠保留 vs 同阶逐点 FIR）\n")
lines.append("| 模式 | 最大相对误差 | 最大绝对误差 / LSB |")
lines.append("|---|---:|---:|")
lines.append(f"| 255 阶 | {t[7]:.3e} | {t[9]:.3f} |")
lines.append(f"| 1023 阶 | {t[8]:.3e} | {t[10]:.3f} |")
lines.append("")
lines.append("## 4. 频谱与滤波效果（Welch，4096 点，Hann）\n")
lines.append("| 频率 | 256 raw / dB | 256 proc / dB | 256 实测衰减 | "
             "1024 raw / dB | 1024 proc / dB | 1024 实测衰减 | 理论 256 | 理论 1024 |")
lines.append("|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
for r in tone_rows:
    f0, r256, p256v, a256, r1024, p1024v, a1024, hm256, hm1024, ht256, ht1024 = r
    lines.append(f"| {f0} Hz | {r256:.1f} | {p256v:.1f} | {a256:.1f} dB | "
                 f"{r1024:.1f} | {p1024v:.1f} | {a1024:.1f} dB | "
                 f"{ht256:.1f} dB | {ht1024:.1f} dB |")
lines.append("")
lines.append("说明：实测衰减为 processed/raw 的 Welch 谱线比；"
             "6 kHz 受采集噪声底限制，约 -64 dB；滤波器理论阻带为 -80 / -92 dB。"
             "输入本身含明显低频哼声和幅度起伏，1 kHz 的 raw/proc 差值会受输入幅度变化影响，"
             "滤波器增益以理论列为准。\n")
lines.append("![time256](analysis_figs/E4_time_256.png)\n")
lines.append("![spec256](analysis_figs/E4_spectrum_256.png)\n")
lines.append("![time1024](analysis_figs/E4_time_1024.png)\n")
lines.append("![spec1024](analysis_figs/E4_spectrum_1024.png)\n")
lines.append("![filter](analysis_figs/E4_filter_response.png)\n")
lines.append("![timing](analysis_figs/E4_timing.png)\n")
lines.append("## 5. 结论\n")
lines.append("- 255 阶 OS 约 0.716M cycles/块（1.59 ms），比逐点 255 阶快 3.04×；")
lines.append("- 1023 阶 4 分区 OS 约 0.880M cycles/块（1.96 ms），比逐点 1023 阶快 9.86×；")
lines.append("- 2 kHz 在截止点理论约 -6 dB（实测 -3.4~-5.6 dB，受输入幅度起伏影响）；")
lines.append("- 6 kHz 实测被压约 64 dB，受采集噪声底限制；滤波器理论阻带为 -80 / -92 dB；")
lines.append("- 1 kHz 理论 0 dB；实测 raw/proc 差值受输入幅度波动影响，1024 模式甚至为正，"
             "滤波器增益以理论响应为准；")
lines.append("- 输入含明显低频哼声，低通滤波器会保留这部分低频，因此 processed 波形仍保留慢变分量；")
lines.append("- 重叠保留输出与同阶逐点 FIR 的最大相对误差在 1e-6 量级，算法一致。")
lines.append("")

with open(os.path.join(BASE, "E4_analysis.md"), "w", encoding="utf-8") as f:
    f.write("\n".join(lines))

# print summary
print("Timing:")
for lab, cyc, spd in tim_rows:
    print(f"  {lab:12s} cycles={cyc:,.0f}  us={fmt_us(cyc):.1f}  speedup={spd:.2f}")
print("Tones:")
for r in tone_rows:
    print("  ", r)
print("Figures:", sorted(os.listdir(FIG)))
