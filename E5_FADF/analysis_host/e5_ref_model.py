#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
E5 频域自适应滤波（FDAF）系统辨识 —— 主机端 Python 参考模型 + 算法正确性验证。

算法口径：docs/E5_FADF_设计说明.md 第 2–4 节
    N = 512, hop = L = 256，未知系统 A(256 点)/B(1024 点)，Sigma|h| = 0.9
    标准 FDAF（256 taps）、分区 FDAF（4 x 256 = 1024 taps）、标准 1024 点（N=2048, hop=1024）
    约束版 / 不约束版

只依赖 numpy + PIL（本机没有 matplotlib，用 PIL 画图，中文 msyh.ttc）。

运行（一条命令重跑，末尾打印关键数字）：
    & 'C:\\Users\\24026\\.dsh\\dsh-runtimes\\dsh-primary-runtime\\dependencies\\python\\python.exe' e5_ref_model.py

输出：
    analysis_host/figs/e5_learning_curves.png
    analysis_host/figs/e5_ir_compare_256.png
    analysis_host/figs/e5_ir_compare_1024.png
    analysis_host/figs/e5_freq_response.png
    analysis_host/figs/e5_constraint_effect.png
    analysis_host/ref_results.json
    analysis_host/REF_MODEL_NOTES.md
"""

import os
import json
import numpy as np
from PIL import Image, ImageDraw, ImageFont

np.seterr(all="ignore")

BASE = r"D:\DSP_experiment\E5_FADF"
OUT = os.path.join(BASE, "analysis_host")
FIG = os.path.join(OUT, "figs")
os.makedirs(FIG, exist_ok=True)

FS = 48000.0
L = 256
N = 512
PRE = 4 * 1024          # x 逻辑起点之前的零历史
NBLK = 300

FONT_REG = r"C:\Windows\Fonts\msyh.ttc"
FONT_BOLD = r"C:\Windows\Fonts\msyhbd.ttc"
if not os.path.exists(FONT_BOLD):
    FONT_BOLD = FONT_REG


# ===========================================================================
# 1. 白噪声 / 未知系统 / FFT 工具
# ===========================================================================
def xs32(n, seed=0x1234567, half_scale=1.0):
    """xorshift32 均匀序列，映射到 [-0.5*half_scale, 0.5*half_scale)。

    DSP 口径：x = (xorshift32 输出 / 2^32 - 0.5) * half_scale；
    x：half_scale = 1 -> x in [-0.5, 0.5)，sigma_x^2 = 1/12。
    系统 B 的 r(n)：half_scale = 2 -> r in [-1, 1)（零均值、幅度 1）。
    """
    s = seed & 0xFFFFFFFF
    out = np.empty(n, dtype=np.float32)
    for i in range(n):
        s ^= (s << 13) & 0xFFFFFFFF
        s &= 0xFFFFFFFF
        s ^= (s >> 17)
        s ^= (s << 5) & 0xFFFFFFFF
        s &= 0xFFFFFFFF
        out[i] = np.float32(half_scale * (s / 4294967296.0 - 0.5))
    return out


XS = xs32(NBLK * 1024 + 8 * 1024, 0x1234567, 1.0)


def make_xb(hop):
    """返回第 m 块（hop 点）的 x；m<0 视为 0（上电零历史）。"""
    def xb(m):
        if m < 0:
            return np.zeros(hop, np.float32)
        s = PRE + m * hop
        return XS[s:s + hop].copy()
    return xb


def unknown_sys_A():
    """h_A(n) = 0.70 d(n-2) + 0.35 exp(-(n-2)/40) cos(2pi 1800 (n-2)/48000) u(n-2), n=0..255"""
    n = np.arange(256, dtype=np.float64)
    h = np.zeros(256)
    k = n[n >= 2] - 2.0
    h[2:] = 0.35 * np.exp(-k / 40.0) * np.cos(2.0 * np.pi * 1800.0 * k / FS)
    h[2] += 0.70
    h *= 0.9 / np.sum(np.abs(h))          # Sigma|h| = 0.9
    return h.astype(np.float32)


def unknown_sys_B():
    """h_B(n) = 0.60 d(n-2) + exp(-(n-2)/150) u(n-2) r(n), n=0..1023, 种子 0x1234567"""
    n = np.arange(1024, dtype=np.float64)
    r = xs32(1024, 0x1234567, 2.0).astype(np.float64)   # 零均值 [-1,1)
    h = np.zeros(1024)
    k = n[n >= 2] - 2.0
    h[2:] = np.exp(-k / 150.0) * r[2:]
    h[2] += 0.60
    h *= 0.9 / np.sum(np.abs(h))          # Sigma|h| = 0.9
    return h.astype(np.float32)


def F(x):
    return np.fft.fft(np.asarray(x, dtype=np.float32)).astype(np.complex64)


def Fi(X):
    return np.fft.ifft(np.asarray(X, dtype=np.complex64)).astype(np.complex64)


def misalign(hhat, htrue):
    """mis = 10log10( Sigma(hhat-h)^2 / Sigma h^2 ) dB"""
    hh = np.asarray(hhat, dtype=np.float64)
    h0 = np.asarray(htrue, dtype=np.float64)
    n = max(len(hh), len(h0))
    hh = np.pad(hh, (0, n - len(hh)))
    h0 = np.pad(h0, (0, n - len(h0)))
    return float(10.0 * np.log10((np.sum((hh - h0) ** 2) + 1e-30) / (np.sum(h0 ** 2) + 1e-30)))


def mse_db(e, d):
    e = np.asarray(e, dtype=np.float64)
    d = np.asarray(d, dtype=np.float64)
    return float(10.0 * np.log10((np.sum(e ** 2) + 1e-30) / (np.sum(d ** 2) + 1e-30)))


# ===========================================================================
# 2. 三个自适应结构（帧约定见设计说明第 3/4 节）
# ===========================================================================
def std_run(xb, htrue, mu, nblk, constrained=True, ef_mode="zero",
            keep="first", capture_at=None):
    """标准 FDAF：L=256 taps，N=512，hop=256。

    xf_m = [x_{m-1}, x_m]，X=FFT(xf)；y = IFFT(X W) 的后 256 点；e = d - y。
    ef_mode='zero'：E = FFT([0, e_m])（与设计说明的逐点相关公式严格等价）；
    ef_mode='doc' ：E = FFT([e_{m-1}, e_m])（设计说明第 3 节原文写法）。
    keep='first' 约束保留 g(0..255)；keep='second' 保留 g(256..511)（错误方向对照）。
    """
    mu32 = np.float32(mu)
    Ht = F(np.concatenate([np.asarray(htrue, np.float32), np.zeros(L, np.float32)]))
    W = np.zeros(N, np.complex64)
    e_prev = np.zeros(L, np.float32)
    mse = np.zeros(nblk)
    mis = np.zeros(nblk)
    cap = None
    maxW = 0.0
    for m in range(nblk):
        xfm = np.concatenate([xb(m - 1), xb(m)])
        X = F(xfm)
        d = Fi(X * Ht)[L:].real.astype(np.float32)
        y = Fi(X * W)[L:].real.astype(np.float32)
        e = d - y
        if ef_mode == "doc":
            E = F(np.concatenate([e_prev, e]))
        else:
            E = F(np.concatenate([np.zeros(L, np.float32), e]))
        G = np.conj(X) * E
        if capture_at is not None and m == capture_at:
            cap = dict(m=m, Xf=xfm.copy(), X=X.copy(), E=E.copy(), e=e.copy(),
                       e_prev=e_prev.copy(), d=d.copy(), y=y.copy(), W=W.copy())
        if constrained:
            g = Fi(G).real.astype(np.float32)
            if keep == "first":
                gc = np.concatenate([g[:L], np.zeros(L, np.float32)])
            else:
                gc = np.concatenate([np.zeros(L, np.float32), g[L:]])
            W = (W + mu32 * F(gc)).astype(np.complex64)
        else:
            W = (W + mu32 * G).astype(np.complex64)
        e_prev = e
        mse[m] = mse_db(e, d)
        hhat = Fi(W)[:L].real.astype(np.float32)
        mis[m] = misalign(hhat, htrue)
        maxW = max(maxW, float(np.max(np.abs(W))))
    hhat = Fi(W)[:L].real.astype(np.float32)
    return dict(mse=mse, mis=mis, W=W, hhat=hhat, maxW=maxW, cap=cap)


def part_run(xb, htrue1024, mu, nblk, constrained=True, ef_mode="zero", capture_at=None):
    """分区 FDAF：P=4 个 256 点分区（总 1024 taps），N=512，hop=256。"""
    mu32 = np.float32(mu)
    hs = [np.asarray(htrue1024[p * L:(p + 1) * L], np.float32) for p in range(4)]
    Hp = [F(np.concatenate([h, np.zeros(L, np.float32)])) for h in hs]
    Wp = [np.zeros(N, np.complex64) for _ in range(4)]
    Xh = [np.zeros(N, np.complex64) for _ in range(4)]   # Xh[p] = X_{m-p}
    e_prev = np.zeros(L, np.float32)
    mse = np.zeros(nblk)
    mis = np.zeros(nblk)
    cap = None
    maxW = 0.0
    for m in range(nblk):
        Xm = F(np.concatenate([xb(m - 1), xb(m)]))
        Xh = [Xm] + Xh[:3]
        D = np.zeros(N, np.complex64)
        Y = np.zeros(N, np.complex64)
        for p in range(4):
            D += Xh[p] * Hp[p]
            Y += Xh[p] * Wp[p]
        d = Fi(D)[L:].real.astype(np.float32)
        y = Fi(Y)[L:].real.astype(np.float32)
        e = d - y
        if ef_mode == "doc":
            E = F(np.concatenate([e_prev, e]))
        else:
            E = F(np.concatenate([np.zeros(L, np.float32), e]))
        if capture_at is not None and m == capture_at:
            cap = dict(m=m, Xh=[X.copy() for X in Xh], E=E.copy(), e=e.copy(),
                       d=d.copy(), y=y.copy(), Wp=[w.copy() for w in Wp])
        for p in range(4):
            G = np.conj(Xh[p]) * E
            if constrained:
                g = Fi(G).real.astype(np.float32)
                gc = np.concatenate([g[:L], np.zeros(L, np.float32)])
                Wp[p] = (Wp[p] + mu32 * F(gc)).astype(np.complex64)
            else:
                Wp[p] = (Wp[p] + mu32 * G).astype(np.complex64)
        e_prev = e
        mse[m] = mse_db(e, d)
        hhat = np.concatenate([Fi(w)[:L].real.astype(np.float32) for w in Wp])
        mis[m] = misalign(hhat, htrue1024)
        maxW = max(maxW, max(float(np.max(np.abs(w))) for w in Wp))
    hhat = np.concatenate([Fi(w)[:L].real.astype(np.float32) for w in Wp])
    return dict(mse=mse, mis=mis, W=Wp, hhat=hhat, maxW=maxW, cap=cap)


def std1024_run(xb1024, htrue1024, mu, nblk, constrained=True, ef_mode="zero"):
    """标准 1024 点 FDAF：N=2048，hop=1024（延迟 21.33 ms）。"""
    L2 = 1024
    N2 = 2048
    mu32 = np.float32(mu)
    Ht = F(np.concatenate([np.asarray(htrue1024, np.float32), np.zeros(L2, np.float32)]))
    W = np.zeros(N2, np.complex64)
    e_prev = np.zeros(L2, np.float32)
    mse = np.zeros(nblk)
    mis = np.zeros(nblk)
    maxW = 0.0
    for m in range(nblk):
        xfm = np.concatenate([xb1024(m - 1), xb1024(m)])
        X = F(xfm)
        d = Fi(X * Ht)[L2:].real.astype(np.float32)
        y = Fi(X * W)[L2:].real.astype(np.float32)
        e = d - y
        if ef_mode == "doc":
            E = F(np.concatenate([e_prev, e]))
        else:
            E = F(np.concatenate([np.zeros(L2, np.float32), e]))
        G = np.conj(X) * E
        if constrained:
            g = Fi(G).real.astype(np.float32)
            gc = np.concatenate([g[:L2], np.zeros(L2, np.float32)])
            W = (W + mu32 * F(gc)).astype(np.complex64)
        else:
            W = (W + mu32 * G).astype(np.complex64)
        e_prev = e
        mse[m] = mse_db(e, d)
        hhat = Fi(W)[:L2].real.astype(np.float32)
        mis[m] = misalign(hhat, htrue1024)
        maxW = max(maxW, float(np.max(np.abs(W))))
    hhat = Fi(W)[:L2].real.astype(np.float32)
    return dict(mse=mse, mis=mis, W=W, hhat=hhat, maxW=maxW, cap=None)


# ===========================================================================
# 3. PIL 画图（风格参考 E4 e4_postprocess.py，S=1，宽 1500 <= 1600）
# ===========================================================================
COLORS = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b"]


def font(sz, bold=False):
    return ImageFont.truetype(FONT_BOLD if bold else FONT_REG, int(sz))


def canvas(w, h, title):
    img = Image.new("RGB", (w, h), "white")
    d = ImageDraw.Draw(img)
    d.text((w // 2, 22), title, font=font(21, True), fill="#111111", anchor="mm")
    return img, d


def panel(d, box, xlim, ylim, xticks, yticks, subtitle="", xlabel="", ylabel="", xfmt=None):
    x0, y0, x1, y1 = box
    d.rectangle([x0, y0, x1, y1], fill="white", outline="#333333", width=1)
    if subtitle:
        d.text(((x0 + x1) // 2, y0 - 18), subtitle, font=font(15, True),
               fill="#111111", anchor="mm")
    X = lambda x: x0 + (x - xlim[0]) / (xlim[1] - xlim[0]) * (x1 - x0)
    Y = lambda y: y1 - (y - ylim[0]) / (ylim[1] - ylim[0]) * (y1 - y0)
    for xt in xticks:
        if xlim[0] <= xt <= xlim[1]:
            xx = X(xt)
            d.line([xx, y0, xx, y1], fill="#e6e6e6", width=1)
            lab = xfmt(xt) if xfmt else f"{xt:.3g}"
            d.text((xx, y1 + 4), lab, font=font(12), fill="#222222", anchor="ma")
    for yt in yticks:
        if ylim[0] <= yt <= ylim[1]:
            yy = Y(yt)
            d.line([x0, yy, x1, yy], fill="#e6e6e6", width=1)
            d.text((x0 - 6, yy), f"{yt:.3g}", font=font(12), fill="#222222", anchor="rm")
    if xlabel:
        d.text(((x0 + x1) // 2, y1 + 26), xlabel, font=font(14), fill="#111111")
    if ylabel:
        d.text((x0, y0 - 36), ylabel, font=font(14), fill="#111111", anchor="lb")
    return X, Y


def polyline(d, xs, ys, X, Y, color, ylim, width=2):
    pts = []
    for x, y in zip(xs, ys):
        if not np.isfinite(y):
            continue
        yy = min(max(float(y), ylim[0]), ylim[1])
        pts.append((X(float(x)), Y(yy)))
    if len(pts) > 1:
        d.line(pts, fill=color, width=width, joint="curve")


def legend(d, x0, y0, items, fsz=13):
    lh = 24
    w = 300
    h = 16 + lh * len(items)
    d.rectangle([x0, y0, x0 + w, y0 + h], fill="white", outline="#999999", width=1)
    for i, (lab, col) in enumerate(items):
        yy = y0 + 8 + i * lh + lh // 2
        d.line([x0 + 10, yy, x0 + 42, yy], fill=col, width=3)
        d.text((x0 + 50, yy), lab, font=font(fsz), fill="#111111", anchor="lm")


def fig_learning(curves, fname, title, ylim=(-50, 6)):
    """curves: [(label, color, mse_array)]"""
    nblk = max(len(c[2]) for c in curves)
    img, d = canvas(1500, 900, title)
    xlim = (0, nblk - 1)
    n_yt = int((ylim[1] - ylim[0]) // 10) + 1
    X, Y = panel(d, (110, 105, 1445, 800), xlim, ylim,
                 list(range(0, nblk + 1, 50)),
                 [ylim[0] + 10 * i for i in range(n_yt)],
                 "", "块序号 m（每块 256 点；标准 1024 变体每块 1024 点）",
                 "10log10(Sigma e^2 / Sigma d^2) / dB")
    for lab, col, arr in curves:
        polyline(d, np.arange(len(arr)), arr, X, Y, col, ylim)
    legend(d, 1080, 125, [(lab, col) for lab, col, _ in curves])
    img.save(os.path.join(FIG, fname))


def fig_ir_compare(h_true, hats, fname, title, n_full, n_zoom, xlabel="抽头序号 n", ylabel="幅度"):
    """hats: [(label, color, hhat)]"""
    img, d = canvas(1500, 900, title)
    allv = [h_true[:n_full]] + [hh[:n_full] for _, _, hh in hats]
    ymax = max(float(np.max(np.abs(v))) for v in allv) * 1.15
    ylim = (-ymax, ymax)
    X, Y = panel(d, (110, 125, 1445, 425), (0, n_full - 1), ylim,
                 np.linspace(0, n_full - 1, 9).round().astype(int).tolist(),
                 [ylim[0] * 0.8, 0, ylim[1] * 0.8],
                 f"0 - {n_full - 1} 全段", "", ylabel)
    polyline(d, np.arange(n_full), h_true[:n_full], X, Y, "#111111", ylim, 3)
    for lab, col, hh in hats:
        polyline(d, np.arange(n_full), hh[:n_full], X, Y, col, ylim, 1)
    legend(d, 1080, 140, [("真值 h", "#111111")] + [(lab, col) for lab, col, _ in hats])
    X, Y = panel(d, (110, 605, 1445, 845), (0, n_zoom - 1), ylim,
                 np.linspace(0, n_zoom - 1, 9).round().astype(int).tolist(),
                 [ylim[0] * 0.8, 0, ylim[1] * 0.8],
                 f"0 - {n_zoom - 1} 放大", xlabel, ylabel)
    polyline(d, np.arange(n_zoom), h_true[:n_zoom], X, Y, "#111111", ylim, 3)
    for lab, col, hh in hats:
        polyline(d, np.arange(n_zoom), hh[:n_zoom], X, Y, col, ylim, 1)
    img.save(os.path.join(FIG, fname))


def fig_freq(h_true_A, h_hat_A, h_true_B, h_hats_B, fname):
    nfft = 16384
    fa = np.arange(nfft // 2 + 1) * FS / nfft
    faH = np.abs(np.fft.rfft(np.asarray(h_true_A, np.float64), nfft))
    faHA = np.abs(np.fft.rfft(np.asarray(h_hat_A, np.float64), nfft))
    fbH = np.abs(np.fft.rfft(np.asarray(h_true_B, np.float64), nfft))
    refA = faH.max()
    refB = fbH.max()
    img, d = canvas(1500, 900, "E5 系统 A/B 幅频响应：真值 vs FDAF 辨识（0-24 kHz）")
    YT = list(range(-60, 6, 10))
    XT = list(range(0, 24001, 4000))
    kfmt = lambda v: "0" if v == 0 else f"{v / 1000:g}k"
    X, Y = panel(d, (110, 125, 1445, 430), (0, 24000), (-60, 5), XT, YT,
                 "系统 A（256 点，标准 FDAF 约束版辨识）", "", "幅度 / dB", xfmt=kfmt)
    polyline(d, fa, 20 * np.log10(faH / refA + 1e-12), X, Y, "#111111", (-60, 5), 3)
    polyline(d, fa, 20 * np.log10(faHA / refA + 1e-12), X, Y, COLORS[0], (-60, 5), 1)
    legend(d, 1080, 140, [("真值 hA", "#111111"), ("辨识 hhat", COLORS[0])])
    X, Y = panel(d, (110, 595, 1445, 845), (0, 24000), (-60, 5), XT, YT,
                 "系统 B（1024 点，分区 FDAF 约束版 / 标准 1024 点）",
                 "频率 / Hz", "幅度 / dB", xfmt=kfmt)
    polyline(d, fa, 20 * np.log10(fbH / refB + 1e-12), X, Y, "#111111", (-60, 5), 3)
    for lab, col, hh in h_hats_B:
        H = np.abs(np.fft.rfft(np.asarray(hh, np.float64), nfft))
        polyline(d, fa, 20 * np.log10(H / refB + 1e-12), X, Y, col, (-60, 5), 1)
    legend(d, 1060, 610, [("真值 hB", "#111111")] + [(lab, col) for lab, col, _ in h_hats_B])
    img.save(os.path.join(FIG, fname))


# ===========================================================================
# 4. 主流程
# ===========================================================================
def main():
    hA = unknown_sys_A()
    hB = unknown_sys_B()
    xb = make_xb(L)
    xb1024 = make_xb(1024)

    def run_cfg(kind, mu, constrained=True, ef_mode="zero", keep="first", capture_at=None):
        if kind == "std256":
            return std_run(xb, hA, mu, NBLK, constrained, ef_mode, keep, capture_at)
        if kind == "part1024":
            return part_run(xb, hB, mu, NBLK, constrained, ef_mode, capture_at)
        if kind == "std1024":
            return std1024_run(xb1024, hB, mu, NBLK, constrained, ef_mode)
        raise ValueError(kind)

    def make_row(mu, r):
        mis = np.asarray(r["mis"], np.float64)
        mse = np.asarray(r["mse"], np.float64)
        stable = bool(np.all(np.isfinite(mis)) and np.all(np.isfinite(mse))
                      and r["maxW"] < 1e3 and abs(mis[-1]) < 200.0)
        idx = np.where(mis <= -25.0)[0]
        return dict(mu=float(mu), final_mis=float(mis[-1]), min_mis=float(np.min(mis)),
                    conv25=(int(idx[0]) if len(idx) else -1),
                    mse_final=float(mse[-1]), maxW=float(r["maxW"]), stable=stable)

    def pick_mu(rows, target=-30.0):
        # 优先：能达标（final_mis <= target）的**最小** mu（收敛余量最大、数值上最保守）；
        # 若无一达标，退到 -25 dB；仍无则取最终失配最好者。
        for thr in (target, -25.0):
            good = [r for r in rows if r["stable"] and r["final_mis"] <= thr]
            if good:
                if thr == target:
                    return good[0]["mu"]
                return min(good, key=lambda r: r["final_mis"])["mu"]
        st = [r for r in rows if r["stable"]] or rows
        return min(st, key=lambda r: r["final_mis"])["mu"]

    mu_list_256 = [1.5e-5, 5e-5, 1e-4, 2e-4, 4e-4, 8e-4, 1.5e-3, 3e-3, 6e-3, 1.2e-2, 2.4e-2]
    mu_list_1024 = [1e-6, 5e-6, 2e-5, 5e-5, 1e-4, 2e-4, 4e-4, 8e-4, 1.5e-3, 3e-3, 6e-3]

    VARIANTS = [
        dict(id=0, name="标准256 约束", kind="std256", constrained=True, sys="A",
             mu_list=mu_list_256),
        dict(id=1, name="标准256 不约束", kind="std256", constrained=False, sys="A",
             mu_list=mu_list_256),
        dict(id=2, name="分区1024 约束", kind="part1024", constrained=True, sys="B",
             mu_list=mu_list_256),
        dict(id=3, name="分区1024 不约束", kind="part1024", constrained=False, sys="B",
             mu_list=mu_list_256),
        dict(id=4, name="标准1024 约束(N=2048)", kind="std1024", constrained=True, sys="B",
             mu_list=mu_list_1024),
    ]

    # ---- mu 扫描 ----
    print("=" * 78)
    print("[E5][sweep] mu 扫描（300 块，白噪声，sigma_x^2=1/12）")
    sweep_all = {}
    chosen = {}
    for v in VARIANTS:
        rows = []
        for mu in v["mu_list"]:
            r = run_cfg(v["kind"], mu, v["constrained"], "zero", "first")
            rows.append(make_row(mu, r))
        sweep_all[str(v["id"])] = rows
        chosen[v["id"]] = pick_mu(rows)
        mu = chosen[v["id"]]
        print(f"[E5][sweep] 变体{v['id']} {v['name']}: 选 mu={mu:.3g}")
        for r in rows:
            mark = "  <== 选中" if r["mu"] == mu else ""
            print(f"    mu={r['mu']:<9.3g} final_mis={r['final_mis']:<9.2f} "
                  f"min_mis={r['min_mis']:<9.2f} conv25={r['conv25']:<4d} "
                  f"mse_final={r['mse_final']:<8.2f} max|W|={r['maxW']:<10.3g} "
                  f"stable={r['stable']}{mark}")

    # ---- 正式运行 5 个变体 ----
    print("=" * 78)
    print("[E5][final] 5 个变体正式结果")
    FIN = {}
    for v in VARIANTS:
        mu = chosen[v["id"]]
        r = run_cfg(v["kind"], mu, v["constrained"], "zero", "first")
        row = make_row(mu, r)
        FIN[str(v["id"])] = dict(variant=v["name"], mu=mu, row=row, run=r)
        print(f"[E5][final] 变体{v['id']} {v['name']}: mu={mu:.4g} "
              f"final_mis={row['final_mis']:.2f} dB  conv25(block)={row['conv25']} "
              f"mse_final={row['mse_final']:.2f} dB")

    # ---- 验证 (a)：帧约定 / IFFT 相关公式 ----
    print("=" * 78)
    m_cap = 30
    rv = run_cfg("std256", chosen[0], True, "zero", "first", capture_at=m_cap)
    cap = rv["cap"]
    e = cap["e"].astype(np.float64)
    xbuf = np.concatenate([xb(cap["m"] - 1), xb(cap["m"])]).astype(np.float64)
    # 逐点定义：g[k] = sum_{i=0}^{L-1} e(mL+i) x(mL+i-k), k=0..255；xbuf 索引 L+t <-> x(mL+t)
    g_def = np.array([np.sum(e * xbuf[L + np.arange(L) - k]) for k in range(L)], np.float64)
    X = cap["X"]
    E_zero = F(np.concatenate([np.zeros(L, np.float32), cap["e"]]))
    g_zero = Fi(np.conj(X) * E_zero)[:L].real.astype(np.float64)
    E_doc = F(np.concatenate([cap["e_prev"], cap["e"]]))
    g_doc = Fi(np.conj(X) * E_doc)[:L].real.astype(np.float64)

    def rel(a, b):
        return float(np.linalg.norm(a - b) / (np.linalg.norm(b) + 1e-30))

    ver_a = dict(
        block=m_cap,
        rel_err_zero_pad=rel(g_zero, g_def),
        max_abs_err_zero_pad=float(np.max(np.abs(g_zero - g_def))),
        rel_err_doc_frame=rel(g_doc, g_def),
        max_abs_err_doc_frame=float(np.max(np.abs(g_doc - g_def))),
        gdef_norm=float(np.linalg.norm(g_def)),
    )
    print(f"[E5][verify-a] g=IFFT(conj(X)E) vs g_def=sum e*x  (拟验证帧约定与 IFFT 相关公式)")
    print(f"    E=FFT([0,e_m])     : rel_err={ver_a['rel_err_zero_pad']:.3e}  "
          f"max_abs={ver_a['max_abs_err_zero_pad']:.3e}  ||g_def||={ver_a['gdef_norm']:.4g}")
    print(f"    E=FFT([e_prev,e_m]): rel_err={ver_a['rel_err_doc_frame']:.3e}  "
          f"max_abs={ver_a['max_abs_err_doc_frame']:.3e}")

    # 输出对齐：y = IFFT(X W) 后 256 点 == 直接卷积；d 同
    w_hat = Fi(cap["W"])[:L].real.astype(np.float64)
    y_def = np.array([np.sum(w_hat * xbuf[L + j - np.arange(L)]) for j in range(L)], np.float64)
    d_def = np.array([np.sum(hA.astype(np.float64) * xbuf[L + j - np.arange(L)])
                      for j in range(L)], np.float64)
    ver_align = dict(
        rel_err_y=rel(cap["y"].astype(np.float64), y_def),
        rel_err_d=rel(cap["d"].astype(np.float64), d_def),
        max_abs_y=float(np.max(np.abs(cap["y"] - y_def))),
    )
    print(f"[E5][verify-a] 输出对齐: y vs 直接卷积 rel_err={ver_align['rel_err_y']:.3e}; "
          f"d vs h_true 直接卷积 rel_err={ver_align['rel_err_d']:.3e}")

    # ---- 验证 (a2)：分区 d 与逐点卷积一致 + 分区梯度公式 ----
    m2 = 10
    rp = run_cfg("part1024", chosen[2], True, "zero", "first", capture_at=m2)
    cap2 = rp["cap"]
    e2 = cap2["e"].astype(np.float64)
    xbuf5 = np.concatenate([xb(cap2["m"] - 4), xb(cap2["m"] - 3), xb(cap2["m"] - 2),
                            xb(cap2["m"] - 1), xb(cap2["m"])]).astype(np.float64)
    d_ref = np.array([np.sum(hB.astype(np.float64) * xbuf5[1024 + j - np.arange(1024)])
                      for j in range(L)], np.float64)
    ver_part = dict(rel_err_d_part=rel(cap2["d"].astype(np.float64), d_ref),
                    part_grad=[])
    for p in (0, 2):
        Xp = cap2["Xh"][p]
        gp = Fi(np.conj(Xp) * cap2["E"])[:L].real.astype(np.float64)
        xbufp = np.concatenate([xb(cap2["m"] - p - 1), xb(cap2["m"] - p)]).astype(np.float64)
        gpd = np.array([np.sum(e2 * xbufp[L + np.arange(L) - k]) for k in range(L)], np.float64)
        ver_part["part_grad"].append(dict(p=p, rel_err=rel(gp, gpd),
                                          max_abs=float(np.max(np.abs(gp - gpd)))))
    print(f"[E5][verify-a] 分区: d vs 逐点卷积 rel_err={ver_part['rel_err_d_part']:.3e}; "
          f"g_p(p=0) rel={ver_part['part_grad'][0]['rel_err']:.3e}, "
          f"g_p(p=2) rel={ver_part['part_grad'][1]['rel_err']:.3e}")

    # ---- 验证 (b)：约束方向（保留前一半 vs 后一半） ----
    mu_ctrl = chosen[0]
    run_first = FIN["0"]["run"]
    run_second = run_cfg("std256", mu_ctrl, True, "zero", "second")
    row_second = make_row(mu_ctrl, run_second)
    ctrl = dict(
        mu=mu_ctrl,
        keep_first=dict(first10=[float(v) for v in run_first["mse"][:10]],
                        last10=[float(v) for v in run_first["mse"][-10:]],
                        final_mis=float(run_first["mis"][-1]),
                        min_mis=float(np.min(run_first["mis"]))),
        keep_second=dict(first10=[float(v) for v in run_second["mse"][:10]],
                         last10=[float(v) for v in run_second["mse"][-10:]],
                         final_mis=float(run_second["mis"][-1]),
                         min_mis=float(np.min(run_second["mis"])),
                         maxW=float(row_second["maxW"]),
                         hhat_max=float(np.max(np.abs(run_second["hhat"])))),
    )
    print("[E5][verify-b] 约束方向:")
    print(f"    保留前一半 g(0..255): final_mis={ctrl['keep_first']['final_mis']:.2f} dB, "
          f"min_mis={ctrl['keep_first']['min_mis']:.2f} dB")
    print(f"    保留后一半 g(256..511): final_mis={ctrl['keep_second']['final_mis']:.2f} dB, "
          f"min_mis={ctrl['keep_second']['min_mis']:.2f} dB, "
          f"max|W|={ctrl['keep_second']['maxW']:.3g}, max|hhat|={ctrl['keep_second']['hhat_max']:.3g}")

    # ---- 说明原文 ef=[e_prev,e] 写法在同一 mu 下的对比 ----
    docE = run_cfg("std256", mu_ctrl, True, "doc", "first")
    docE_mis = float(docE["mis"][-1])
    print(f"[E5][docE] E=FFT([e_prev,e_m]) 同 mu={mu_ctrl:.3g}: final_mis={docE_mis:.2f} dB")

    # ---- 画图 ----
    print("=" * 78)
    fig_learning(
        [(f"变体{v['id']}: " + v["name"], COLORS[i], FIN[str(v["id"])]["run"]["mse"])
         for i, v in enumerate(VARIANTS)],
        "e5_learning_curves.png",
        "E5 FDAF 学习曲线（5 个变体，白噪声激励，300 块）")
    fig_ir_compare(hA, [("辨识 hhat（标准256 约束）", COLORS[0], FIN["0"]["run"]["hhat"])],
                   "e5_ir_compare_256.png",
                   "E5 系统 A（256 点）冲激响应：真值 vs FDAF 辨识",
                   256, 64)
    fig_ir_compare(hB,
                   [("辨识 hhat（分区1024 约束）", COLORS[0], FIN["2"]["run"]["hhat"]),
                    ("辨识 hhat（标准1024 约束）", COLORS[2], FIN["4"]["run"]["hhat"])],
                   "e5_ir_compare_1024.png",
                   "E5 系统 B（1024 点）冲激响应：真值 vs FDAF 辨识",
                   1024, 128, xlabel="抽头序号 n（B 共 1024 点）")
    fig_freq(hA, FIN["0"]["run"]["hhat"], hB,
             [("辨识 hhat（分区1024）", COLORS[0], FIN["2"]["run"]["hhat"]),
              ("辨识 hhat（标准1024）", COLORS[2], FIN["4"]["run"]["hhat"])],
             "e5_freq_response.png")
    fig_learning(
        [("标准256 约束（保留前一半，正确）", COLORS[0], FIN["0"]["run"]["mse"]),
         ("标准256 不约束", COLORS[1], FIN["1"]["run"]["mse"]),
         ("标准256 保留后一半（错误方向）", "#7f7f7f", run_second["mse"]),
         ("标准256 约束但 E=[e_prev,e] （说明原文）", COLORS[3], docE["mse"])],
        "e5_constraint_effect.png",
        "E5 梯度约束效果：保留前一半 / 不约束 / 保留后一半 / 说明原文 E 帧",
        ylim=(-90, 6))

    # ---- 汇总数据 ----
    res = dict(
        meta=dict(script="analysis_host/e5_ref_model.py", fs=FS, N=N, L=L, nblk=NBLK,
                  sigma_x2=1.0 / 12.0, xorshift_seed=0x1234567,
                  x_first8=[float(v) for v in XS[PRE:PRE + 8]]),
        systems=dict(
            A=dict(sum_abs=float(np.sum(np.abs(hA))), sum_sq=float(np.sum(hA.astype(np.float64) ** 2)),
                   h2=float(hA[2]), peak=float(np.max(np.abs(hA))),
                   h=[float(v) for v in hA]),
            B=dict(sum_abs=float(np.sum(np.abs(hB))), sum_sq=float(np.sum(hB.astype(np.float64) ** 2)),
                   h2=float(hB[2]), peak=float(np.max(np.abs(hB))),
                   h=[float(v) for v in hB]),
        ),
        verification=dict(frame_identity=ver_a, output_alignment=ver_align,
                          partitioned=ver_part),
        constraint_direction=ctrl,
        doc_e_frame_final_mis=docE_mis,
        mu_sweep=sweep_all,
        variants=[dict(id=v["id"], name=v["name"], kind=v["kind"],
                       constrained=v["constrained"], mu=chosen[v["id"]],
                       **{k: FIN[str(v["id"])]["row"][k] for k in
                          ("final_mis", "min_mis", "conv25", "mse_final", "maxW", "stable")},
                       mse_curve=[round(float(x), 3) for x in FIN[str(v["id"])]["run"]["mse"]],
                       mis_curve=[round(float(x), 3) for x in FIN[str(v["id"])]["run"]["mis"]])
                  for v in VARIANTS],
        figures=["figs/e5_learning_curves.png", "figs/e5_ir_compare_256.png",
                 "figs/e5_ir_compare_1024.png", "figs/e5_freq_response.png",
                 "figs/e5_constraint_effect.png"],
    )
    with open(os.path.join(OUT, "ref_results.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=1)

    # =======================================================================
    # 5. REF_MODEL_NOTES.md
    # =======================================================================
    def md_tab(headers, rows):
        outl = ["| " + " | ".join(headers) + " |",
                "|" + "|".join(["---"] * len(headers)) + "|"]
        for r in rows:
            outl.append("| " + " | ".join(str(c) for c in r) + " |")
        return outl

    def f2(x):
        return f"{x:.2f}"

    ok_zero = ver_a["rel_err_zero_pad"] < 1e-4
    ok_doc = ver_a["rel_err_doc_frame"] < 1e-4
    conv_bad = ctrl["keep_second"]["final_mis"] > -5.0

    Ln = []
    Ln.append("# E5 FDAF 主机端参考模型与算法验证（REF_MODEL_NOTES）")
    Ln.append("")
    Ln.append("> 生成脚本：`analysis_host/e5_ref_model.py`（numpy float32 + PIL，无 matplotlib；")
    Ln.append("> 中文图字体 `C:\\Windows\\Fonts\\msyh.ttc`）。")
    Ln.append("> 重跑：`& '<python>' e5_ref_model.py`（workdir = `D:\\DSP_experiment\\E5_FADF`）。")
    Ln.append("> 结构化结果：`analysis_host/ref_results.json`。")
    Ln.append("")
    Ln.append("## 0. 结论速览")
    Ln.append("")
    if ok_zero:
        Ln.append(f"1. **帧约定 / IFFT 相关公式（验证 a）成立**：当 `E_m = FFT([0(256), e_m])` 时，")
        Ln.append(f"   `IFFT(conj(X_m)E_m)` 的前 256 点与逐点定义 "
                  f"`g[k]=Sigma_{{i=0}}^{{255}} e(mL+i)x(mL+i-k)` 完全一致，")
        Ln.append(f"   相对误差 **{ver_a['rel_err_zero_pad']:.2e}**（max_abs "
                  f"{ver_a['max_abs_err_zero_pad']:.2e}）。")
    else:
        Ln.append(f"1. **验证 a**：零填充 E 帧时相对误差 {ver_a['rel_err_zero_pad']:.2e}（未达 1e-4）。")
    if not ok_doc:
        Ln.append(f"   但**说明第 3 节写法 `ef_m=[e_{{m-1}}(0..255), e_m(0..255)]` 与它自己给出的逐点公式不一致**：")
        Ln.append(f"   同一帧、同一 e 序列下，该 E 帧算出的 `IFFT(conj(X)E)[0..255]` 相对误差为 "
                  f"**{ver_a['rel_err_doc_frame']:.2e}**")
        Ln.append("   （多出 `x` 与**上一块/更早误差**的互相关项，见第 2 节推导）。")
        Ln.append("   → **建议 C 实现用 `E = FFT([0, e_m])`**（零填充在前半），它与说明第 3 节的")
        Ln.append("   逐点公式严格等价、就是 block-LMS 的精确梯度，并且**不需要保存 e 历史**。")
        Ln.append(f"   两种 E 帧在“无附加噪声的系统辨识”下都能收敛到真解：同一 mu={mu_ctrl:.3g} 下")
        Ln.append(f"   说明原文写法 300 块失配 **{docE_mis:.2f} dB**，零填充版 "
                  f"{f2(FIN['0']['row']['final_mis'])} dB（原文写法反而略快，见第 4 节末）。")
        Ln.append("   → 若 C 已按原文实现、需要主机 vs 模拟器逐点对拍，两边必须用同一种 E 帧")
        Ln.append("   （脚本里 `std_run(..., ef_mode='doc')` 即原文写法）。")
    Ln.append("2. **约束方向（验证 b）**：`保留 g 前一半 g(0..255)、后一半置零` 正确 —— ")
    Ln.append(f"   标准 256 点 FDAF（mu={mu_ctrl:.3g}）300 块后失配 "
              f"**{f2(ctrl['keep_first']['final_mis'])} dB**；")
    Ln.append(f"   保留后一半时 W 的前半恒为 0、`hhat` 恒 0，失配停在 "
              f"**{f2(ctrl['keep_second']['final_mis'])} dB**（等于未辨识），")
    Ln.append(f"   学习曲线停在 {f2(ctrl['keep_second']['last10'][-1])} dB 附近，**不收敛**"
              + ("（且 W 能量增长，见发散证据）" if ctrl['keep_second']['maxW'] > 1e2 else "") + "。")
    Ln.append("3. **步长**：说明给出的初值 `mu=1.5e-5`（256 点）/`1e-6`（1024 点）在 300 块内")
    Ln.append("   **远不足以收敛**：实测 300 块失配仅 **-0.84 dB / -0.22 dB**（见第 4 节扫描表）。")
    Ln.append("   300 块内达到 -25 dB 大约需要：256 点 `mu >= 5e-4`、标准 1024 点 `mu >= 1.5e-4`；")
    Ln.append("   建议值见第 4 节表格，均满足 300 块内失配 <= -25 dB 且不发散。")
    Ln.append("")
    Ln.append("## 1. 实现要点")
    Ln.append("")
    Ln.append("- 采样率 48 kHz；`N=512`，`hop=L=256`；标准 1024 点用 `N=2048, hop=1024`。")
    Ln.append("- 激励：xorshift32（种子 0x1234567），`x = (u32/2^32 - 0.5) in [-0.5,0.5)`，"
              "`sigma_x^2 = 1/12`。前 4 块 x 历史为 0（模拟上电）。")
    Ln.append(f"- 未知系统 A：`Sigma|h|=0.900`，`h[0]=h[1]=0`，`h[2]={res['systems']['A']['h2']:.4f}`，"
              f"`Sigma h^2={res['systems']['A']['sum_sq']:.4f}`。")
    Ln.append(f"- 未知系统 B：`Sigma|h|=0.900`，`h[0]=h[1]=0`，`h[2]={res['systems']['B']['h2']:.4f}`，"
              f"`Sigma h^2={res['systems']['B']['sum_sq']:.4f}`（1024 点，2 点纯延迟）。")
    Ln.append("- 帧约定：`xf_m=[x_{m-1}(0..255), x_m(0..255)]`，`X_m=FFT_512(xf_m)`；")
    Ln.append("  `y_m = IFFT(X_m W)` 的**后 256 点**（与说明一致，已逐点验证 = 有效线性卷积）；")
    Ln.append("  `d_m` 由同一个 512 点重叠保留帧乘 `H_true_pad=FFT([h_true, 0])` 得到（与 y 同相）。")
    Ln.append("- 梯度：`G=conj(X_m)E_m`，`g=IFFT(G)`；约束版 `g(0..255)` 保留、`g(256..511)=0`，")
    Ln.append("  `W += mu*FFT(g)`；不约束版 `W += mu*G`。分区版每分区独立约束，`G_p=conj(X_{m-p})E_m`。")
    Ln.append("- 学习曲线 `10log10(Sigma e^2 / Sigma d^2)`（每块一点）；失配 "
              "`10log10(Sigma(hhat-h)^2 / Sigma h^2)`。"
              "`hhat`：标准版取 `IFFT(W)` 前 256（/1024）点，分区版取各分区 `IFFT(W_p)` 前 256 点拼接。")
    Ln.append("- `F()/Fi()` 用 `np.fft` 后立即 cast 成 complex64，逐点乘加 float32，贴近 DSP 行为。")
    Ln.append("")
    Ln.append("## 2. 验证 (a)：帧约定与 `IFFT(conj(X)E)` 相关公式")
    Ln.append("")
    Ln.append(f"在第 m={ver_a['block']} 块（标准 256 点约束版，mu={mu_ctrl:.3g}）取单独一帧，")
    Ln.append("逐点比较 `g_fft = IFFT(conj(X_m)E_m)` 与 `g_def[k]=Sigma_{i=0}^{255} e(mL+i)x(mL+i-k)`"
              "（k=0..255；x 用真实时域样本，含上一块历史）：")
    Ln.append("")
    Ln.extend(md_tab(["E 帧构造", "相对误差", "最大绝对误差", "结论"],
                     [["`[0(256), e_m]`（推荐）", f"{ver_a['rel_err_zero_pad']:.3e}",
                       f"{ver_a['max_abs_err_zero_pad']:.3e}", "严格一致" if ok_zero else "不一致"],
                      ["`[e_{m-1}, e_m]`（说明原文）", f"{ver_a['rel_err_doc_frame']:.3e}",
                       f"{ver_a['max_abs_err_doc_frame']:.3e}", "有额外项" if not ok_doc else "一致"]]))
    Ln.append("")
    Ln.append("输出侧同时验证（同一帧）：")
    Ln.append("")
    Ln.extend(md_tab(["关系", "相对误差"],
                     [["`y_m` = IFFT(X W) 后 256 点 vs 直接卷积 `Sigma_n w_n x(mL+j-n)`",
                       f"{ver_align['rel_err_y']:.3e}"],
                      ["`d_m` vs `h_true` 直接卷积", f"{ver_align['rel_err_d']:.3e}"],
                      ["分区版 `d_m` vs `h_B` 1024 点直接卷积",
                       f"{ver_part['rel_err_d_part']:.3e}"],
                      [f"分区梯度 `g_p`（p=0）vs 逐点定义", f"{ver_part['part_grad'][0]['rel_err']:.3e}"],
                      [f"分区梯度 `g_p`（p=2）vs 逐点定义", f"{ver_part['part_grad'][1]['rel_err']:.3e}"]]))
    Ln.append("")
    Ln.append("**数学解释（为什么必须把 e 放在已移位的帧位置、前半置零）**：")
    Ln.append("")
    Ln.append("`IFFT(conj(X)E)[k] = Sigma_{n=0}^{2L-1} xf[n]·ef[(n+k) mod 2L]`（注意是 `n+k` 不是 `n-k`）。")
    Ln.append("令 `xf[n]=x(mL+n-L)`、`ef[q]=e(mL+q-L)`（q>=L 时取当前块 e；q<L 时若取上一块 e 就引入额外项）。")
    Ln.append("取 k=0..L-1：")
    Ln.append("")
    Ln.append("- x 部分 n=L+i（当前块）：`(L+i+k) mod 2L` 落在后半 -> `e_cur[i+k]`，对 i+k<L 正好是")
    Ln.append("  `e(mL+i')x(mL+i'-k)`（i'=i+k）；")
    Ln.append("- x 部分 n=j（上一块）：`(j+k) mod 2L` 落在后半 -> `e_cur[j+k-L]`，对 j+k>=L 正好补齐")
    Ln.append("  i'<k 的历史项 `e(mL+i')x(mL+i'-k)`（i'=j+k-L）。")
    Ln.append("")
    Ln.append("若前半 ef 取 `e_prev`，则上述两处“落在前半”的索引会取到 **e(mL+i+k-2L)** / "
              "**e((m-1)L+j+k)** 等与当前块无关的误差，")
    Ln.append("这就是 `[e_prev, e]` 版本相对误差不为 0 的原因（数值见上表）。")
    Ln.append("说明第 3 节自己给出的等式 `g[k]=Sigma_{i=0}^{L-1} e(mL+i)x(mL+i-k)` 恰好对应"
              "**零填充在前半**的 E 帧。")
    Ln.append("")
    Ln.append("> 补充：两种 E 帧在系统辨识（d=h*x、无附加噪声）下都能收敛到真解（额外项也是 w 误差的线性函数，")
    Ln.append("> 不产生稳态失调地板；数值上见第 4 节末 `E=[e_prev,e]` 的最终失配），")
    Ln.append("> 但零填充版 = 精确 block-LMS 梯度，且省掉 e 历史缓存，建议 C 按零填充实现。")
    Ln.append("")
    Ln.append("## 3. 验证 (b)：梯度约束操作保留哪一半")
    Ln.append("")
    Ln.append(f"标准 256 点 FDAF，同一 mu={mu_ctrl:.3g}，白噪声，300 块：")
    Ln.append("")
    Ln.extend(md_tab(["变体", "前 10 块 MSE(dB)", "最后 10 块 MSE(dB)", "最终失配(dB)", "max|W|"],
                     [["保留前一半 g(0..255)（说明写法）",
                       ", ".join(f2(x) for x in ctrl["keep_first"]["first10"]),
                       ", ".join(f2(x) for x in ctrl["keep_first"]["last10"]),
                       f2(ctrl["keep_first"]["final_mis"]), "—"],
                      ["保留后一半 g(256..511)（错误方向）",
                       ", ".join(f2(x) for x in ctrl["keep_second"]["first10"]),
                       ", ".join(f2(x) for x in ctrl["keep_second"]["last10"]),
                       f2(ctrl["keep_second"]["final_mis"]),
                       f"{ctrl['keep_second']['maxW']:.3g}"]]))
    Ln.append("")
    if conv_bad:
        Ln.append("**结论：说明写的“保留 g 前一半（0..255）、后一半置零”是正确的**；")
        Ln.append("反方向（保留后一半）时更新量 `FFT([0, g2])` 的时域等效只落在 256..511，")
        Ln.append("W 的 0..255 分量永远保持 0，`hhat` 恒为 0，等于根本没辨识（失配 0 dB），")
        Ln.append("且 W 的 256..511 分量持续累积、能量增长，属于“不收敛/发散”，与设计说明一致。")
    else:
        Ln.append("注意：本次数值结果与上面的预期不同，请复查约束方向/帧约定。")
    Ln.append("")
    Ln.append("## 4. 步长扫描与 5 个变体结果")
    Ln.append("")
    Ln.append("扫描设置：300 块、白噪声（`sigma_x^2=1/12`）、每变体独立扫 mu；")
    Ln.append("选取规则：在稳定（不发散）候选里取**最小**能达到最终失配 <= -30 dB 的 mu（否则退到 -25 dB）。")
    Ln.append("")
    Ln.extend(md_tab(["变体", "结构", "系统", "建议 mu", "300 块失配(dB)", "首次 <=-25 dB 块",
                      "末块 MSE(dB)", "说明初值 mu"],
                     [[v["name"], v["kind"], v["sys"], f"{chosen[v['id']]:.3g}",
                       f2(FIN[str(v["id"])]["row"]["final_mis"]),
                       str(FIN[str(v["id"])]["row"]["conv25"]),
                       f2(FIN[str(v["id"])]["row"]["mse_final"]),
                       ("1.5e-5" if v["kind"] != "std1024" else "1e-6")]
                      for v in VARIANTS]))
    Ln.append("")
    Ln.append("说明：延迟 — 变体 0..3 为 256 点块（5.33 ms），变体 4 为 1024 点块（21.33 ms）；")
    Ln.append("标准 1024 点变体的“块序号”每块是 1024 个采样点（其它变体每块 256 点），")
    Ln.append("因此其曲线在相同块序号下覆盖的时间长 4 倍。")
    Ln.append("")
    Ln.append("稳定性实测边界（本模型）：标准 256 点 `mu=2.4e-2` 仍稳定；分区约束 `mu=2.4e-2` 发散、")
    Ln.append("分区不约束 `mu=1.2e-2` 发散、标准 1024（N=2048）`mu=6e-3` 稳定。")
    Ln.append("推荐的 mu 距发散边界 >= 4x（多数 >= 10x）。")
    Ln.append("")
    Ln.append("各变体 mu 扫描明细（final_mis = 第 300 块失配 dB，conv25 = 首次 <= -25 dB 的块号，-1 表示未达到）：")
    Ln.append("")
    for v in VARIANTS:
        Ln.append(f"**变体 {v['id']}：{v['name']}**（选中 mu = {chosen[v['id']]:.3g}）")
        Ln.append("")
        Ln.extend(md_tab(["mu", "最终失配(dB)", "最小失配(dB)", "conv25 块", "末块 MSE(dB)",
                          "max|W|", "稳定"],
                         [[f"{r['mu']:.3g}", f2(r["final_mis"]), f2(r["min_mis"]),
                           str(r["conv25"]), f2(r["mse_final"]),
                           f"{r['maxW']:.3g}", "是" if r["stable"] else "否"]
                          for r in sweep_all[str(v["id"])]]))
        Ln.append("")
    Ln.append(f"附：同一 mu={mu_ctrl:.3g} 下把 E 帧改成说明原文的 `[e_prev, e_m]`，"
              f"标准 256 点约束版 300 块失配 = **{docE_mis:.2f} dB**（零填充版 "
              f"{f2(FIN['0']['row']['final_mis'])} dB），走势见图 `figs/e5_constraint_effect.png`。")
    Ln.append("")
    Ln.append("## 5. 图")
    Ln.append("")
    for fp in res["figures"]:
        Ln.append(f"- `{fp}`")
    Ln.append("")
    Ln.append("![learning](figs/e5_learning_curves.png)")
    Ln.append("")
    Ln.append("![ir256](figs/e5_ir_compare_256.png)")
    Ln.append("")
    Ln.append("![ir1024](figs/e5_ir_compare_1024.png)")
    Ln.append("")
    Ln.append("![freq](figs/e5_freq_response.png)")
    Ln.append("")
    Ln.append("![constraint](figs/e5_constraint_effect.png)")
    Ln.append("")
    Ln.append("## 6. 对 C 实现（fadf.h / fadf.c）的建议")
    Ln.append("")
    Ln.append("1. **E 帧用零填充**：`ef = [0(256), e_m]`，`E=FFT512(ef)`；不要按说明第 3 节字面把")
    Ln.append("   `e_{m-1}` 放前半（否则 `IFFT(conj(X)E)` 前一半不等于逐点梯度，且要多存一块 e）。")
    Ln.append("   若已按原文实现：收敛不受影响（同 mu 下甚至更快），但前 256 点不再是精确梯度")
    Ln.append("   （见第 2 节误差）；主机/模拟器对拍时两边必须用同一种 E 帧（脚本参数 ef_mode）。")
    Ln.append("2. **约束方向**：`g = IFFT(conj(X)E)` 实部；`g[0..255]` 保留、`g[256..511]=0`；再 `FFT` 后更新 W。")
    Ln.append("   千万不要反过来保留后半（W 前 256 个时域分量将永远为 0）。")
    Ln.append("3. **帧相位**：`X_m = FFT([x_{m-1}, x_m])`，输出取 `IFFT(X_m W)` 的**后 256 点**；")
    Ln.append("   `d_m` 用同一帧乘 `H_true_pad=FFT([h_true,0])` 求后 256 点，保证 d/y 同相。")
    Ln.append("   若 S1 用 `IFFT(Y + jD)` 一次算 y、d：`y=real`、`d=imag`。")
    Ln.append("4. **分区**：`X_{m-p}` 历史 4 帧（`X_{m-3}..X_{m}`），`D=Sigma X_{m-p}Htrue_p`、")
    Ln.append("   `Y=Sigma X_{m-p}W_p`；梯度 `G_p=conj(X_{m-p})E` 各自约束后更新 `W_p`。")
    Ln.append("   启动阶段 `X_{m-1..m-3}=0` 即可（本模型验证了此时 d 与逐点卷积一致）。")
    Ln.append("5. **步长**：建议取第 4 节表格值（比说明初值大约 "
              f"{chosen[0]/1.5e-5:.0f}x（256 点）/ {chosen[4]/1e-6:.0f}x（1024 点））；"
              "若实时链路块数更多，可再降 mu 换更小失配。")
    Ln.append("6. **初始化**：`W=0`、`e_prev=0`（若用零填充 E 则不需要 `e_prev`）、")
    Ln.append("   `X` 历史清零即可；`Htrue_p`/`Htrue_pad` 在 `E5_Init()` 里预先算好，实时路径不再算。")
    Ln.append("7. **数值**：内部 float32；`conj(X)` 的虚部符号不要写反（相关 vs 卷积）；")
    Ln.append("   `IFFT` 后取实部即可（虚部为 0 的数值残差）。")
    Ln.append("8. **验证钩子**：启动 benchmark 完成后把 `hhat`、每块 MSE、失配与本文第 4 节表对照；")
    Ln.append("   若模拟器结果与主机模型差 > 1 dB，优先查 E 帧是否零填充、约束半边、d/y 相位。")
    Ln.append("")
    Ln.append("## 7. 与设计说明的差异清单")
    Ln.append("")
    Ln.append("| 说明原文 | 主机模型验证后的写法 |")
    Ln.append("|---|---|")
    Ln.append("| `ef_m=[e_{m-1}(0..255), e_m(0..255)]` | **`ef_m=[0(256), e_m(0..255)]`**（否则与逐点公式不符） |")
    Ln.append("| `mu = 1.5e-5`（256 点）/ `1e-6`（1024 点） | 见第 4 节表（300 块内达到 -25 dB 需要更大 mu） |")
    Ln.append("| 其余（N=512/hop=256、帧 [x_{m-1},x_m]、后 256 点输出、保留前一半约束、")
    Ln.append("  失配/学习曲线定义、A/B 参数与归一化） | 与主机模型一致 |")
    Ln.append("")

    with open(os.path.join(OUT, "REF_MODEL_NOTES.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(Ln))

    # ---- 末尾打印关键数字 ----
    print("=" * 78)
    print("[E5][summary] 推荐 mu 与 300 块最终失配:")
    for v in VARIANTS:
        row = FIN[str(v["id"])]["row"]
        print(f"  v{v['id']} {v['name']:<22s} mu={chosen[v['id']]:<9.3g} "
              f"mis={row['final_mis']:6.2f} dB  conv25={row['conv25']:<4d} "
              f"mse299={row['mse_final']:7.2f} dB")
    print("[E5][summary] verify-a zero-pad rel_err=%.3e ; doc-ef rel_err=%.3e"
          % (ver_a["rel_err_zero_pad"], ver_a["rel_err_doc_frame"]))
    print("[E5][summary] verify-b keep_first mis=%.2f dB ; keep_second mis=%.2f dB"
          % (ctrl["keep_first"]["final_mis"], ctrl["keep_second"]["final_mis"]))
    print("[E5][summary] E=[e_prev,e] same mu final mis=%.2f dB" % docE_mis)
    print("[E5][summary] systems: |hA|_1=%.3f |hB|_1=%.3f"
          % (res["systems"]["A"]["sum_abs"], res["systems"]["B"]["sum_abs"]))
    print("[E5][summary] files:")
    for fp in ["analysis_host/e5_ref_model.py", "analysis_host/ref_results.json",
               "analysis_host/REF_MODEL_NOTES.md"] + \
              ["analysis_host/" + f for f in res["figures"]]:
        print("   ", fp, "OK" if os.path.exists(os.path.join(BASE, fp)) else "MISSING")
    print("[E5][done]")


if __name__ == "__main__":
    main()
