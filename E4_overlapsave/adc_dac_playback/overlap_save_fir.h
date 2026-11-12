/* =====================================================================
 *  overlap_save_fir.h  ——  E4：实时重叠保留法 FIR（48 kHz 演示版）
 * ---------------------------------------------------------------------
 *  对应 PPT：任务 4「基于重叠保留法的实时滤波」
 *
 *  本文件同时实现两套滤波器，方便一次 debug 采数据、一次导出分析：
 *    1) 255 阶（256 个系数）普通重叠保留法 FIR；
 *    2) 1023 阶（1024 个系数）均匀分区重叠保留法 FIR，
 *       分成 4 个 256 点的子滤波器，对应 PPT 第 5 页的公式：
 *
 *           y(n) = sum_{p=0}^{3} sum_{i=0}^{255} h(i+256p) x(n-i-256p)
 *
 *       频域实现：Y_m = sum_{p=0}^{3} X_{m-p} * H_p
 *       其中 H_p 是第 p 段 256 个系数补零到 512 点后的 FFT。
 *
 *  ---------------------------------------------------------------------
 *  重叠保留法的参数：
 *      L    = 256   （滤波器长度，即 255 阶）
 *      N    = 512   （帧长 = 2L）
 *      hop  = 256   （帧移 = L，相邻帧重叠 50%）
 *
 *  每帧：
 *      frame[0..255]   = 上一帧的 256 个输入（历史）
 *      frame[256..511] = 本帧新来的 256 个输入
 *      X = FFT(frame)
 *      Y = X * H
 *      y = IFFT(Y)
 *      丢掉 y[0..255]，保留 y[256..511] 作为本帧输出
 *
 *  这和 PPT 第 4 页 L=32、N=64、帧移 32 的例子是同一套，
 *  只是把 L 从 32 放大到 256。
 *
 *  ---------------------------------------------------------------------
 *  采样率说明：
 *      按你的选择，本工程直接在 48 kHz 上演示重叠保留法（ADC/DAC 也都
 *      是 48 kHz），没有做 48k -> 16k 的抽取。PPT 里的 16 kHz 版本只需
 *      在 E4 前面加一级 3:1 抽取、后面加一级 1:3 插值，重叠保留的主体
 *      代码不用改；如果以后要上 16 kHz，把 E4_FS_HZ 改成 16000 并补上
 *      重采样即可。
 *
 *  ---------------------------------------------------------------------
 *  实现约定：
 *      - 滤波器系数和 512 点旋转因子都在 e4_init() 里一次性算好，
 *        运行时不调用 sin/cos；
 *      - 全部用 static 数组/结构体，不用 malloc；
 *      - 只处理第 0 通道，处理完复制到 4 个通道（和 E2 的做法一致）。
 * ===================================================================== */

#ifndef OVERLAP_SAVE_FIR_H
#define OVERLAP_SAVE_FIR_H

#include <math.h>
#include <string.h>
#include "my_fft.h"          /* my_complex、mc_add、mc_sub、mc_mul */

/* ---------------- 基本参数 ---------------- */
#define E4_FS_HZ            (48000.0f)   /* 本工程实际采样率              */
#define E4_FC_HZ            (2000.0f)    /* 低通截止频率，和 E1 一致       */

#define E4_FFT_N            (512)        /* N = 2L                        */
#define E4_FFT_LOG2N        (9)          /* log2(512)                     */
#define E4_HOP              (256)        /* 帧移 = L                      */

#define E4_TAPS1            (256)        /* 255 阶 = 256 个系数           */
#define E4_TAPS2            (1024)       /* 1023 阶 = 1024 个系数         */
#define E4_PART             (4)          /* 1023 阶分成 4 个区            */
#define E4_PART_TAPS        (256)        /* 每个区 256 个系数             */

#define E4_FRAMES_PER_CALL  (64)         /* 每次回调每通道 64 帧          */

#define E4_PI               (3.14159265358979323846f)

#if (E4_FFT_N != (2 * E4_HOP))
#error "E4: overlap-save requires N = 2 * hop"
#endif
#if (E4_FFT_N != (1 << E4_FFT_LOG2N))
#error "E4: E4_FFT_LOG2N does not match E4_FFT_N"
#endif
#if (E4_TAPS2 != (E4_PART * E4_PART_TAPS))
#error "E4: E4_TAPS2 must equal E4_PART * E4_PART_TAPS"
#endif
#if ((E4_PART & (E4_PART - 1)) != 0)
#error "E4: E4_PART must be a power of two"
#endif
#if ((E4_HOP % E4_FRAMES_PER_CALL) != 0)
#error "E4: E4_HOP must be an integer multiple of E4_FRAMES_PER_CALL"
#endif

/* =====================================================================
 *  状态结构体
 * ===================================================================== */

/* 255 阶普通重叠保留法 + 逐点 FIR 对比 */
typedef struct
{
    float       h[E4_TAPS1];                 /* 滤波器系数                 */
    float       x_hist[E4_HOP];              /* 上一帧的 256 个输入        */
    float       d_hist[E4_TAPS1 - 1];        /* 逐点 FIR 的历史            */
    float       d_buf[E4_TAPS1 - 1 + E4_HOP];/* 逐点 FIR 的工作缓冲        */
    my_complex  H[E4_FFT_N];                 /* 滤波器频响（512 点）       */
} E4_OS256;

/* 1023 阶、4 分区重叠保留法 + 逐点 FIR 对比 */
typedef struct
{
    float       h[E4_TAPS2];                 /* 1024 个系数                */
    float       x_hist[E4_HOP];              /* 上一帧的 256 个输入        */
    float       d_hist[E4_TAPS2 - 1];        /* 逐点 FIR 的历史            */
    float       d_buf[E4_TAPS2 - 1 + E4_HOP];/* 逐点 FIR 的工作缓冲        */
    my_complex  H[E4_PART][E4_FFT_N];        /* 4 段子滤波器的频响         */
    my_complex  X_hist[E4_PART][E4_FFT_N];   /* 最近 4 帧输入的频谱        */
    int         x_idx;                       /* X_hist 的写指针            */
} E4_OS1024;

/* 两套算法共用的 FFT/IFFT 工作区 */
typedef struct
{
    float       frame[E4_FFT_N];             /* 时域帧                     */
    my_complex  X[E4_FFT_N];                 /* 频域                       */
    my_complex  Y[E4_FFT_N];                 /* 频域乘积/逆变换            */
} E4_WORK;

/* =====================================================================
 *  内部变量
 * ===================================================================== */

static my_complex e4_tw[E4_FFT_N / 2];       /* W_512^k, k = 0..255        */
static int        e4_tw_ready = 0;

/* =====================================================================
 *  滤波器设计：Hamming 窗低通
 *      h[n] = sinc(2*fc/fs*(n-M/2)) * (0.54 - 0.46*cos(2*pi*n/M))
 *      再归一化到 sum(h) = 1（直流增益 1）
 *  等价于 MATLAB 的 fir1(taps-1, fc/(fs/2))，只是窗函数明确写成 Hamming。
 * ===================================================================== */
static void e4_design_lpf(float *h, int taps, float fc, float fs)
{
    int   i;
    int   M = taps - 1;
    float sum = 0.0f;

    for (i = 0; i < taps; i++)
    {
        float x = (2.0f * fc / fs) * ((float)i - 0.5f * (float)M);
        float s;
        float w;

        if (fabsf(x) < 1.0e-6f)
        {
            s = 1.0f;
        }
        else
        {
            s = sinf(E4_PI * x) / (E4_PI * x);
        }

        w = 0.54f - 0.46f * cosf(2.0f * E4_PI * (float)i / (float)M);
        h[i] = s * w;
        sum += h[i];
    }

    if (fabsf(sum) > 1.0e-12f)
    {
        for (i = 0; i < taps; i++)
        {
            h[i] /= sum;
        }
    }
}

/* =====================================================================
 *  512 点基-2 FFT / IFFT
 *  旋转因子表 e4_tw 在 e4_tw_init() 里一次性算好。
 * ===================================================================== */
static void e4_tw_init(void)
{
    int k;

    if (e4_tw_ready)
    {
        return;
    }

    for (k = 0; k < (E4_FFT_N / 2); k++)
    {
        float ang = -2.0f * E4_PI * (float)k / (float)E4_FFT_N;
        e4_tw[k].re = cosf(ang);
        e4_tw[k].im = sinf(ang);
    }

    e4_tw_ready = 1;
}

/* 复数输入、就地 FFT，输出自然顺序 */
static void e4_fft_cplx(my_complex *X)
{
    int        i;
    int        j;
    int        k;
    int        m;
    int        half;
    int        span;
    int        step;
    my_complex t;
    my_complex w;

    /* ---- 位反序置换 ---- */
    j = 0;
    for (i = 1; i < E4_FFT_N; i++)
    {
        k = E4_FFT_N >> 1;
        while (j & k)
        {
            j &= ~k;
            k >>= 1;
        }
        j |= k;

        if (i < j)
        {
            t    = X[i];
            X[i] = X[j];
            X[j] = t;
        }
    }

    /* ---- 9 级蝶形 ---- */
    for (m = 1; m <= E4_FFT_LOG2N; m++)
    {
        half = 1 << (m - 1);
        span = 1 << m;
        step = E4_FFT_N >> m;

        for (i = 0; i < E4_FFT_N; i += span)
        {
            for (j = 0; j < half; j++)
            {
                w = e4_tw[j * step];
                t = mc_mul(X[i + j + half], w);

                X[i + j + half] = mc_sub(X[i + j], t);
                X[i + j]        = mc_add(X[i + j], t);
            }
        }
    }
}

/* 实数输入 -> 复数谱 */
static void e4_fft_real(const float *x, my_complex *X)
{
    int i;

    for (i = 0; i < E4_FFT_N; i++)
    {
        X[i].re = x[i];
        X[i].im = 0.0f;
    }

    e4_fft_cplx(X);
}

/* 复数输入、就地 IFFT（含 1/N 归一化） */
static void e4_ifft_cplx(my_complex *X)
{
    const float inv_n = 1.0f / (float)E4_FFT_N;
    int i;

    for (i = 0; i < E4_FFT_N; i++)
    {
        X[i].im = -X[i].im;
    }

    e4_fft_cplx(X);

    for (i = 0; i < E4_FFT_N; i++)
    {
        X[i].re =  X[i].re * inv_n;
        X[i].im = -X[i].im * inv_n;
    }
}

/* =====================================================================
 *  初始化：设计 256/1024 点低通，算好 H 和 H_p，清历史
 * ===================================================================== */
static void e4_init(E4_OS256 *s1, E4_OS1024 *s2, E4_WORK *w)
{
    int i;
    int p;

    e4_tw_init();

    /* ---- 设计两组低通系数 ---- */
    e4_design_lpf(s1->h, E4_TAPS1, E4_FC_HZ, E4_FS_HZ);
    e4_design_lpf(s2->h, E4_TAPS2, E4_FC_HZ, E4_FS_HZ);

    /* ---- 255 阶滤波器的频响 H ---- */
    for (i = 0; i < E4_FFT_N; i++)
    {
        w->frame[i] = (i < E4_TAPS1) ? s1->h[i] : 0.0f;
    }
    e4_fft_real(w->frame, s1->H);

    /* ---- 1023 阶滤波器分成 4 段，每段补零到 512 点后算 H_p ---- */
    for (p = 0; p < E4_PART; p++)
    {
        for (i = 0; i < E4_FFT_N; i++)
        {
            w->frame[i] = (i < E4_PART_TAPS)
                        ? s2->h[p * E4_PART_TAPS + i]
                        : 0.0f;
        }
        e4_fft_real(w->frame, s2->H[p]);
    }

    /* ---- 清历史 ---- */
    memset(s1->x_hist, 0, sizeof(s1->x_hist));
    memset(s1->d_hist, 0, sizeof(s1->d_hist));
    memset(s1->d_buf,  0, sizeof(s1->d_buf));

    memset(s2->x_hist, 0, sizeof(s2->x_hist));
    memset(s2->d_hist, 0, sizeof(s2->d_hist));
    memset(s2->d_buf,  0, sizeof(s2->d_buf));
    memset(s2->X_hist, 0, sizeof(s2->X_hist));
    s2->x_idx = 0;
}

/* =====================================================================
 *  255 阶重叠保留法
 *      输入 x[0..255] 是本帧新来的 256 点；
 *      输出 y[0..255] 是本帧的 256 点滤波结果。
 * ===================================================================== */
static void e4_os256_process(E4_OS256 *s, E4_WORK *w, const float *x, float *y)
{
    int i;

    for (i = 0; i < E4_HOP; i++)
    {
        w->frame[i]         = s->x_hist[i];
        w->frame[E4_HOP + i] = x[i];
    }

    e4_fft_real(w->frame, w->X);

    for (i = 0; i < E4_FFT_N; i++)
    {
        w->Y[i] = mc_mul(w->X[i], s->H[i]);
    }

    e4_ifft_cplx(w->Y);

    for (i = 0; i < E4_HOP; i++)
    {
        y[i] = w->Y[E4_HOP + i].re;      /* 丢掉前 256 点，保留后 256 点 */
    }

    memcpy(s->x_hist, x, sizeof(s->x_hist));
}

/* =====================================================================
 *  1023 阶、4 分区重叠保留法
 *      Y_m = X_m*H_0 + X_{m-1}*H_1 + X_{m-2}*H_2 + X_{m-3}*H_3
 * ===================================================================== */
static void e4_os1024_process(E4_OS1024 *s, E4_WORK *w, const float *x, float *y)
{
    int i;
    int k;
    int p;

    for (i = 0; i < E4_HOP; i++)
    {
        w->frame[i]          = s->x_hist[i];
        w->frame[E4_HOP + i] = x[i];
    }

    e4_fft_real(w->frame, w->X);

    for (k = 0; k < E4_FFT_N; k++)
    {
        w->Y[k].re = 0.0f;
        w->Y[k].im = 0.0f;
    }

    for (p = 0; p < E4_PART; p++)
    {
        const my_complex *src;

        if (p == 0)
        {
            src = w->X;                  /* 当前帧 */
        }
        else
        {
            src = s->X_hist[(s->x_idx - p + E4_PART) & (E4_PART - 1)];
        }

        for (k = 0; k < E4_FFT_N; k++)
        {
            w->Y[k] = mc_add(w->Y[k], mc_mul(src[k], s->H[p][k]));
        }
    }

    e4_ifft_cplx(w->Y);

    for (i = 0; i < E4_HOP; i++)
    {
        y[i] = w->Y[E4_HOP + i].re;
    }

    memcpy(s->X_hist[s->x_idx], w->X, sizeof(w->X));
    s->x_idx = (s->x_idx + 1) & (E4_PART - 1);

    memcpy(s->x_hist, x, sizeof(s->x_hist));
}

/* =====================================================================
 *  逐点 FIR，用来和重叠保留法对比结果/耗时
 *      d_buf = [历史 taps-1 点][本帧 n 点]
 *      y[i]  = sum_{k=0}^{taps-1} h[k] * d_buf[i + taps-1 - k]
 * ===================================================================== */
static void e4_direct_fir(const float *h, int taps,
                          float *d_hist, float *d_buf,
                          const float *x, float *y, int n)
{
    int i;
    int k;

    memcpy(d_buf, d_hist, (size_t)(taps - 1) * sizeof(float));
    memcpy(d_buf + taps - 1, x, (size_t)n * sizeof(float));

    for (i = 0; i < n; i++)
    {
        float acc = 0.0f;
        for (k = 0; k < taps; k++)
        {
            acc += h[k] * d_buf[i + taps - 1 - k];
        }
        y[i] = acc;
    }

    memcpy(d_hist, d_buf + n, (size_t)(taps - 1) * sizeof(float));
}

#endif /* OVERLAP_SAVE_FIR_H */
