/* =====================================================================
 *  fadf.h  ——  E5：频域自适应滤波（FDAF）系统辨识
 * ---------------------------------------------------------------------
 *  对应 PPT：任务 5「基于频域自适应滤波的系统辨识」
 *
 *  实验框架（和 E1–E4 一样跑在 SC589 + ADC/DAC 回环上）：
 *    - 48 kHz，每次音频回调每通道 64 帧（1.333 ms），4 次回调 = 256 点
 *      一个处理块（5.333 ms）；
 *    - 激励 x(n) 由 DSP 内部产生（宽带白噪声）；
 *    - “未知”线性系统用预设参数的 FIR 系数 h_true 描述，期望信号
 *      d(n) = h_true * x(n) 在 DSP 内部用重叠保留法精确算出；
 *    - 自适应滤波器输出 y(n)，误差 e(n) = d(n) - y(n)；
 *    - 收敛后由频域权 W 反变换得到时域冲激响应估计 h_hat(n)。
 *
 *  =====================================================================
 *  一、标准 FDAF（基础，帧 N=512、帧移 L=256、滤波器 256 点）
 * ---------------------------------------------------------------------
 *  第 m 块（每块新来 L=256 点）：
 *
 *      xf_m = [ x_{m-1}(0..L-1) , x_m(0..L-1) ]     重叠保留帧（512 点）
 *      ef_m = [ 0 (L 个零)      , e_m(0..L-1) ]     误差帧（前半零填充）
 *
 *      X_m = FFT(xf_m)                              (512 点复数谱)
 *      y_m = IFFT( X_m .* W ) 的后 L 点              (有效线性卷积)
 *      e_m = d_m - y_m
 *      E_m = FFT(ef_m)
 *
 *  误差帧为什么前半补零：IFFT(conj(X)E)[k] = sum_n xf[n] ef[(n+k) mod N]，
 *  只有把本块 e 放在后半（前半补零）时，前 L 点才严格等于逐点梯度
 *  g[k] = sum_i e(mL+i) x(mL+i-k)。主机端参考模型
 *  （analysis_host/e5_ref_model.py）用逐点定义对拍验证过：
 *  相对误差 8.6e-8；若把 e_{m-1} 放在前半，则多出与旧误差的互相关项。
 *
 *  梯度（对 MSE 求导）：
 *      g[k] = sum_i e(mL+i) * x(mL+i-k) ,  k = 0..L-1
 *  频域一次算出来：
 *      g = IFFT( conj(X_m) .* E_m )
 *  其中 k = 0..L-1 是“物理冲激响应”部分，k = L..N-1 是循环混叠部分。
 *
 *  **梯度约束操作（Constraint Operation）**：
 *      g(0..L-1) 保留，g(L..N-1) 强制置零，再做 FFT 得到约束后的梯度 Gc。
 *      W <- W + mu * Gc
 *  不做约束的版本直接 W <- W + mu * conj(X).*E（只用于对比）。
 *
 *  =====================================================================
 *  二、分区 FDAF（进阶，总长 1024 点 = 4 个 256 点分区）
 * ---------------------------------------------------------------------
 *      总冲激响应 h = [h_0 | h_1 | h_2 | h_3]，每个分区 256 点。
 *      Y_m = sum_{p=0}^{3} X_{m-p} .* W_p      （X_{m-p} 是最近 4 帧的谱）
 *      每个分区独立做梯度约束：
 *          G_p = conj(X_{m-p}) .* E_m
 *          g_p = IFFT(G_p); g_p(L..N-1)=0; Gc_p = FFT(g_p)
 *          W_p <- W_p + mu * Gc_p
 *      算法延迟 = 一个块 = 256 点 = 5.333 ms。
 *      作为对比：标准 FDAF 要覆盖 1024 点必须 N=2048、hop=1024，
 *      延迟 = 1024 点 = 21.33 ms，每块 5 个 2048 点变换。
 *
 *  =====================================================================
 *  三、运算量优化：两个实数序列打成一个复数变换
 * ---------------------------------------------------------------------
 *      y_m 与 d_m 都是实序列，它们的频谱 Y、D 可以合成一个复数谱
 *      Y + jD，一次 IFFT 同时得到 y + j d（实部/虚部）。这样每块省一次
 *      IFFT，是 E5 能在 Debug（-O0）下实时跑的关键。
 *
 *  =====================================================================
 *  四、未知系统（预设参数）
 * ---------------------------------------------------------------------
 *  系统 A（基础，256 点，带 1.8 kHz 阻尼共振的短混响）：
 *      h_A(n) = 0.70*delta(n-2) + 0.35*exp(-(n-2)/40)*cos(2*pi*1800*(n-2)/fs)
 *  系统 B（进阶，1024 点，类“房间”混响）：
 *      h_B(n) = 0.60*delta(n-2) + exp(-(n-2)/150)*r(n)   （r 为固定种子伪随机）
 *  两者都归一化到 sum|h| = 0.9。
 *
 *  本文件只放“纯算法 + 纯 DSP 状态”，不依赖 CCES 驱动，便于在 SHARC
 *  指令集模拟器里单独验证（见 sim/e5_sim_test.c）。
 * ===================================================================== */

#ifndef FADF_H
#define FADF_H

#include <math.h>
#include <string.h>
#include <stdint.h>
#include "my_fft.h"          /* my_complex、mc_* 基本复数运算 */

/* ---------------------------------------------------------------------
 * 0. 参数
 * --------------------------------------------------------------------- */
#define E5_FS_HZ            (48000.0f)

#define E5_HOP              (256)        /* 块长 L / 帧移 = 256           */
#define E5_FFT_N            (512)        /* 帧长 N = 2L = 512             */
#define E5_LOG2N            (9)          /* log2(512)                     */

#define E5_PART             (4)          /* 分区数                        */
#define E5_PART_TAPS        (256)        /* 每个分区 256 点               */
#define E5_TAPS_STD         (256)        /* 基础模式自适应滤波器长度      */
#define E5_TAPS_PART        (1024)       /* 进阶模式总长度                */

#define E5_FFT_N_BIG        (2048)       /* 对比用的标准 1024 点：N=2048  */
#define E5_LOG2N_BIG        (11)
#define E5_TAPS_BIG         (1024)

#define E5_MAX_TAPS         (1024)
#define E5_FRAMES_PER_CALL  (64)         /* 每次回调每通道 64 帧          */

#define E5_PI               (3.14159265358979323846f)

/* 白噪声 / 未知系统伪随机种子 */
#define E5_NOISE_SEED       (0x1234567u)
#define E5_SYS_B_SEED       (0x5A5A1234u)

/* 自适应步长
 * 主机端参考模型（analysis_host/e5_ref_model.py，300 块白噪声）扫出来的值：
 *   标准 256 点（约束/不约束）   mu = 8e-4
 *   分区 1024 点（约束）         mu = 8e-4
 *   分区 1024 点（不约束）       mu = 6e-3（不约束版慢、稳态差一些）
 *   标准 1024 点（N=2048，约束） mu = 2e-4
 * 稳定边界：标准 256 点约 9e-2，上述取值有 10x 以上余量。 */
#define E5_MU_STD           (8.0e-4f)    /* 标准 256 点 / 分区约束版      */
#define E5_MU_PART          (8.0e-4f)    /* 分区 1024 点，约束版          */
#define E5_MU_PART_U        (6.0e-3f)    /* 分区 1024 点，不约束版        */
#define E5_MU_BIG           (2.0e-4f)    /* 标准 1024 点（N=2048）        */

/* 启动 benchmark 每个变体跑的块数 */
#ifndef E5_BENCH_BLOCKS
#define E5_BENCH_BLOCKS     (300)
#endif

/* 输入白噪声幅度：x 均匀分布 [-E5_X_AMP, +E5_X_AMP) */
#define E5_X_AMP            (0.5f)

/* 未知系统归一化目标：sum|h| = E5_SYS_L1 */
#define E5_SYS_L1           (0.9f)

#if (E5_FFT_N != (2 * E5_HOP))
#error "E5: overlap-save requires E5_FFT_N = 2 * E5_HOP"
#endif
#if (E5_FFT_N != (1 << E5_LOG2N))
#error "E5: E5_LOG2N does not match E5_FFT_N"
#endif
#if (E5_FFT_N_BIG != (1 << E5_LOG2N_BIG))
#error "E5: E5_LOG2N_BIG does not match E5_FFT_N_BIG"
#endif
#if (E5_TAPS_PART != (E5_PART * E5_PART_TAPS))
#error "E5: E5_TAPS_PART must equal E5_PART * E5_PART_TAPS"
#endif

/* ---------------------------------------------------------------------
 * 1. 旋转因子表：W_Nbig^k = exp(-j*2*pi*k/Nbig)，k = 0..Nbig/2-1
 *    512 点 FFT 用 stride = Nbig/512 = 4 取子表，2048 点用 stride = 1。
 *    初始化时一次算好，运行时不再调用 sin/cos。
 * --------------------------------------------------------------------- */
static my_complex e5_tw[E5_FFT_N_BIG / 2];
static int        e5_tw_ready = 0;

static void e5_tw_init(void)
{
    int k;

    if (e5_tw_ready)
    {
        return;
    }

    for (k = 0; k < (E5_FFT_N_BIG / 2); k++)
    {
        float ang = -2.0f * E5_PI * (float)k / (float)E5_FFT_N_BIG;
        e5_tw[k].re = cosf(ang);
        e5_tw[k].im = sinf(ang);
    }

    e5_tw_ready = 1;
}

/* ---------------------------------------------------------------------
 * 2. 通用基-2 复数 FFT / IFFT
 *    n 取 512 或 2048；就地变换，输入自然序、输出自然序。
 * --------------------------------------------------------------------- */
static void e5_fft_n(my_complex *X, int n, int log2n)
{
    int        i;
    int        j;
    int        k;
    int        m;
    int        half;
    int        span;
    int        step;
    int        stride;
    my_complex t;

    /* ---- 位反序 ---- */
    j = 0;
    for (i = 1; i < n; i++)
    {
        k = n >> 1;
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

    /* ---- log2(n) 级蝶形 ---- */
    stride = E5_FFT_N_BIG / n;
    for (m = 1; m <= log2n; m++)
    {
        half = 1 << (m - 1);
        span = 1 << m;
        step = (n >> m) * stride;

        for (i = 0; i < n; i += span)
        {
            for (j = 0; j < half; j++)
            {
                int   a  = i + j;
                int   b  = a + half;
                float wr = e5_tw[j * step].re;
                float wi = e5_tw[j * step].im;
                float xr = X[b].re;
                float xi = X[b].im;
                float tr = (xr * wr) - (xi * wi);
                float ti = (xr * wi) + (xi * wr);
                float ar = X[a].re;
                float ai = X[a].im;

                X[b].re = ar - tr;
                X[b].im = ai - ti;
                X[a].re = ar + tr;
                X[a].im = ai + ti;
            }
        }
    }
}

/* 就地 IFFT（含 1/n 归一化） */
static void e5_ifft_n(my_complex *X, int n, int log2n)
{
    float inv = 1.0f / (float)n;
    int   i;

    for (i = 0; i < n; i++)
    {
        X[i].im = -X[i].im;
    }

    e5_fft_n(X, n, log2n);

    for (i = 0; i < n; i++)
    {
        X[i].re =  X[i].re * inv;
        X[i].im = -X[i].im * inv;
    }
}

/* 实数输入 -> 复数谱（拷贝到复数数组后做 FFT） */
static void e5_fft_real_n(const float *x, my_complex *X, int n, int log2n)
{
    int i;

    for (i = 0; i < n; i++)
    {
        X[i].re = x[i];
        X[i].im = 0.0f;
    }

    e5_fft_n(X, n, log2n);
}

/* 两个实序列的频谱 A、B -> 一次 IFFT 得到 a + j*b
 *   Z[k] = A[k] + j*B[k] = (A.re - B.im) + j*(A.im + B.re)
 *   IFFT(Z) 的实部就是 a，虚部就是 b（A、B 都是实序列的谱） */
static void e5_ifft2_n(const my_complex *A, const my_complex *B,
                       my_complex *T, float *a, float *b,
                       int n, int log2n)
{
    int i;

    for (i = 0; i < n; i++)
    {
        T[i].re = A[i].re - B[i].im;
        T[i].im = A[i].im + B[i].re;
    }

    e5_ifft_n(T, n, log2n);

    for (i = 0; i < n; i++)
    {
        a[i] = T[i].re;
        b[i] = T[i].im;
    }
}

/* ---------------------------------------------------------------------
 * 3. 白噪声激励（xorshift32，确定性、可复现）
 * --------------------------------------------------------------------- */
static void e5_noise_init(uint32_t *state, uint32_t seed)
{
    *state = (seed != 0u) ? seed : E5_NOISE_SEED;
}

/* 生成 n 点白噪声：均匀分布 [-E5_X_AMP, +E5_X_AMP) */
static void e5_noise_block(float *x, int n, uint32_t *state)
{
    uint32_t s = *state;
    int      i;

    for (i = 0; i < n; i++)
    {
        s ^= (s << 13);
        s ^= (s >> 17);
        s ^= (s << 5);
        x[i] = ((float)(s >> 8) * (1.0f / 16777216.0f) - 0.5f) * (2.0f * E5_X_AMP);
    }

    *state = s;
}

/* 有符号伪随机 [-1,1)，用于未知系统 B 的混响尾 */
static float e5_rand_sym(uint32_t *state)
{
    uint32_t s = *state;
    s ^= (s << 13);
    s ^= (s >> 17);
    s ^= (s << 5);
    *state = s;
    return ((float)(s >> 8) * (1.0f / 16777216.0f) - 0.5f) * 2.0f;
}

/* ---------------------------------------------------------------------
 * 4. 预设的“未知”线性系统
 * --------------------------------------------------------------------- */
static void e5_normalize_l1(float *h, int taps, float target)
{
    float sum = 0.0f;
    int   i;

    for (i = 0; i < taps; i++)
    {
        sum += fabsf(h[i]);
    }

    if (sum > 1.0e-12f)
    {
        float g = target / sum;
        for (i = 0; i < taps; i++)
        {
            h[i] *= g;
        }
    }
}

/* 系统 A：256 点，主径 + 1.8 kHz 阻尼共振 */
static void e5_design_sys_a(float *hA)
{
    int i;

    for (i = 0; i < E5_TAPS_STD; i++)
    {
        float v = 0.0f;

        if (i >= 2)
        {
            int m = i - 2;
            v = 0.35f * expf(-(float)m / 40.0f)
                       * cosf(2.0f * E5_PI * 1800.0f * (float)m / E5_FS_HZ);
        }
        if (i == 2)
        {
            v += 0.70f;
        }

        hA[i] = v;
    }

    e5_normalize_l1(hA, E5_TAPS_STD, E5_SYS_L1);
}

/* 系统 B：1024 点，主径 + 指数衰减随机混响尾 */
static void e5_design_sys_b(float *hB)
{
    uint32_t seed = E5_SYS_B_SEED;
    int      i;

    for (i = 0; i < E5_TAPS_PART; i++)
    {
        float v;

        if (i >= 2)
        {
            int m = i - 2;
            v = expf(-(float)m / 150.0f) * e5_rand_sym(&seed);
        }
        else
        {
            v = 0.0f;
        }

        if (i == 2)
        {
            v += 0.60f;
        }

        hB[i] = v;
    }

    e5_normalize_l1(hB, E5_TAPS_PART, E5_SYS_L1);
}

/* 把滤波器系数补零到 512 点算频响，第 p 段放在 Hspec[p*N .. p*N+N-1]。
 *   taps_per_part = 0 时表示整段只有 1 个分区（基础模式）。 */
static void e5_system_spectra(const float *h, int taps, int npart, int taps_per_part,
                              my_complex *Hspec, my_complex *T)
{
    int p;
    int i;

    for (p = 0; p < npart; p++)
    {
        for (i = 0; i < E5_FFT_N; i++)
        {
            T[i].re = 0.0f;
            T[i].im = 0.0f;
        }

        for (i = 0; i < taps_per_part; i++)
        {
            int idx = p * taps_per_part + i;
            if (idx < taps)
            {
                T[i].re = h[idx];
            }
        }

        e5_fft_n(T, E5_FFT_N, E5_LOG2N);

        for (i = 0; i < E5_FFT_N; i++)
        {
            Hspec[p * E5_FFT_N + i] = T[i];
        }
    }
}

/* 由频域权 W（npart 个分区）求时域冲激响应估计
 *   hhat[0 .. npart*tapsPerPart-1] = 各分区 IFFT 的前 tapsPerPart 点 */
static void e5_weights_to_taps(const my_complex *W, int npart, int taps_per_part,
                               float *hhat, my_complex *T)
{
    int p;
    int i;

    for (p = 0; p < npart; p++)
    {
        for (i = 0; i < E5_FFT_N; i++)
        {
            T[i] = W[p * E5_FFT_N + i];
        }

        e5_ifft_n(T, E5_FFT_N, E5_LOG2N);

        for (i = 0; i < taps_per_part; i++)
        {
            hhat[p * taps_per_part + i] = T[i].re;
        }
    }
}

/* 同样的函数，但 1024 点标准版（N=2048） */
static void e5_weights_to_taps_big(const my_complex *W, int taps, float *hhat, my_complex *T)
{
    int i;

    for (i = 0; i < E5_FFT_N_BIG; i++)
    {
        T[i] = W[i];
    }

    e5_ifft_n(T, E5_FFT_N_BIG, E5_LOG2N_BIG);

    for (i = 0; i < taps; i++)
    {
        hhat[i] = T[i].re;
    }
}

/* ---------------------------------------------------------------------
 * 5. 状态 / 工作区
 * --------------------------------------------------------------------- */

/* 所有变体共用的大工作区（按最大点数 2048 分配，512 点只用到前 512 个） */
typedef struct
{
    my_complex X[E5_FFT_N_BIG];      /* 当前帧输入谱                     */
    my_complex E[E5_FFT_N_BIG];      /* 当前帧误差谱                     */
    my_complex T[E5_FFT_N_BIG];      /* 通用临时（打包谱 / 梯度 / IFFT） */
    float      frame[E5_FFT_N_BIG];  /* 时域重叠保留帧                   */
    float      dbuf[E5_FFT_N_BIG];   /* d(n) 整帧（取后 L 点用）          */
    float      ebuf[E5_MAX_TAPS];    /* 本块 e(n)（L 点）                 */
    float      xnew[E5_MAX_TAPS];    /* 本块新来 x(n)（L 点）             */
} E5_WORK;

/* 实时/benchmark 共用：标准(1 分区)与分区(4 分区)两种 FDAF，N=512 */
typedef struct
{
    my_complex W[E5_PART][E5_FFT_N];      /* 频域权，np 个分区            */
    my_complex Xhist[E5_PART][E5_FFT_N];  /* 最近 np 帧输入谱             */
    int        np;                        /* 1 = 标准 256 点；4 = 分区    */
    float      xhist[E5_HOP];             /* 上一块 x                     */
    float      xcur[E5_HOP];              /* 本块 x 的副本（供流逝输出）  */
    float      xout[E5_HOP];              /* 正在送往 DAC/采集的块        */
    float      xnew[E5_HOP];              /* 正在累积的下一块 x           */
} E5_RT;

/* 对比用：标准 1024 点 FDAF（N=2048、hop=1024），只用于启动 benchmark */
typedef struct
{
    my_complex W[E5_FFT_N_BIG];
    my_complex Hspec[E5_FFT_N_BIG];   /* 未知系统 B 补零到 2048 点的频响 */
    float      xhist[E5_TAPS_BIG];
    float      xnew[E5_TAPS_BIG];
} E5_BIG;

/* 两者不会同时用（benchmark 顺序跑完再进实时），用 union 省内存 */
typedef union
{
    E5_RT  rt;
    E5_BIG big;
} E5_STATE;

/* ---------------------------------------------------------------------
 * 6. 梯度约束 + 权更新（标准/分区通用，np 个分区）
 *
 *    输入：Xhist、E、ebuf
 *    输出：更新后的 W、xhist、ehist
 * --------------------------------------------------------------------- */
static void e5_block_update(E5_RT *s, E5_WORK *w, float mu, int constrain)
{
    int p;
    int i;
    int k;

    if (constrain)
    {
        /* ---- G_p = conj(Xhist[p]) .* E；IFFT -> 截断 -> FFT -> 更新 ---- */
        for (p = 0; p < s->np; p++)
        {
            for (k = 0; k < E5_FFT_N; k++)
            {
                float xr = s->Xhist[p][k].re;
                float xi = s->Xhist[p][k].im;
                float er = w->E[k].re;
                float ei = w->E[k].im;

                /* conj(X)*E = (xr - j xi)(er + j ei) */
                w->T[k].re = (xr * er) + (xi * ei);
                w->T[k].im = (xr * ei) - (xi * er);
            }

            e5_ifft_n(w->T, E5_FFT_N, E5_LOG2N);

            /* 约束：只保留前一半（物理冲激响应），后一半（混叠）置零 */
            for (i = E5_HOP; i < E5_FFT_N; i++)
            {
                w->T[i].re = 0.0f;
                w->T[i].im = 0.0f;
            }
            for (i = 0; i < E5_HOP; i++)
            {
                w->T[i].im = 0.0f;
            }

            e5_fft_n(w->T, E5_FFT_N, E5_LOG2N);

            for (k = 0; k < E5_FFT_N; k++)
            {
                s->W[p][k].re += mu * w->T[k].re;
                s->W[p][k].im += mu * w->T[k].im;
            }
        }
    }
    else
    {
        /* ---- 不约束版：直接频域更新 W += mu * conj(X)*E ---- */
        for (p = 0; p < s->np; p++)
        {
            for (k = 0; k < E5_FFT_N; k++)
            {
                float xr = s->Xhist[p][k].re;
                float xi = s->Xhist[p][k].im;
                float er = w->E[k].re;
                float ei = w->E[k].im;

                s->W[p][k].re += mu * ((xr * er) + (xi * ei));
                s->W[p][k].im += mu * ((xr * ei) - (xi * er));
            }
        }
    }

    /* ---- 历史更新 ----
     * 注意 x 要用 s->xcur（本块 x 的副本），不能用 s->xnew：
     * 实时链路里 S2 阶段执行时，s->xnew 已经在累积下一块的样点了。
     * 误差帧是 [0, e_m] 零填充，不需要保存 e 历史。 */
    memcpy(s->xhist, s->xcur, sizeof(s->xhist));
}

/* 复位状态 */
static void e5_rt_reset(E5_RT *s, int np)
{
    memset(s, 0, sizeof(*s));
    s->np = np;
}

/* 复位 2048 点标准版状态（保留 Hspec，它由调用者算好后填进来） */
static void e5_big_reset(E5_BIG *s)
{
    memset(s->W, 0, sizeof(s->W));
    memset(s->xhist, 0, sizeof(s->xhist));
    memset(s->xnew, 0, sizeof(s->xnew));
}

/* 计算一块的 MSE（dB）：10*log10(sum e^2 / sum d^2) */
static float e5_block_mse_db(const float *e, const float *d, int n)
{
    float se = 0.0f;
    float sd = 0.0f;
    int   i;

    for (i = 0; i < n; i++)
    {
        se += e[i] * e[i];
        sd += d[i] * d[i];
    }

    if (sd < 1.0e-20f)
    {
        return 0.0f;
    }
    if (se < 1.0e-20f)
    {
        return -200.0f;
    }

    return 10.0f * log10f(se / sd);
}

/* 失配（dB）：10*log10(sum(hhat-h)^2 / sum h^2) */
static float e5_misalignment_db(const float *hhat, const float *h, int taps)
{
    float se = 0.0f;
    float sh = 0.0f;
    int   i;

    for (i = 0; i < taps; i++)
    {
        float d = hhat[i] - h[i];
        se += d * d;
        sh += h[i] * h[i];
    }

    if (sh < 1.0e-20f)
    {
        return 0.0f;
    }
    if (se < 1.0e-20f)
    {
        return -200.0f;
    }

    return 10.0f * log10f(se / sh);
}

/* ---------------------------------------------------------------------
 * 7. 启动 benchmark：标准 / 分区 FDAF（N=512）跑 blocks 块
 *    benchmark 直接调用实时链路的 3 个阶段函数，保证“测到的就是实时
 *    要跑的同一段代码”。cycles 由调用者用 cycle_count 在外部测。
 * --------------------------------------------------------------------- */

/* 前置声明：实时阶段函数在文件末尾定义 */
static void e5_rt_S0(E5_RT *s, E5_WORK *w);
static void e5_rt_S1(E5_RT *s, E5_WORK *w, const my_complex *Hspec);
static void e5_rt_S2(E5_RT *s, E5_WORK *w, float mu, int constrain);

static void e5_bench_engine(E5_RT *s, E5_WORK *w,
                            const float *hTrue,
                            const my_complex *Hspec,
                            float mu, int constrain, int blocks,
                            float *hhat, float *mse_db, float *mis_db,
                            uint32_t *noise)
{
    int b;
    int i;

    for (b = 0; b < blocks; b++)
    {
        /* 产生本块白噪声，再依次走实时链路的 S0/S1/S2 */
        e5_noise_block(s->xnew, E5_HOP, noise);

        e5_rt_S0(s, w);
        e5_rt_S1(s, w, Hspec);
        e5_rt_S2(s, w, mu, constrain);

        mse_db[b] = e5_block_mse_db(w->ebuf, w->dbuf, E5_HOP);
    }

    e5_weights_to_taps(&s->W[0][0], s->np, E5_PART_TAPS, hhat, w->T);
    for (i = s->np * E5_PART_TAPS; i < E5_MAX_TAPS; i++)
    {
        hhat[i] = 0.0f;
    }
    *mis_db = e5_misalignment_db(hhat, hTrue, s->np * E5_PART_TAPS);
}

/* ---------------------------------------------------------------------
 * 8. 启动 benchmark：标准 1024 点 FDAF（N=2048、hop=1024）
 *    Hspec 由调用者先填进 s->Hspec（hB 补零到 2048 点做 FFT）。
 * --------------------------------------------------------------------- */
static void e5_bench_big(E5_BIG *s, E5_WORK *w,
                         const float *hTrue,
                         float mu, int constrain, int blocks,
                         float *hhat, float *mse_db, float *mis_db,
                         uint32_t *noise)
{
    const my_complex *Hspec = s->Hspec;
    int b;
    int i;
    int k;

    e5_big_reset(s);

    for (b = 0; b < blocks; b++)
    {
        e5_noise_block(w->xnew, E5_TAPS_BIG, noise);

        /* 组帧：N=2048，hop=1024 */
        for (i = 0; i < E5_TAPS_BIG; i++)
        {
            w->frame[i]                  = s->xhist[i];
            w->frame[E5_TAPS_BIG + i]    = w->xnew[i];
        }
        e5_fft_real_n(w->frame, w->X, E5_FFT_N_BIG, E5_LOG2N_BIG);

        /* Y = X*W，D = X*Hspec，打包一次 IFFT */
        for (k = 0; k < E5_FFT_N_BIG; k++)
        {
            float xr = w->X[k].re;
            float xi = w->X[k].im;
            float wr = s->W[k].re;
            float wi = s->W[k].im;
            float hr = Hspec[k].re;
            float hi = Hspec[k].im;
            float yr = (xr * wr) - (xi * wi);
            float yi = (xr * wi) + (xi * wr);
            float dr = (xr * hr) - (xi * hi);
            float di = (xr * hi) + (xi * hr);

            w->T[k].re = yr - di;
            w->T[k].im = yi + dr;
        }
        e5_ifft_n(w->T, E5_FFT_N_BIG, E5_LOG2N_BIG);

        for (i = 0; i < E5_TAPS_BIG; i++)
        {
            float y = w->T[E5_TAPS_BIG + i].re;
            float d = w->T[E5_TAPS_BIG + i].im;

            w->dbuf[i] = d;
            w->ebuf[i] = d - y;
        }

        /* 误差帧：前半零填充 + 后半本块 e，再 FFT */
        for (i = 0; i < E5_TAPS_BIG; i++)
        {
            w->frame[i]                  = 0.0f;
            w->frame[E5_TAPS_BIG + i]    = w->ebuf[i];
        }
        e5_fft_real_n(w->frame, w->E, E5_FFT_N_BIG, E5_LOG2N_BIG);

        /* 梯度 + 约束 + 更新 */
        if (constrain)
        {
            for (k = 0; k < E5_FFT_N_BIG; k++)
            {
                float xr = w->X[k].re;
                float xi = w->X[k].im;
                float er = w->E[k].re;
                float ei = w->E[k].im;

                w->T[k].re = (xr * er) + (xi * ei);
                w->T[k].im = (xr * ei) - (xi * er);
            }
            e5_ifft_n(w->T, E5_FFT_N_BIG, E5_LOG2N_BIG);

            for (i = E5_TAPS_BIG; i < E5_FFT_N_BIG; i++)
            {
                w->T[i].re = 0.0f;
                w->T[i].im = 0.0f;
            }
            for (i = 0; i < E5_TAPS_BIG; i++)
            {
                w->T[i].im = 0.0f;
            }

            e5_fft_n(w->T, E5_FFT_N_BIG, E5_LOG2N_BIG);

            for (k = 0; k < E5_FFT_N_BIG; k++)
            {
                s->W[k].re += mu * w->T[k].re;
                s->W[k].im += mu * w->T[k].im;
            }
        }
        else
        {
            for (k = 0; k < E5_FFT_N_BIG; k++)
            {
                float xr = w->X[k].re;
                float xi = w->X[k].im;
                float er = w->E[k].re;
                float ei = w->E[k].im;

                s->W[k].re += mu * ((xr * er) + (xi * ei));
                s->W[k].im += mu * ((xr * ei) - (xi * er));
            }
        }

        memcpy(s->xhist, w->xnew, sizeof(s->xhist));

        mse_db[b] = e5_block_mse_db(w->ebuf, w->dbuf, E5_TAPS_BIG);
    }

    e5_weights_to_taps_big(s->W, E5_TAPS_BIG, hhat, w->T);
    *mis_db = e5_misalignment_db(hhat, hTrue, E5_TAPS_BIG);
}

/* =====================================================================
 * 9. 实时链路：分阶段（每次音频回调只做 1 个阶段，保证单次回调
 *    计算量不超过 DMA ping-pong 期限 ~2.66 ms）
 *
 *    阶段号在 adc_dac_playback.c 里维护；这里给出 4 个阶段函数。
 *    Hspec：未知系统频响（np 个分区连续存放）。
 * ===================================================================== */

/* S0：组当前块的重叠保留帧，FFT -> X；必要时把 X 挪进 Xhist[0] */
static void e5_rt_S0(E5_RT *s, E5_WORK *w)
{
    int i;
    int k;
    int p;

    /* 本块 x 已在 s->xnew（由回调累积） */
    for (i = 0; i < E5_HOP; i++)
    {
        w->frame[i]          = s->xhist[i];
        w->frame[E5_HOP + i] = s->xnew[i];
    }

    e5_fft_real_n(w->frame, w->X, E5_FFT_N, E5_LOG2N);

    if (s->np > 1)
    {
        for (p = s->np - 1; p >= 1; p--)
        {
            memcpy(s->Xhist[p], s->Xhist[p - 1], sizeof(s->Xhist[0]));
        }
        for (k = 0; k < E5_FFT_N; k++)
        {
            s->Xhist[0][k] = w->X[k];
        }
    }
    else
    {
        for (k = 0; k < E5_FFT_N; k++)
        {
            s->Xhist[0][k] = w->X[k];
        }
    }

    /* 本块 x 留一份副本，供 S1 之后的“流逝输出/采集”使用 */
    memcpy(s->xcur, s->xnew, sizeof(s->xcur));
}

/* S1：算 Y、D（打包一次 IFFT）-> y、d -> e -> E */
static void e5_rt_S1(E5_RT *s, E5_WORK *w, const my_complex *Hspec)
{
    int i;
    int k;
    int p;

    for (k = 0; k < E5_FFT_N; k++)
    {
        float yr = 0.0f;
        float yi = 0.0f;
        float dr = 0.0f;
        float di = 0.0f;

        for (p = 0; p < s->np; p++)
        {
            float xr = s->Xhist[p][k].re;
            float xi = s->Xhist[p][k].im;
            float wr = s->W[p][k].re;
            float wi = s->W[p][k].im;
            float hr = Hspec[p * E5_FFT_N + k].re;
            float hi = Hspec[p * E5_FFT_N + k].im;

            yr += (xr * wr) - (xi * wi);
            yi += (xr * wi) + (xi * wr);
            dr += (xr * hr) - (xi * hi);
            di += (xr * hi) + (xi * hr);
        }

        w->T[k].re = yr - di;
        w->T[k].im = yi + dr;
    }

    e5_ifft_n(w->T, E5_FFT_N, E5_LOG2N);

    for (i = 0; i < E5_HOP; i++)
    {
        float y = w->T[E5_HOP + i].re;
        float d = w->T[E5_HOP + i].im;

        w->dbuf[i] = d;
        w->ebuf[i] = d - y;
    }

    for (i = 0; i < E5_HOP; i++)
    {
        w->frame[i]          = 0.0f;     /* 前半零填充：ef = [0, e_m] */
        w->frame[E5_HOP + i] = w->ebuf[i];
    }
    e5_fft_real_n(w->frame, w->E, E5_FFT_N, E5_LOG2N);

    /* 本块 x 的副本交给输出/采集流 */
    memcpy(s->xout, s->xcur, sizeof(s->xout));
}

/* S2：梯度约束 + 权更新 + 历史更新 */
static void e5_rt_S2(E5_RT *s, E5_WORK *w, float mu, int constrain)
{
    e5_block_update(s, w, mu, constrain);
}

#endif /* FADF_H */
