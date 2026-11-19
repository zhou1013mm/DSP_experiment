/* =====================================================================
 *  my_fft.h  ——  32 点基-2 FFT / IFFT 的 C 语言实现
 * ---------------------------------------------------------------------
 *  对应 MATLAB:  class10 FFT/my_fft.m  的   X = my_fft(x)
 *
 *  运算方法与 my_fft.m 完全一致（基-2、时间抽取、3 层嵌套循环）：
 *
 *     1) 位反序：my_fft.m 开头那段 reshape 预处理
 *                  X = [X(:,1:2^(M-n)); X(:,2^(M-n)+1:2^(M-n+1))]
 *                在 C 里就是“把下标按二进制位倒过来”的置换，见
 *                my_fft_bitrev()。N = 4 时：[x0 x1 x2 x3] -> [x0 x2 x1 x3]
 *
 *     2) 加权：  X(a, b+2^(m-1)) *= WN^((b-1)*2^(M-m))
 *                WN = exp(-j*2*pi/N)，即 WN^k = cos(2*pi*k/N) - j*sin(2*pi*k/N)
 *
 *     3) 蝶形：  X(a,b)             = X(a,b) + X(a,b+half)
 *                X(a,b+half)        = X(a,b)_old - X(a,b+half)_weighted
 *                （my_fft.m 第 46~47 行写成
 *                  X(a,b) = X(a,b) + X2;  X2 = X(a,b) - 2*X2;
 *                  因为 X(a,b) 此时已是 X1_old + X2，所以第二条算出来
 *                  正好是 X1_old - X2，与上面的写法等价）
 *
 *  3 层循环的顺序也和 my_fft.m 一样： m(级) -> a(组) -> b(组内的结)。
 *  输入先做位反序、再做时间抽取的蝶形，输出 X[k] 为自然顺序 k = 0..31。
 *
 * ---------------------------------------------------------------------
 *  关于“复数运算”：
 *    C 语言（尤其 CCES 的编译器）不保证有 <complex.h>，所以这里自己定义
 *       typedef struct { float re, im; } my_complex;
 *    并给出加减乘、共轭、取模这些基本运算：
 *       加法：(a+jb) + (c+jd) = (a+c) + j(b+d)
 *       减法：(a+jb) - (c+jd) = (a-c) + j(b-d)
 *       乘法：(a+jb) * (c+jd) = (ac-bd) + j(ad+bc)
 *       共轭：conj(a+jb) = a - jb
 *       模  ：|a+jb| = sqrt(a^2 + b^2)
 *
 * ---------------------------------------------------------------------
 *  用法示例（在 adc_dac_playback.c 里）：
 *
 *      float      x[MY_FFT_N];        // 32 点时域数据
 *      my_complex X[MY_FFT_N];        // 32 点复数谱
 *      float      mag[MY_FFT_N / 2];  // k = 0..15 的幅度
 *
 *      my_fft(x, X);                  // 实数输入 -> 复数谱（自然顺序）
 *      my_fft_mag(X, mag);            // 取前 N/2 点幅度（实序列谱对称）
 *      my_ifft(X, x);                 // 逆变换，取实部还原时域序列
 *
 *  性能要点（课件要求“常用变量和数组可以预先初始化”）：
 *    - 旋转因子表 MY_FFT_W 是编译期常量，运行时只做乘加，不调用
 *      sin/cos，也不做任何初始化判断；
 *    - 全部函数是 static inline，只在本编译单元可见，不会重复定义。
 *
 *  与库函数对比（课件要求 3）：
 *    SHARC 上若 <trans.h> 可用，可对同一组数据分别调用 rfftN(N, x, ...)
 *    和 my_fft()，比较结果与 CCOUNT 计数；本实现的点数/顺序/缩放约定
 *    与 my_fft.m 一致（不做 1/N 归一化，IFFT 才乘 1/N）。
 * ===================================================================== */

#ifndef MY_FFT_H
#define MY_FFT_H

#include <math.h>

/* ---------------------------------------------------------------------
 * 0. 点数配置
 *    FFT 长度固定为 32 点：M = log2(32) = 5，旋转因子表只需 N/2 = 16 项
 *    （k = 16..31 的 WN^k = -WN^(k-16)）。
 *    若确实要做 64 点：MY_FFT_N 改 64、MY_FFT_LOG2N 改 6，并把 MY_FFT_W
 *    按 exp(-j*2*pi*k/64) 补到 32 项，然后把下面的 #error 去掉。
 * --------------------------------------------------------------------- */
#define MY_FFT_N        32
#define MY_FFT_LOG2N    5
#define MY_FFT_HALF     (MY_FFT_N / 2)

#if (MY_FFT_N != 32) || (MY_FFT_HALF != 16)
#error "my_fft.h: MY_FFT_W 是按 32 点算好的，改点数时请同步扩充旋转因子表"
#endif

/* =====================================================================
 * 1. 复数类型与基本运算
 * ===================================================================== */

typedef struct
{
    float re;   /* 实部 */
    float im;   /* 虚部 */
} my_complex;

/* 由实部、虚部构造一个复数 */
static inline my_complex mc_make(float re, float im)
{
    my_complex c;
    c.re = re;
    c.im = im;
    return c;
}

/* 加法 */
static inline my_complex mc_add(my_complex a, my_complex b)
{
    return mc_make(a.re + b.re, a.im + b.im);
}

/* 减法 */
static inline my_complex mc_sub(my_complex a, my_complex b)
{
    return mc_make(a.re - b.re, a.im - b.im);
}

/* 乘法： (a+jb)(c+jd) = (ac - bd) + j(ad + bc) */
static inline my_complex mc_mul(my_complex a, my_complex b)
{
    return mc_make((a.re * b.re) - (a.im * b.im),
                   (a.re * b.im) + (a.im * b.re));
}

/* 乘以实系数 */
static inline my_complex mc_scale(my_complex a, float k)
{
    return mc_make(a.re * k, a.im * k);
}

/* 共轭 */
static inline my_complex mc_conj(my_complex a)
{
    return mc_make(a.re, -a.im);
}

/* 模的平方 |c|^2 = re^2 + im^2（比较大小用，省一次开方） */
static inline float mc_abs2(my_complex a)
{
    return (a.re * a.re) + (a.im * a.im);
}

/* 模 |c| = sqrt(re^2 + im^2) */
static inline float mc_abs(my_complex a)
{
    return sqrtf(mc_abs2(a));
}

/* =====================================================================
 * 2. 旋转因子表  W32^k = exp(-j*2*pi*k/32)，k = 0..15
 *    对应 my_fft.m 里的 WN = exp(-j*2*pi/N) 及其幂次
 * ===================================================================== */

static const my_complex MY_FFT_W[MY_FFT_HALF] =
{
    { +1.000000000e+00f, +0.000000000e+00f },   /* W32^ 0 = 1        */
    { +9.807852804e-01f, -1.950903220e-01f },   /* W32^ 1            */
    { +9.238795325e-01f, -3.826834324e-01f },   /* W32^ 2            */
    { +8.314696123e-01f, -5.555702330e-01f },   /* W32^ 3            */
    { +7.071067812e-01f, -7.071067812e-01f },   /* W32^ 4 = (1-j)/√2 */
    { +5.555702330e-01f, -8.314696123e-01f },   /* W32^ 5            */
    { +3.826834324e-01f, -9.238795325e-01f },   /* W32^ 6            */
    { +1.950903220e-01f, -9.807852804e-01f },   /* W32^ 7            */
    { +0.000000000e+00f, -1.000000000e+00f },   /* W32^ 8 = -j       */
    { -1.950903220e-01f, -9.807852804e-01f },   /* W32^ 9            */
    { -3.826834324e-01f, -9.238795325e-01f },   /* W32^10            */
    { -5.555702330e-01f, -8.314696123e-01f },   /* W32^11            */
    { -7.071067812e-01f, -7.071067812e-01f },   /* W32^12            */
    { -8.314696123e-01f, -5.555702330e-01f },   /* W32^13            */
    { -9.238795325e-01f, -3.826834324e-01f },   /* W32^14            */
    { -9.807852804e-01f, -1.950903220e-01f }    /* W32^15            */
};

/* =====================================================================
 * 3. 位反序置换（对应 my_fft.m 开头的 reshape 预处理）
 *
 *    my_fft.m 是这样做的：每一轮把 1 x 2^M 的 X 折成
 *        X = [X(:,1:2^(M-n)); X(:,2^(M-n)+1:2^(M-n+1))]
 *    n = 1..M 折完以后，第 i 个位置上的就是原序列的第 bitrev(i) 个元素。
 *    C 里不需要矩阵，直接用“下标二进制位倒过来”的方式原地交换即可。
 *    N = 32 时：0,16,8,24,4,20,12,28,2,18,... 就是位反序的顺序。
 * ===================================================================== */

static inline void my_fft_bitrev(my_complex *X)
{
    int i;
    int j = 0;              /* j = i 的位反序结果，逐位递推得到 */
    int k;
    my_complex t;

    for (i = 1; i < MY_FFT_N; i++)
    {
        k = MY_FFT_N >> 1;              /* 从最高位开始找可以进位的那一位 */
        while (j & k)
        {
            j &= ~k;
            k >>= 1;
        }
        j |= k;

        if (i < j)                      /* 只交换一次，避免换回去 */
        {
            t    = X[i];
            X[i] = X[j];
            X[j] = t;
        }
    }
}

/* =====================================================================
 * 4. 蝶形运算（my_fft.m 里 m / a / b 三层循环的直译）
 *
 *    第 m 级：每组 2^m 个点，前半 half = 2^(m-1) 个、后半 half 个。
 *      - 后半第 b 个点先乘旋转因子 WN^(b * 2^(M-m))；
 *        注意表里相邻项的间隔是 2^(M-m) = MY_FFT_N >> m（step）。
 *      - 然后前半 + 后半、前半 - 后半（蝶形）。
 *    本级一共有 2^(M-m) 组，所以 a 的步长就是整个组的长度 2^m。
 * ===================================================================== */

static inline void my_fft_butterfly(my_complex *X)
{
    int m;              /* 级数 1..M */
    int a;              /* 组起始下标 */
    int b;              /* 组内的“结”序号 0..2^(m-1)-1 */
    int half;           /* 2^(m-1)：每组的前/后半长度 */
    int span;           /* 2^m    ：每组总长度 */
    int step;           /* 2^(M-m)：旋转因子表的下标步进 */
    my_complex w;
    my_complex t;

    for (m = 1; m <= MY_FFT_LOG2N; m++)
    {
        half = 1 << (m - 1);
        span = 1 << m;
        step = MY_FFT_N >> m;

        for (a = 0; a < MY_FFT_N; a += span)            /* 2^(M-m) 个组   */
        {
            for (b = 0; b < half; b++)                  /* 每组的 2^(m-1) 个结 */
            {
                /* 旋转因子 WN^(b * 2^(M-m))                                  */
                /* （MATLAB 里 b 从 1 开始写的是 WN^((b-1)*2^(M-m))，同一个东西）*/
                w = MY_FFT_W[b * step];

                t = mc_mul(X[a + b + half], w);         /* 后半乘权重 */

                X[a + b + half] = mc_sub(X[a + b], t);  /* 蝶形：差 */
                X[a + b]        = mc_add(X[a + b], t);  /* 蝶形：和 */
            }
        }
    }
}

/* =====================================================================
 * 5. 32 点 FFT
 * ===================================================================== */

/* 复数输入，就地变换：X[k] = sum_n X[n] * W32^(n*k)，输出自然顺序 */
static inline void my_fft_cplx(my_complex *X)
{
    my_fft_bitrev(X);
    my_fft_butterfly(X);
}

/* 实数输入 -> 复数谱（DAC/ADC 采到的都是实序列，最常用这个入口） */
static inline void my_fft(const float *x, my_complex *X)
{
    int i;
    for (i = 0; i < MY_FFT_N; i++)
    {
        X[i].re = x[i];
        X[i].im = 0.0f;
    }
    my_fft_cplx(X);
}

/* =====================================================================
 * 6. 32 点 IFFT
 *    x[n] = (1/N) * sum_k X[k] * W32^(-n*k)
 *    用 FFT 实现：先把 X 共轭 -> FFT -> 再共轭 -> 乘 1/N
 *    （因为 conj(FFT(conj(X))) = N * ifft(X)）
 * ===================================================================== */

/* 复数输入，就地逆变换 */
static inline void my_ifft_cplx(my_complex *X)
{
    const float inv_n = 1.0f / (float)MY_FFT_N;
    int i;

    for (i = 0; i < MY_FFT_N; i++)
    {
        X[i].im = -X[i].im;             /* 共轭 */
    }

    my_fft_cplx(X);                     /* 用 FFT 算 */

    for (i = 0; i < MY_FFT_N; i++)
    {
        X[i].im = -X[i].im * inv_n;     /* 再共轭并归一化 */
        X[i].re =  X[i].re * inv_n;
    }
}

/* 复数谱 -> 实数序列（实序列的 IFFT 结果虚部理论为 0，直接取实部） */
static inline void my_ifft(const my_complex *X, float *x)
{
    my_complex tmp[MY_FFT_N];
    int i;

    for (i = 0; i < MY_FFT_N; i++)
    {
        tmp[i] = X[i];
    }
    my_ifft_cplx(tmp);

    for (i = 0; i < MY_FFT_N; i++)
    {
        x[i] = tmp[i].re;
    }
}

/* =====================================================================
 * 7. 幅度谱 |X[k]|，k = 0 .. N/2-1
 *    实序列的频谱共轭对称，只要前 N/2 点就够了（Nyquist 点 k = N/2
 *    也在 X[N/2] 里，若需要可单独取 mc_abs(X[N/2])）。
 *    这里不除 N，归一化交给调用者（例如要画 dB 图时再除）。
 * ===================================================================== */

static inline void my_fft_mag(const my_complex *X, float *mag)
{
    int k;
    for (k = 0; k < MY_FFT_HALF; k++)
    {
        mag[k] = mc_abs(X[k]);
    }
}

#endif /* MY_FFT_H */
