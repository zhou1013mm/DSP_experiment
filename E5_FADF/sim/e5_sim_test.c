/* =====================================================================
 *  e5_sim_test.c —— E5 FDAF 算法的 SHARC 指令集模拟器验证程序
 * ---------------------------------------------------------------------
 *  不依赖 ADC/DAC 驱动，只把 fadf.h 的算法跑在 SC589 SHARC Core1 上：
 *    1) 对 5 个变体各跑 300 块，实测 cycles/块、学习曲线、失配；
 *    2) 单独测实时链路 S0/S1/S2 三个阶段的 cycles（判断 2.66 ms 期限）；
 *    3) 结果全部写进全局数组，模拟器跑完后用 CCES_runner 导出。
 *
 *  由 sim/build_run.ps1 编译/链接/运行。导出后由
 *  analysis_host/e5_sim_analyze.py 读文件验证。
 * ===================================================================== */
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <sys/platform.h>

#ifdef DO_CYCLE_COUNTS
#undef DO_CYCLE_COUNTS
#endif
#define DO_CYCLE_COUNTS 1
#include <cycle_count.h>

#include "fadf.h"

/* 链接 app_startup.doj 需要 adi_initialize（真实工程里由 system/adi_initialize.c 提供） */
int adi_initialize(void)
{
    return 0;
}

/* 数据写完后的标记函数：CCES_runner 的 -d 用它在“命中断点”时导出内存 */
void e5_sim_done(void)
{
    /* 故意空函数；加 volatile 防止被内联优化掉 */
    volatile int x = 0;
    (void)x;
}

/* ---------- 被测算法用的数据 ---------- */
static E5_WORK  gWork;
static E5_STATE gState;

static my_complex gHA[E5_FFT_N];
static my_complex gHB[E5_PART][E5_FFT_N];
static my_complex gHBbig[E5_FFT_N_BIG];

static float gHA_t[E5_TAPS_STD];
static float gHB_t[E5_TAPS_PART];

/* ---------- 导出给 Python 的结果（volatile 防止被优化掉） ---------- */
volatile float    gSimHhat[E5_MAX_TAPS];
volatile float    gSimHtrueA[E5_TAPS_STD];
volatile float    gSimHtrueB[E5_TAPS_PART];
volatile float    gSimMSE[5 * E5_BENCH_BLOCKS];
volatile float    gSimTiming[16];
volatile uint32_t gSimDebug[16];

/* ---------- 辅助 ---------- */
static void sim_copy_hhat(const float *src, int n)
{
    int i;
    for (i = 0; i < E5_MAX_TAPS; i++)
    {
        gSimHhat[i] = (i < n) ? src[i] : 0.0f;
    }
}

int main(void)
{
    int      i;
    int      v;
    float    mis;
    uint32_t noise;
    cycle_t  c0;
    cycle_t  c1;
    float    cyc;

    /* 结果数组清零 */
    for (i = 0; i < 16; i++)
    {
        gSimTiming[i] = 0.0f;
        gSimDebug[i]  = 0u;
    }
    for (i = 0; i < (5 * E5_BENCH_BLOCKS); i++)
    {
        gSimMSE[i] = 0.0f;
    }

    /* ---- 初始化：旋转因子 + 两个未知系统 + 频响 ---- */
    e5_tw_init();
    e5_design_sys_a(gHA_t);
    e5_design_sys_b(gHB_t);

    for (i = 0; i < E5_TAPS_STD; i++)
    {
        gSimHtrueA[i] = gHA_t[i];
    }
    for (i = 0; i < E5_TAPS_PART; i++)
    {
        gSimHtrueB[i] = gHB_t[i];
    }

    e5_system_spectra(gHA_t, E5_TAPS_STD, 1, E5_TAPS_STD, &gHA[0], gWork.T);
    e5_system_spectra(gHB_t, E5_TAPS_PART, E5_PART, E5_PART_TAPS, &gHB[0][0], gWork.T);

    memset(&gWork,  0, sizeof(gWork));
    memset(&gState, 0, sizeof(gState));

    /* ================= 变体 0：标准 256 点，约束 ================= */
    e5_rt_reset(&gState.rt, 1);
    e5_noise_init(&noise, E5_NOISE_SEED);
    START_CYCLE_COUNT(c0);
    e5_bench_engine(&gState.rt, &gWork, gHA_t, &gHA[0],
                    E5_MU_STD, 1, E5_BENCH_BLOCKS,
                    (float *)gSimHhat, (float *)&gSimMSE[0], &mis, &noise);
    STOP_CYCLE_COUNT(c1, c0);
    gSimTiming[0] = (float)c1 / (float)E5_BENCH_BLOCKS;
    gSimTiming[5] = mis;
    sim_copy_hhat((const float *)gSimHhat, E5_TAPS_STD);

    /* ================= 变体 1：标准 256 点，不约束 ================= */
    e5_rt_reset(&gState.rt, 1);
    e5_noise_init(&noise, E5_NOISE_SEED);
    START_CYCLE_COUNT(c0);
    e5_bench_engine(&gState.rt, &gWork, gHA_t, &gHA[0],
                    E5_MU_STD, 0, E5_BENCH_BLOCKS,
                    (float *)gSimHhat, (float *)&gSimMSE[1 * E5_BENCH_BLOCKS], &mis, &noise);
    STOP_CYCLE_COUNT(c1, c0);
    gSimTiming[1] = (float)c1 / (float)E5_BENCH_BLOCKS;

    /* ================= 变体 2：分区 1024 点，约束 ================= */
    e5_rt_reset(&gState.rt, E5_PART);
    e5_noise_init(&noise, E5_NOISE_SEED);
    START_CYCLE_COUNT(c0);
    e5_bench_engine(&gState.rt, &gWork, gHB_t, &gHB[0][0],
                    E5_MU_PART, 1, E5_BENCH_BLOCKS,
                    (float *)gSimHhat, (float *)&gSimMSE[2 * E5_BENCH_BLOCKS], &mis, &noise);
    STOP_CYCLE_COUNT(c1, c0);
    gSimTiming[2] = (float)c1 / (float)E5_BENCH_BLOCKS;
    gSimTiming[6] = mis;

    /* ================= 变体 3：分区 1024 点，不约束 ================= */
    e5_rt_reset(&gState.rt, E5_PART);
    e5_noise_init(&noise, E5_NOISE_SEED);
    START_CYCLE_COUNT(c0);
    e5_bench_engine(&gState.rt, &gWork, gHB_t, &gHB[0][0],
                    E5_MU_PART_U, 0, E5_BENCH_BLOCKS,
                    (float *)gSimHhat, (float *)&gSimMSE[3 * E5_BENCH_BLOCKS], &mis, &noise);
    STOP_CYCLE_COUNT(c1, c0);
    gSimTiming[3] = (float)c1 / (float)E5_BENCH_BLOCKS;

    /* ================= 变体 4：标准 1024 点（N=2048），约束 ================= */
    for (i = 0; i < E5_FFT_N_BIG; i++)
    {
        gWork.frame[i] = (i < E5_TAPS_PART) ? gHB_t[i] : 0.0f;
    }
    e5_fft_real_n(gWork.frame, gHBbig, E5_FFT_N_BIG, E5_LOG2N_BIG);

    memset(&gState, 0, sizeof(gState));
    for (i = 0; i < E5_FFT_N_BIG; i++)
    {
        gState.big.Hspec[i] = gHBbig[i];
    }
    e5_big_reset(&gState.big);
    e5_noise_init(&noise, E5_NOISE_SEED);
    START_CYCLE_COUNT(c0);
    e5_bench_big(&gState.big, &gWork, gHB_t,
                 E5_MU_BIG, 1, E5_BENCH_BLOCKS,
                 (float *)gSimHhat, (float *)&gSimMSE[4 * E5_BENCH_BLOCKS], &mis, &noise);
    STOP_CYCLE_COUNT(c1, c0);
    gSimTiming[4] = (float)c1 / (float)E5_BENCH_BLOCKS;
    gSimTiming[7] = mis;

    /* ================= 实时阶段 cycles：标准 256 点，约束 =================
     * 模拟 ProcessBuffers 的调用顺序：每块 S0 -> S1 -> S2，
     * 记三个阶段各自的最大单阶段 cycles（判断 2.66 ms DMA 期限）。 */
    e5_rt_reset(&gState.rt, 1);
    e5_noise_init(&noise, E5_NOISE_SEED);
    gSimTiming[8]  = 0.0f;
    gSimTiming[9]  = 0.0f;
    gSimTiming[10] = 0.0f;
    for (v = 0; v < 20; v++)
    {
        e5_noise_block(gState.rt.xnew, E5_HOP, &noise);

        START_CYCLE_COUNT(c0);
        e5_rt_S0(&gState.rt, &gWork);
        STOP_CYCLE_COUNT(c1, c0);
        cyc = (float)c1;
        if (gSimTiming[8] < cyc)
        {
            gSimTiming[8] = cyc;
        }

        START_CYCLE_COUNT(c0);
        e5_rt_S1(&gState.rt, &gWork, &gHA[0]);
        STOP_CYCLE_COUNT(c1, c0);
        cyc = (float)c1;
        if (gSimTiming[9] < cyc)
        {
            gSimTiming[9] = cyc;
        }

        START_CYCLE_COUNT(c0);
        e5_rt_S2(&gState.rt, &gWork, E5_MU_STD, 1);
        STOP_CYCLE_COUNT(c1, c0);
        cyc = (float)c1;
        if (gSimTiming[10] < cyc)
        {
            gSimTiming[10] = cyc;
        }
    }

    /* ================= 实时阶段 cycles：分区 1024 点，不约束 ============
     * S2 只做频域乘加（4 个分区），没有 IFFT/FFT。 */
    e5_rt_reset(&gState.rt, E5_PART);
    e5_noise_init(&noise, E5_NOISE_SEED);
    gSimTiming[11] = 0.0f;
    gSimTiming[12] = 0.0f;
    for (v = 0; v < 20; v++)
    {
        e5_noise_block(gState.rt.xnew, E5_HOP, &noise);

        START_CYCLE_COUNT(c0);
        e5_rt_S0(&gState.rt, &gWork);
        STOP_CYCLE_COUNT(c1, c0);

        START_CYCLE_COUNT(c0);
        e5_rt_S1(&gState.rt, &gWork, &gHB[0][0]);
        STOP_CYCLE_COUNT(c1, c0);
        if (gSimTiming[11] < (float)c1)
        {
            gSimTiming[11] = (float)c1;
        }

        START_CYCLE_COUNT(c0);
        e5_rt_S2(&gState.rt, &gWork, E5_MU_PART_U, 0);
        STOP_CYCLE_COUNT(c1, c0);
        if (gSimTiming[12] < (float)c1)
        {
            gSimTiming[12] = (float)c1;
        }
    }

    gSimDebug[0] = 1u;   /* done 标志 */
    gSimDebug[1] = (uint32_t)E5_BENCH_BLOCKS;

    /* 标记数据已写好，然后正常返回（模拟器用 -bound-exe-time 收尾） */
    e5_sim_done();
    return 0;
}
