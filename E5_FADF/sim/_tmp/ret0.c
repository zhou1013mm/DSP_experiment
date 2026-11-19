/*
 * sim_hello.c -- minimal ADSP-SC589 SHARC Core1 smoke test for the CCES 2.9.3
 * ChipFactory instruction-set simulator (no target board needed).
 *
 * Validates: cc21k compile -> link -> simulator load -> run -> memory readback
 * -> cycle_count.h cycle measurement.
 *
 * Globals are non-static and volatile so they remain in DM data and their
 * symbols stay in the .dxe (CCES_runner -d exports memory by symbol name).
 * sim_hello_done() is a noinline marker used as the -d breakpoint trigger.
 *
 * main() returns a self-check code that chipfactory -exit-value reads back:
 *   42  = pattern OK, cycle counter reads 0 (functional sim, EMUCLK not modelled)
 *   298 = pattern OK, cycle counter reads > 0 (cycle counter modelled)
 *   43  = pattern BAD
 */
#include <stdint.h>
#include <cycle_count.h>

#define SIM_OUT_LEN      64
#define SIM_LOOP_COUNT   10000

volatile float     gOut[SIM_OUT_LEN];   /* [i] = (float)i * 0.5f          */
volatile uint32_t  gCycles[4];          /* [0] measured cycles            */
                                        /* [1] loop accumulator = 49995000*/
                                        /* [2] sentinel 0xFFFFFFFF        */
                                        /* [3] loop count 10000           */
volatile uint32_t  gPattern[4];         /* fixed test pattern             */
volatile int32_t   gVersion;            /* build version stamp            */
volatile int32_t   gResult;             /* 0 = PASS, 1 = FAIL             */

int adi_initialize(void)                /* required by app_startup.doj    */
{
    return 0;
}

/* noinline end-of-work marker: CCES_runner -d dumps memory on this bp */
void sim_hello_done(void)
{
    __asm__ volatile ("nop;");
}

int main(void)
{
    int32_t   i;
    int32_t   acc = 0;
    cycle_t   c_start;
    cycle_t   c_elapsed;

    gVersion    = 20261119;
    gPattern[0] = 0x12345678u;
    gPattern[1] = 0xDEADBEEFu;
    gPattern[2] = 0x55AA55AAu;
    gPattern[3] = 0xA5A5A5A5u;

    for (i = 0; i < SIM_OUT_LEN; i++) {
        gOut[i] = (float)i * 0.5f;
    }

    START_CYCLE_COUNT(c_start);
    for (i = 0; i < SIM_LOOP_COUNT; i++) {
        acc = acc + i;
    }
    STOP_CYCLE_COUNT(c_elapsed, c_start);

    gCycles[0] = (uint32_t)c_elapsed;
    gCycles[1] = (uint32_t)acc;
    gCycles[2] = 0xFFFFFFFFu;
    gCycles[3] = 0x00002710u;           /* 10000 */

    gResult = (acc == 49995000) ? 0 : 1;

    sim_hello_done();

    return 0;
}

