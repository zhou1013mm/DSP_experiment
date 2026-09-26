/*
 **
 ** sru_config.c source file generated on ‹ãŒŽ 17, 2026 at 08:06:24.	
 **
 ** Copyright (C) 2014-2026 Analog Devices Inc., All Rights Reserved.
 **
 ** This file is generated automatically based upon the options selected in 
 ** the SRU Configuration editor. Changes to the SRU configuration should
 ** be made by changing the appropriate options rather than editing this file.
 **
 ** Only registers with non-default values are written to this file.
 **
 */
 
#include <stdint.h>
#include <sys/platform.h>

int32_t adi_SRU_Init(void);

/*
 * Initialize the Signal Routing Unit
 */
int32_t adi_SRU_Init(void)
{
    /* SPT4A_CLK_I, SPT5A_CLK_I, SPT4B_CLK_I, SPT5B_CLK_I, SPT6B_CLK_I, SPT6A_CLK_I */
    *pREG_DAI1_CLK0 = (unsigned int) 0x2526302b;

    /* SPT5A_D1_I, SPT5B_D0_I, SPT6A_D0_I, SPT5B_D1_I, SPT6A_D1_I */
    *pREG_DAI1_DAT1 = (unsigned int) 0x0f3be289;

    /* SPT4B_D1_I, SPT4B_D0_I, SPT5A_D0_I, SPT4A_D1_I, SPT4A_D0_I */
    *pREG_DAI1_DAT0 = (unsigned int) 0x08fbe185;

    /* SRC4_TDM_OP_I, SRC7_TDM_OP_I, SRC6_TDM_OP_I, SRC7_DAT_IP_I, SRC5_TDM_OP_I */
    *pREG_DAI1_DAT3 = (unsigned int) 0x3efbefbe;

    /* SRC5_DAT_IP_I, SPT6B_D1_I, SPT6B_D0_I, SRC4_DAT_IP_I, SRC6_DAT_IP_I */
    *pREG_DAI1_DAT2 = (unsigned int) 0x3efbe450;

    /* SPDIF1_RX_I */
    *pREG_DAI1_DAT5 = (unsigned int) 0x3e000000;

    /* SPDIF1_TX_DAT_I */
    *pREG_DAI1_DAT4 = (unsigned int) 0x0000003e;

    /* SPT6B_FS_I, SPT4B_FS_I, SPT5A_FS_I, SPT6A_FS_I, SPT5B_FS_I, SPT4A_FS_I */
    *pREG_DAI1_FS0 = (unsigned int) 0x3de6b473;

    /* DAI1_PB19_I, DAI1_PB18_I, INV_DAI1_PB20_I, DAI1_PB17_I, INV_DAI1_PB19_I, DAI1_PB20_I */
    *pREG_DAI1_PIN4 = (unsigned int) 0x0fc94f9e;

    /* DAI1_PB09_I, DAI1_PB12_I, DAI1_PB11_I, DAI1_PB10_I */
    *pREG_DAI1_PIN2 = (unsigned int) 0x0fc68c98;

    /* DAI1_PB04_I, DAI1_PB02_I, DAI1_PB03_I, DAI1_PB01_I */
    *pREG_DAI1_PIN0 = (unsigned int) 0x0fdfbf16;

    /* DAI1_PB08_I, DAI1_PB06_I, DAI1_PB07_I, DAI1_PB05_I */
    *pREG_DAI1_PIN1 = (unsigned int) 0x0fdfbf17;

    /* DAI1_PBEN20_I, DAI1_PBEN17_I, DAI1_PBEN16_I, DAI1_PBEN19_I, DAI1_PBEN18_I */
    *pREG_DAI1_PBEN3 = (unsigned int) 0x0071f79b;

    /* DAI1_PBEN10_I, DAI1_PBEN09_I, DAI1_PBEN06_I, DAI1_PBEN08_I, DAI1_PBEN07_I */
    *pREG_DAI1_PBEN1 = (unsigned int) 0x1348d000;

    /* DAI1_PBEN05_I, DAI1_PBEN01_I, DAI1_PBEN02_I, DAI1_PBEN03_I, DAI1_PBEN04_I */
    *pREG_DAI1_PBEN0 = (unsigned int) 0x01000001;

    /* PADS0 DAI0 Port Input Enable Control Register */
    *pREG_PADS0_DAI0_IE = (unsigned int) 0x001FFFFE;

    /* PADS0 DAI1 Port Input Enable Control Register */
    *pREG_PADS0_DAI1_IE = (unsigned int) 0x001FFFFE;

    return 0;
}

