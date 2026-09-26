/*
** adi_initialize.c source file generated on November 19, 2015 at 13:13:16.
**
** Copyright (C) 2000-2015 Analog Devices Inc., All Rights Reserved.
**
** This file is generated automatically. You should not modify this source file,
** as your changes will be lost if this source file is re-generated.
*/

#include <sys/platform.h>
#include <services/int/adi_gic.h>

#include "adi_initialize.h"

extern int32_t adi_SRU_Init(void);

int32_t adi_initComponents(void)
{
	int32_t result = 0;

	result = adi_gic_Init();


	if (result == 0) {
		result = adi_SRU_Init(); /* auto-generated code (order:0) */
	}

	return result;
}

