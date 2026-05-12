/*
 * Copyright (c) 2025 Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include <SEGGER_RTT.h>


__attribute__((constructor(1000)))
static void modm_rtt_init()
{
	SEGGER_RTT_Init();
}

/*********************************************************************
*
*       SEGGER_RTT_GetBytesInBuffer()
*
*  Function description
*    Returns the number of bytes currently used in the down buffer.
*
*  Parameters
*    BufferIndex  Index of the down buffer.
*
*  Return value
*    Number of bytes that are used in the buffer.
*/
unsigned SEGGER_RTT_GetBytesInDownBuffer(unsigned BufferIndex) {
	unsigned RdOff;
	unsigned WrOff;
	unsigned r;
	volatile SEGGER_RTT_CB* pRTTCB;
	//
	// Avoid warnings regarding volatile access order.  It's not a problem
	// in this case, but dampen compiler enthusiasm.
	//
	// Access RTTCB uncached to make sure we see changes made by the J-Link side and all of our changes go into HW directly
	pRTTCB = (volatile SEGGER_RTT_CB*)((uintptr_t)&_SEGGER_RTT + SEGGER_RTT_UNCACHED_OFF);
	RdOff = pRTTCB->aDown[BufferIndex].RdOff;
	WrOff = pRTTCB->aDown[BufferIndex].WrOff;
	if (RdOff <= WrOff) {
		r = WrOff - RdOff;
	} else {
		r = pRTTCB->aDown[BufferIndex].SizeOfBuffer - (WrOff - RdOff);
	}
	return r;
}

#include <printf/printf.h>

static void output_gadget(char c, void* BufferIndex)
{
	SEGGER_RTT_PutCharSkipNoLock((unsigned) BufferIndex, c);
}

int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...)
{
	va_list ParamList;
	va_start(ParamList, sFormat);
	const int retval = vfctprintf(output_gadget, (void*) BufferIndex, sFormat, ParamList);
	va_end(ParamList);
	return retval;
}

int SEGGER_RTT_vprintf(unsigned BufferIndex, const char * sFormat, va_list * pParamList)
{
	return vfctprintf(output_gadget, (void*) BufferIndex, sFormat, *pParamList);
}
