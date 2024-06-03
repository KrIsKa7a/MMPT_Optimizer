/*
 * ring_buffer.c
 *
 *  Created on: Jun 3, 2024
 *      Author: User
 */
#include "ring_buffer.h"

void InitEntry(ADC_Entries* const psADC_EntriesL)
{
	psADC_EntriesL->u8Index = 0;
}

void AddEntry(ADC_Entries* const psADC_EntriesL, uint16_t u16Entry)
{
	if (psADC_EntriesL->u8Index >= cENTRIES_COUNT)
	{
		psADC_EntriesL->u8Index = psADC_EntriesL->u8Index % 16;
	}

	psADC_EntriesL->u16Data[psADC_EntriesL->u8Index++] = u16Entry;
}

void ClearEntries(ADC_Entries* const psADC_EntriesL)
{
	uint8_t u8IndexL;
	for (u8IndexL = 0; u8IndexL < cENTRIES_COUNT; u8IndexL++)
	{
		psADC_EntriesL->u16Data[u8IndexL] = 0;
	}

	psADC_EntriesL->u8Index = 0;
}

uint16_t GetAverage(const ADC_Entries* const psADC_EntriesL)
{
	uint8_t u8IndexL;
	uint32_t u32EntriesSumL = 0;
	for (u8IndexL = 0; u8IndexL < psADC_EntriesL->u8Index; u8IndexL++)
	{
		u32EntriesSumL += psADC_EntriesL->u16Data[u8IndexL];
	}

	return (uint16_t)(u32EntriesSumL / psADC_EntriesL->u8Index);
}
