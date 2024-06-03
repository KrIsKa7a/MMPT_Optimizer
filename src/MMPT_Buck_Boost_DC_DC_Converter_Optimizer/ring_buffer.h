/*
 * ring_buffer.h
 *
 *  Created on: Jun 3, 2024
 *      Author: User
 */

#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

#include <stdint.h>

#define cENTRIES_COUNT	16

typedef struct ADC_Entries_t
{
	uint16_t u16Data[cENTRIES_COUNT];
	uint8_t u8Index;
} ADC_Entries;

void InitEntry(ADC_Entries* const psADC_EntriesL);
void AddEntry(ADC_Entries* const psADC_EntriesL, uint16_t u16EntryL);
void ClearEntries(ADC_Entries* const psADC_EntriesL);
uint16_t GetAverage(const ADC_Entries* const psADC_EntriesL);

#endif /* RING_BUFFER_H_ */
