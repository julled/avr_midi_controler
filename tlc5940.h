/*
 
 tlc5940.h
  
 Copyright 2010 Matthew T. Pandina. All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 
 1. Redistributions of source code must retain the above copyright notice,
 this list of conditions and the following disclaimer.
 
 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.
 
 THIS SOFTWARE IS PROVIDED BY MATTHEW T. PANDINA "AS IS" AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 EVENT SHALL MATTHEW T. PANDINA OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
 */
 
#pragma once

#include <stdint.h>
#include <avr/io.h>
#include "abbreviations.h"


#define Reihen_Anz 6

#define GSCLK_DDR DDRD   // Timer1 Clock Output via PWM 8Mhz
#define GSCLK_PORT PORTD
#define GSCLK_PIN PD5

#define SIN_DDR DDRB
#define SIN_PORT PORTB
#define SIN_PIN PB5 	//MOSI

#define SCLK_DDR DDRB
#define SCLK_PORT PORTB
#define SCLK_PIN PB7	//SCK	

#define BLANK_DDR DDRB
#define BLANK_PORT PORTB
#define BLANK_PIN PB4	//SS

#define DCPRG_DDR DDRB
#define DCPRG_PORT PORTB
#define DCPRG_PIN PB2	

#define VPRG_DDR DDRB
#define VPRG_PORT PORTB
#define VPRG_PIN PB3

#define XLAT_DDR DDRB
#define XLAT_PORT PORTB
#define XLAT_PIN PB1

#ifndef TLC5940_MANUAL_DC_FUNCS
#define TLC5940_MANUAL_DC_FUNCS 1
#endif

#ifndef TLC5940_N
#define TLC5940_N 1
#endif
// --------------------------------------------------------



#if (12 * TLC5940_N > 255)
#define dcData_t uint16_t
#else
#define dcData_t uint8_t
#endif

#if (24 * TLC5940_N > 255)
#define gsData_t uint16_t
#else
#define gsData_t uint8_t
#endif

#if (16 * TLC5940_N > 255)
#define channel_t uint16_t
#else
#define channel_t uint8_t
#endif

#define channel3_t uint8_t

#define dcDataSize ((dcData_t)12 * TLC5940_N)
#define gsDataSize ((gsData_t)24 * TLC5940_N)
#define numChannels ((channel_t)16 * TLC5940_N)

uint8_t dcData[dcDataSize];
uint8_t gsData[Reihen_Anz][gsDataSize];
extern volatile uint8_t gsUpdateFlag;

static inline void TLC5940_SetGSUpdateFlag(void) {
	__asm__ volatile ("" ::: "memory");
	gsUpdateFlag = 1;
}

void TLC5940_SetDC(channel_t channel, uint8_t value);
void TLC5940_SetAllDC(uint8_t value);
void TLC5940_ClockInDC(void);


void TLC5940_SetGS(uint8_t row, channel_t channel, uint16_t value);
void TLC5940_SetAllGS(uint16_t value);
void TLC5940_Init(void);



/*
uint8_t dcData[12 * TLC5940_N] = {
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,
	0b11111111,	
};

uint8_t gsData[Reihen_Anz][24 * TLC5940_N] = {
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000001,
	0b00000000,
	0b00100000,
	0b00000100,
	0b00000000,
	0b10000000,
	0b00010000,
	0b00000010,
	0b00000000,
	0b01000000,
	0b00001000,
	0b00000001,
	0b00000000,
	0b00100000,
	0b00000100,
	0b00000000,
	0b10000000,
	0b00001111,
	0b11111111,
};*/





//uint8_t dcData[dcDataSize];
//uint8_t gsData[gsDataSize];
volatile uint8_t gsUpdateFlag;
//volatile static uint8_t row = 0;
