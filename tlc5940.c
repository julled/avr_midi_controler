
#include "tlc5940.h"
#include "abbreviations.h"





void TLC5940_Init(void) {
	setOutput(GSCLK_DDR, GSCLK_PIN);
	setOutput(SCLK_DDR, SCLK_PIN);
	setOutput(DCPRG_DDR, DCPRG_PIN);
	setOutput(VPRG_DDR, VPRG_PIN);
	setOutput(XLAT_DDR, XLAT_PIN);
	setOutput(BLANK_DDR, BLANK_PIN);
	setOutput(SIN_DDR, SIN_PIN);
	
	
	//setLow(GSCLK_DDR, GSCLK_PIN);
	setLow(GSCLK_PORT, GSCLK_PIN);
	setLow(SCLK_PORT, SCLK_PIN);
	setLow(DCPRG_PORT, DCPRG_PIN);
	setHigh(VPRG_PORT, VPRG_PIN);
	setLow(XLAT_PORT, XLAT_PIN);
	setHigh(BLANK_PORT, BLANK_PIN);
	DDRD |= 0b00100000;  // Bug workaround ...  setLow(GSCLK_DDR, GSCLK_PIN); sets DDR for GSCLK as Input!
}


void TLC5940_SetDC(channel_t channel, uint8_t value) {	
	channel = numChannels - 1 - channel;
	channel_t i = (channel3_t)channel * 3 / 4;
	
	switch (channel % 4) {
		case 0:
			dcData[i] = (dcData[i] & 0x03) | (uint8_t)(value << 2);
			break;
		case 1:
			dcData[i] = (dcData[i] & 0xFC) | (value >> 4);
			i++;
			dcData[i] = (dcData[i] & 0x0F) | (uint8_t)(value << 4);
			break;
		case 2:
			dcData[i] = (dcData[i] & 0xF0) | (value >> 2);
			i++;
			dcData[i] = (dcData[i] & 0x3F) | (uint8_t)(value << 6);
			break;
		default: // case 3:
			dcData[i] = (dcData[i] & 0xC0) | (value);
			break;
	}
}

void TLC5940_SetAllDC(uint8_t value) {
	uint8_t tmp1 = (uint8_t)(value << 2);
	uint8_t tmp2 = (uint8_t)(tmp1 << 2);
	uint8_t tmp3 = (uint8_t)(tmp2 << 2);
	tmp1 |= (value >> 4);
	tmp2 |= (value >> 2);
	tmp3 |= value;
	

	dcData_t i = 0;
	do {
		dcData[i++] = tmp1;              // bits: 05 04 03 02 01 00 05 04
		dcData[i++] = tmp2;              // bits: 03 02 01 00 05 04 03 02
		dcData[i++] = tmp3;              // bits: 01 00 05 04 03 02 01 00
	} while (i < dcDataSize);

}



void TLC5940_ClockInDC(void) {
	setHigh(DCPRG_PORT, DCPRG_PIN);
	setHigh(VPRG_PORT, VPRG_PIN);
	
	for (dcData_t i = 0; i < dcDataSize; i++) {
		SPDR = dcData[i];
		while (!(SPSR & (1 << SPIF)));
	}
	pulse(XLAT_PORT, XLAT_PIN);
}



void TLC5940_SetAllGS(uint16_t value) {
	uint8_t tmp1 = (value >> 4);
	uint8_t tmp2 = (uint8_t)(value << 4) | (tmp1 >> 4);

	for(uint8_t r = 0; r<Reihen_Anz; r++)
	{
		gsData_t i = 0;
		do {
			gsData[r][i++] = tmp1;              // bits: 11 10 09 08 07 06 05 04
			gsData[r][i++] = tmp2;              // bits: 03 02 01 00 11 10 09 08
			gsData[r][i++] = (uint8_t)value;    // bits: 07 06 05 04 03 02 01 00
		} while (i < gsDataSize);
	}
}

void TLC5940_SetGS(uint8_t row, channel_t channel, uint16_t value) {
	channel = numChannels - 1 - channel ; // channel = numChannels - 1 - channel ;     -1 dass Spalte von 0 bis 7 
	channel3_t i = (channel3_t)channel * 3 / 2;
	
	//row = row - 1; // -1 dass Reihe von 0 bis 5
	
	switch (channel % 2) {
		case 0:
		if(row < Reihen_Anz && i < 24)
		{
			gsData[row][i] = (value >> 4);
			i++;
			gsData[row][i] = (gsData[row][i] & 0x0F) | (uint8_t)(value << 4);			
			break;
		}
		default: // case 1:
		if(row < Reihen_Anz && i < 24)
		{
			gsData[row][i] = (gsData[row][i] & 0xF0) | (value >> 8);
			i++;
			gsData[row][i] = (uint8_t)value;
			break;
		}
	}
}
