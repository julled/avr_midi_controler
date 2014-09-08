
#include "74hc165.h"
#include "abbreviations.h"


void init_74hc165()
{
	// Setze MISO als Eingang
	setInput(MISO_DDR, MISO_PIN);

	// Setze Latch-steuerleitung als Ausgang
	setOutput(PL_DDR, PL_PIN);

	// Enable Bit-Shifting per Clockcycle
	setLow(CE_PORT, CE_PIN);
}
	

void read_all_74hc165()
{

	// Latch in Data von Eingabeeinheiten ( Encoders , Buttons )
	setLow(PL_PORT, PL_PIN);
	setHigh(PL_PORT, PL_PIN);

	// Input von hinten beginnend dass Addr 0 dem Chip entspricht bei dem SIN als erster Eingang genutzt wird
	for (int8_t i = 0; i<Input_expander_count;i++)  
	{
		// Wenn das SPI-Modul aktiviert wird, wird NICHT automatisch SPIF gesetzt, es bleibt auf Null.
		// Deshalb muss nach der Initialisierung des SPI-Moduls ein Dummy Byte gesendet werden, damit am Ende der Übertragung SPIF gesetzt wird 
		SPDR = 0x00;

  		// Warte bis ein Byte empfangen wurde
 		while(!(SPSR & (1<<SPIF)));

 		// Speichere empfangene Daten im Eingangsbuffer
		Input_Expander_Data[i] = SPDR;
	}

}
