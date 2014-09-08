

#include "74hc595.h"
#include "abbreviations.h"


void init_74hc595()
{
	setOutput(RCK_DDR, RCK_PIN);
	
	set_all_output_74hc595(0x00);
}

// Schreibt ein vollständiges Byte für einen bestimmten Chip im Ausgangsbuffer
void set_output_74hc595(uint8_t port_addr, uint8_t data)
{
	Output_Expander_Data[port_addr] = data;
}

// Schreiben eines einzigen Bits (1/0) auf einen bestimmten Chip im Ausgangsbuffer
void set_output_74hc595_pin(uint8_t port_addr, uint8_t pin, uint8_t true_false)
{
	if(true_false)
		//SETBIT(Output_Expander_Data[port_addr] , pin )
		Output_Expander_Data[port_addr] |= pin;
	else
		//CLEARBIT(Output_Expander_Data[port_addr] , pin )
		Output_Expander_Data[port_addr] &= ~pin;
	//Output_Expander_Data[port_addr] = data;
}

// Beschreiben aller Chip im Ausgangsbuffer mit einem bestimmten Wert
void set_all_output_74hc595(uint8_t data)
{
	for (int i = 0; i<Output_expander_count;i++)
		Output_Expander_Data[i] = data;
}

// Übertragungs des Ausgangsbuffers auf alle 74HC595 mithilfe des SPI-Buses 
void write_all_74hc595()
{
 	// Output von hinten beginnend dass Addr 0 dem Chip entspricht bei dem SIN als erster Eingang genutzt wird
	for (int i = Output_expander_count-1; i>= 0;i--) 
	{
		SPDR = Output_Expander_Data[i];    // Daten per SPI senden
    	while (!(SPSR & (1<<SPIF))); 	  // Warten bis Übertragung beendet
	}
	pulse(RCK_PORT, RCK_PIN);	// Pulse auf Chipselect um Daten im Schieberegister aller 74HC595 auf in die internen Register zu latchen
	
}

