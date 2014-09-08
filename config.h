#include "I_O_structs.h"


// In Projektoptionen definiert
#define F_CPU 16000000UL 



//------------------------------- Midi

//  MidiBaudrate = 31250
#define UART_BAUD_RATE 31250
// Hilfsmakro zur UBRR-Berechnung ("Formel" laut Datenblatt)
#define UART_UBRR_CALC(BAUD_,FREQ_) ((FREQ_)/((BAUD_)*16L)-1)

#define NOTE_ON 0x91
#define midi_channel 14

#define midi_button_offset 0
#define midi_encoder_offset 20
#define midi_poti_offset 40
#define midi_touch_offset 60



//------------------------------- Potentiometer

#define ADC_delta_for_change_poti 7
#define potentiometer_count 3
#define POTI_ADC_SAMPLES 2   // erstes Auslesen immer Fehlerhaft wegen Touchpanel evtl
							// zweiter Wert beinhaltet richtiges Ergebniss!

int current_potentiometer = 0;

volatile struct potentiometer_t potentiometer[potentiometer_count]= { 	{ 0, 1}, 
																 	   	{ 0, 3},    //  0, 3
																   		{ 0, 5}	}; 
	

//------------------------------- Buttons


#define button_count 10

volatile struct button_t button[button_count]= { 	{ 0,0, 0,0, 2,1<<6 ,0,1<<0}, 
												   	{ 0,0, 0,0, 2,1<<7 ,0,1<<0},
							 
												   	{ 0,0, 0,0, 3,1<<0 ,0,1<<0}, 
													{ 0,0, 0,0, 3,1<<1 ,0,1<<0}, 
												   	{ 0,0, 0,0, 3,1<<2 ,0,1<<0}, 
												   	{ 0,0, 0,0, 3,1<<3 ,0,1<<0}, 
												   	{ 0,0, 0,0, 3,1<<4 ,0,1<<0}, 
												   	{ 0,0, 0,0, 3,1<<5 ,0,1<<0}, 
												   	{ 0,0, 0,0, 3,1<<6 ,0,1<<0},    
													{ 0,0, 0,0, 3,1<<7 ,0,1<<0} 
													
													}; 


//------------------------------- Encoder x

#define encoder_count 11

volatile struct encoder_t encoder[encoder_count] = { 	{ 64,0,0,  0,1<<0,1<<1,   0,3, 1<<0},   
														{ 64,0,0,  0,1<<2,1<<3,   0,3, 1<<1},
														{ 64,0,0,  0,1<<4,1<<5,   0,3, 1<<2},   
														{ 64,0,0,  0,1<<6,1<<7,   0,3, 1<<3},
								  
							 							{ 64,0,0,  1,1<<0,1<<1,   0,3, 1<<4}, 
														{ 64,0,0,  1,1<<2,1<<3,   0,3, 1<<5}, 
														{ 64,0,0,  1,1<<4,1<<5,   0,3, 1<<6}, 
														{ 64,0,0,  1,1<<6,1<<7,   0,3, 1<<7}, 

														{ 64,0,0,  2,1<<0,1<<1,   0,4, 1<<0}, 
														{ 64,0,0,  2,1<<2,1<<3,   0,2, 0   }, 
														{ 64,0,0,  2,1<<4,1<<5,   0,2, 0   } }; 

//------------------------------- Touchpanel


#define ADC_delta_for_change 7  // Ab welchem Koord-Delta eine neue Pos gesendet wird!
								//Werte altes lumberts TOUCH
#define valid_ADC_val_xmin 30		//30 
#define valid_ADC_val_ymin 80		 //80 
#define valid_ADC_val_xmax 950		//750 
#define valid_ADC_val_ymax 950		//900 

#define ADC_TOUCH_SAMPLES 7

#define TOUCH_DDR DDRA
#define TOUCH_PORT PORTA
#define TOUCH_X1 PA0
#define TOUCH_X2 PA2
#define TOUCH_Y1 PA4    //  PA4 
#define TOUCH_Y2 PA6



volatile struct touchscreen_t touchscreen = { 0,0, 1,1, 0,0, 0 };

								
