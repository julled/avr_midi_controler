		

#include <avr/interrupt.h>

#include <avr/io.h>
#include <stdio.h>
#include <avr/pgmspace.h>

#include <util/delay.h> 
#include <util/atomic.h>
#include <math.h>

#include "tlc5940.h"
#include "74hc595.h"
#include "74hc165.h"
#include "calibrate.h"
//#include "I_O_structs.h"
#include "config.h"
#include "abbreviations.h"


// Prototypen für nicht ausgelagerte Funktionen
//
//
	//-----------------------------UART
	
	// Umleitung der Ausgabe von printf auf UART:
	// a. Deklaration der primitiven Ausgabefunktion
	//int uart_putchar(char c, FILE *stream);
	// b. Umleiten der Standardausgabe stdout (Teil 1)
	//static FILE mystdout = FDEV_SETUP_STREAM( uart_putchar, NULL, _FDEV_SETUP_WRITE );
	

	void uart_init(void);
	int uart_putchar( char c, FILE *stream );

	//-----------------------ADC
	uint16_t ADC_Read( uint8_t channel );
	void ADC_Init();

	//-----------------------Encoder
	int8_t table[16] PROGMEM = {0,0,-1,0 ,0,0,0,1 ,1,0,0,0 ,0,-1,0,0};  
	void encode_init( void );
	int8_t encode_read( int index ) ;

	// ----------------------Display
	void Display_SetCross(uint8_t xLED,uint8_t yLED);
	void multiplex_display_row(uint8_t row);
	void Display_SetCross(uint8_t xLED,uint8_t yLED);

	//-----------------------Touch
	void read_touchscreen();
	uint16_t MedianFilter(uint16_t* values);
	int calibrate();
	uint8_t read_Touch_ADC_values( uint16_t* adcvalx, uint16_t* adcvaly);

	//------------------SPI
	void init_SPI();

	//------------------Timer
	void timer_config();

	//------------------MIDI
	void controlChange(int channel, int controller, uint8_t value);

	void midiMsg(int cmd, int data1, int data2);




int main(void)
 {	
		

    // b. Umleiten der Standardausgabe stdout (Teil 2)
    //stdout = &mystdout;
 
 	// Init everything
	// Init Touch & Potis
	DDRA = 0x00; // ADWandler-Pins auf Eingang schalten
	uint16_t ADC_val;

	ADC_Init();
	// Init LED Matrix
	TLC5940_Init();
	// Init SPI
	init_SPI();
	// Init Timer
	timer_config();

	TLC5940_SetAllDC(63);
	TLC5940_ClockInDC();
	TLC5940_SetAllGS(0);	

	// Init all 74hc595
	init_74hc595();
	// Init all 74hc165
	init_74hc165();

	// Enable Interrupts globally


	// TEMP TEMP TEMP
	DDRC |= 0b01000000;

	// Kalibriere Touchpanel
	calibrate();
	sei();
	// Init UART
	uart_init();

	
	
	while (1)
	{
		static uint8_t current_potentiometer = 0;

		// POTENTIOMETER auslesen
		{		
			/*	switch( current_potentiometer )
			{
			//	case 1:
			//		PORTC &= ~0b01000000;
			//		break;
				case 2:
					PORTC |= 0b01000000;	
					break;

			}	*/		
			

			// erstes Auslesen immer Fehlerhaft wegen Touchpanel evtl
			// zweiter Wert beinhaltet richtiges Ergebniss!
			// POTI_ADC_SAMPLES sollte daher 2 sein damit nach dem zweiten lesen in ADC_val das richtige ergebniss steht
			ADC_val = 0;		
			for  ( uint8_t count = 0 ; count < POTI_ADC_SAMPLES ; count++ )   
				ADC_val = ADC_Read(potentiometer[current_potentiometer].adc_channel);


			if( ADC_val  > ( potentiometer[current_potentiometer].value + ADC_delta_for_change_poti ) || ( ADC_val  < ( potentiometer[current_potentiometer].value - ADC_delta_for_change_poti ) ) )  // +- 8 von 1024 Quantisierungsstufen / 128 Midi Schritte .  // if( ADC_val  > ( potentiometer[current_potentiometer].value + 10 ) || ( ADC_val  < ( potentiometer[current_potentiometer].value - 10 ) ) )																																			
			{
				potentiometer[current_potentiometer].value = ADC_val;
				controlChange(midi_channel, midi_poti_offset + current_potentiometer,ADC_val/8);
				//printf("%i. Poti %i\n", current_potentiometer , potentiometer[current_potentiometer].value );
			}

			current_potentiometer++;
			
			if ( current_potentiometer == potentiometer_count)
				current_potentiometer = 0;
		}

		
		//Display_SetCross(4,2);
		// TOUCHPANEL auslesen

		read_touchscreen();
	  
	  	  if(touchscreen.FLAG_Display_change) 
		  {
			TLC5940_SetAllGS(0);
			//Display_SetParabel(touchscreen.last_x , touchscreen.last_y );
			Display_SetCross(touchscreen.last_LED_x,touchscreen.last_LED_y);
			touchscreen.FLAG_Display_change = 0;
		  }	
	
  	}

	
}


//--------------------Timer---------------------


void timer_config()
{
	//************** I/O Update Timer **************	
	  //TCCR0 = (1<<WGM01) | (1<<CS01) | (1<<CS00);     // CTC, XTAL / 64
	  TCCR0 = (1<<WGM01) | (1<<CS01) ;     // CTC, XTAL / 8
	  OCR0 = (uint8_t)(F_CPU / 64.0 * 1e-3 - 0.5 * 10);       // 1ms   VLLT NOCH RUNTER! prescaler = /8 statt /64 damit kurze Tastendrücke erfasst werden
	  TIMSK |= 1<<OCIE0;
	
	//************** GSCLK-Timer **************
	// Clear TIMER1 Reg back to default
	TCCR1A = 0b00000000;
	TCCR1B = 0b00000000;
	// Enable timer 1 Compare Output channel A in toggle mode
	TCCR1A |= (1 << COM1A0);
	// Configure timer 1 for CTC mode
	TCCR1B |= (1 << WGM12);
	// Set up timer to fCPU (no Prescale) = 16Mhz/1 = 16Mhz
 	TCCR1B |= (1 << CS10);
	// Set CTC compare value to pulse PIN at 8Mhz
	// (1 / Target Frequency) / (1 / Timer Clock Frequency) - 1
	// (1/16000000)/(1/16000000)-1 = 0 = 16Mhz
	// Full period of OC1A ( GSCLK @ TLC5940 ) pulse requires 2 clock ticks (HIGH, LOW)
	// So = f/2 = 16Mhz/2 = 8Mhz
	OCR1AH = 0x00;//0x07;//7;
	OCR1AL = 0x01;//0xFF;//7;
	// Mit Calc erzeugt

	//************** SPI-I/O Timer **************	
	//TIMER 2 config für MATRIX 
	// CTC with OCR2 as TOP , clk_io/1024 (From prescaler)
	TCCR2 = ((1 << WGM21) | (1 << CS22) | (1 << CS21) | (1 << CS20 ));
	// Generate an interrupt every 4096 clock cycles @ 8Mhz /  ( 8192clk cycles @ 16Mhz ) 
	//OCR2 = 7;  // (8-1) = 7     8*1024...
	OCR2 = 7;  // (8-1) = 7     8*1024...
	// Enable Timer/Counte2 Compare Match interrupt
	TIMSK |= (1 << OCIE2);

}


ISR( TIMER0_COMP_vect )             // 1ms fuer manuelle Eingabe pro Einheit sollte sein ... bis jetzt ca 30ms pro Einheit weil 30 eingabeeinheiten. 
{

    //ENCODERAUSWERTUNG
	{         
	
		static uint8_t current_encoder = 0;
		// alten Wert speichern	
	    encoder[current_encoder].last = (encoder[current_encoder].last << 2)  & 0x0F;
		
		// auslesen des Encoderpins A
		if ( Input_Expander_Data[encoder[current_encoder].inputexpander_addr] & encoder[current_encoder].pinA )
		    encoder[current_encoder].last |=2;
		// auslesen des Encoderpins B
	    if ( Input_Expander_Data[encoder[current_encoder].inputexpander_addr] & encoder[current_encoder].pinB )
		    encoder[current_encoder].last |=1;

		// Bestimmung und abspeichern der Drehrichtung anhand des generierten Sprungwertes
	    encoder[current_encoder].enc_delta += pgm_read_byte(&table[encoder[current_encoder].last]);

		// Drehung des Encoders CW
		if(encoder[current_encoder].enc_delta == 1)
		{
			controlChange(midi_channel, midi_encoder_offset + current_encoder,0x7F);

			//encoder[current_encoder].value += encoder[current_encoder].enc_delta;
			if(encoder[current_encoder].value >127)
				encoder[current_encoder].value =127;

			encoder[current_encoder].enc_delta = 0;
		}

		// Drehung des Encoders CCW
		else if(encoder[current_encoder].enc_delta == -1)
		{
			controlChange(midi_channel, midi_encoder_offset + current_encoder,0x01);

			//encoder[current_encoder].value += encoder[current_encoder].enc_delta;
			if(encoder[current_encoder].value <1)
				encoder[current_encoder].value =0;

			encoder[current_encoder].enc_delta = 0;
		}
		
	
		current_encoder++;	

		if(current_encoder == encoder_count )
			current_encoder = 0;
	}
	
	//TASTERAUSWERTUNG
	{

		uint8_t i;
		static uint8_t current_button = 0;
	
		// Hat sich key_state im Vergleich zum Pegel geändert ? 1 = ja , 0 = nein
		i = button[current_button].key_state ^ ( Input_Expander_Data[button[current_button].inputexpander_addr] & button[current_button].pin ); 
				                     
		button[current_button].ct0 = ~( button[current_button].ct0 & i );                          // reset oder zähle counter0
		button[current_button].ct1 = button[current_button].ct0 ^ (button[current_button].ct1 & i);// reset oder zähle counter1
		i &= button[current_button].ct0 & button[current_button].ct1;                              // zähle bis roll over 
		button[current_button].key_state ^= i;                                					   // dann toggle entprellten status
	
		// Wurde Taster gerade gedrückt & wurde Taster bis gerade nicht gedrückt?
		if(button[current_button].key_state && !button[current_button].key_pressed)
		{
			// Setze tasterflag dass Taster gerade gedrückt ist
			button[current_button].key_pressed = 1;
	
			// Midimessage senden : Taster gedrückt
			controlChange(midi_channel,midi_button_offset + current_button,127);
		
		}
		// Wurde Taster losgelassen? 
		else if(!button[current_button].key_state)
		{
			// Wenn Taster bis gerade gedrückt wurde : Tasterflag = true
			if(button[current_button].key_pressed)  
				// Midimessage senden : Taster losgelassen
				controlChange(midi_channel,midi_button_offset + current_button,0);	
			
			// Lösche tasterflag dass Taster gerade gedrückt ist => Taster nicht mehr gedrückt		
			button[current_button].key_pressed = 0;
		}
	
		current_button++;	

		if(current_button == button_count )
			current_button = 0;
	}
	
}


// This interrupt will get called every 4096 clock cycles
ISR(TIMER2_COMP_vect) 
{

	static uint8_t xlatNeedsPulse = 0;
	static uint8_t row = 0;
	
	

	setHigh(BLANK_PORT, BLANK_PIN);

	// VPRG = High wenn vorher DC-Data eingespeichert wurde
	// => if ist nur beim allersten Durchgang true
	if (outputState(VPRG_PORT, VPRG_PIN)) 
	{
		setLow(VPRG_PORT, VPRG_PIN);
		if (xlatNeedsPulse) {
			pulse(XLAT_PORT, XLAT_PIN);
			xlatNeedsPulse = 0;
		}
		pulse(SCLK_PORT, SCLK_PIN);		// dann sende Extra Pulse auf SCLK Leitung um die DC-Daten zu speichner!
	} 

	// Allg. Fall :
	else if (xlatNeedsPulse) 
	{
		pulse(XLAT_PORT, XLAT_PIN);
		xlatNeedsPulse = 0;
		
		// Reihenmultiplexing der TLC5940-Ausgabe
		multiplex_display_row(row);
		row++;
		if (row == 6) row = 0;


	}

	multiplex_led_rings();

	write_all_74hc595(); // Muss zwischen XLAT und SPI sein damit XLAT zuerst geschalten werden kann um  Daten reinzuulatchen
						 // bzw zu aktivieren da nach ISR erst steuersignaler erzeugt werden. Nach SPI für TLC würde zu falschen Daten am TLC
						 // führen bevor die wichtigen steuersignale sie reinlatchen



	read_all_74hc165();

	// Below this we have 4096 cycles to shift in the data for the next cycle
	setLow(BLANK_PORT, BLANK_PIN); // Muss hier stehn sonst verzerrungen in darstellung... ???!
	
	// Schreibe neue PWM Daten in TLC
	for (gsData_t i = 0; i < gsDataSize; i++) {
			SPDR = gsData[row][i];
		while (!(SPSR & (1 << SPIF)));
	}

	xlatNeedsPulse = 1;
	gsUpdateFlag = 0;


}

//---------------------- Encoder Funktionen -----------------------------------


void multiplex_led_rings()
{
	static uint8_t current_multiplexed_encoder = 0;
	
	// Deaktiviere Multiplex-Transistor des letzten durchlaufes
	if(current_multiplexed_encoder != 0)
		set_output_74hc595_pin( encoder[current_multiplexed_encoder-1].led_ring_multiplex_outputexpander_addr, encoder[current_multiplexed_encoder-1].led_ring_multiplex_pin, 0); // to do switch dings
	else
		set_output_74hc595_pin( encoder[encoder_count].led_ring_multiplex_outputexpander_addr, 1<<encoder[encoder_count].led_ring_multiplex_pin, 0); // to do switch dings


	// Schreibe Encoder-Value auf Anzeigeport
	//set_output_74hc595( encoder[current_multiplexed_encoder].led_ring_outputexpander_addr, 1<<(encoder[current_multiplexed_encoder].value-50) );
	//schreibe_Encoder_Leds(encoder[current_multiplexed_encoder].value);


	// Setze neuen Multiplex-Transistor
	set_output_74hc595_pin( encoder[current_multiplexed_encoder].led_ring_multiplex_outputexpander_addr, encoder[current_multiplexed_encoder].led_ring_multiplex_pin,1 ); // to do switch dings


	current_multiplexed_encoder++;
	if(current_multiplexed_encoder == encoder_count )
		current_multiplexed_encoder = 0;
}


void schreibe_Encoder_Leds(uint8_t value)
{

	if ( value < 63 )
	{
		int8_t ledcalc1 = ( ( value/9 ) - 8 ) * (-1) -2;	
		set_output_74hc595(0,0x80>>ledcalc1);
		set_output_74hc595(2,0);
	}	
	else if ( value > 65 )
	{
		set_output_74hc595(2,0b00000010<<((value - 64 )/9-1));	
		set_output_74hc595(0,0);
	}
	else if	( value == 64 || value == 63 || value == 65)
	{
		set_output_74hc595(2,1);	
		set_output_74hc595(0,1);
	}
	//set_output_74hc595_pin(2,1<<0,1);

}






//---------------------- SPI_init -----------------------------------
void init_SPI()
{

	//SPI
	//SPCR = 0x50;		// SPE/MSTR/CPOL=0/CPHA=0	SPI Mode 0
	//SPCR = 0x54;		// SPE/MSTR/CPOL=0/CPHA=1	SPI Mode 1
	//SPCR = 0x58;		// SPE/MSTR/CPOL=1/CPHA=0	SPI Mode 2
	//SPCR = 0x5C;		// SPE/MSTR/CPOL=1/CPHA=1	SPI Mode 3

	// Enable SPI, Master, set clock rate fck/2
	SPCR = (1 << SPE) | (1 << MSTR); //| (1 << CPOL) | (1 << CPHA);
	SPSR = (1 << SPI2X);

}


//---------------------- Touchscreen Funktionen -----------------------------------

void read_touchscreen()
{
	//TOUCHPANELAUSWERTUNG

	uint16_t x_coord_read;
	uint16_t y_coord_read;
	uint16_t midix,midiy;

	uint8_t  x_validvalues = 0;
	uint8_t  y_validvalues = 0;

	uint16_t x_ADC_values[ADC_TOUCH_SAMPLES];
	uint16_t y_ADC_values[ADC_TOUCH_SAMPLES];

	uint16_t x_read_Filtered;
	uint16_t y_read_Filtered;
	
	uint8_t validsamples = read_Touch_ADC_values ( &x_ADC_values[0] , &y_ADC_values[0]);
	
	///////////////////////////////////////////////////
	// Auswertung der eingelesenen AD-Wandler-Daten

	// Alle gemessenen Werte gültig??
	if( validsamples )
	{
		// Medianfilter über Messwerte


		// auskommentiert da AD-Wert schon gut ab einem Sample
		x_read_Filtered = MedianFilter(&x_ADC_values[0]);
		y_read_Filtered = MedianFilter(&y_ADC_values[0]);

		x_coord_read = x_read_Filtered;
		y_coord_read = y_read_Filtered;

		POINT Panel_Touch = { x_coord_read , y_coord_read };
		POINT Midi_Touch = { 0, 0 };

		// Umrechnung der Touchwerte in kalibrierte Midi-Werte
		getDisplayPoint( &Midi_Touch, &Panel_Touch, &matrix );
		
		midix = Midi_Touch.x;
		midiy = Midi_Touch.y;

		touchscreen.FLAG_Display_change = 0;
		
		// Prüfen auf Abweichung vom letzten gemessenen X-Touchpanelwert
		if( ((x_coord_read < touchscreen.last_x - ADC_delta_for_change ) || (x_coord_read > touchscreen.last_x + ADC_delta_for_change))   ) //&& x_coord_read<930
		{
			touchscreen.touched_x  = x_coord_read;
			touchscreen.last_x = x_coord_read;
			
			// Wenn trotz Kalibrierung zu grosse Midi-werte berechnet wurden...
			if (midix>127)
				// Senden einer Midi-Message mit den neuen Koordinaten
				controlChange(midi_channel,midi_touch_offset + 0,127);  // midi_touch_offset + 0 = xCooord!
			else if (midix<1)
				controlChange(midi_channel,midi_touch_offset + 0,0); // midi_touch_offset + 0 = xCooord!
			else
				// Senden einer Midi-Message mit den neuen Koordinaten
				controlChange(midi_channel,midi_touch_offset + 0,(uint8_t)midix); // midi_touch_offset + 0 = xCooord!
			
			// Setze Displayflag dass Display neu gezeichnet werden muss
			touchscreen.FLAG_Display_change = 1;
		}

		// Prüfe auf Abweichung vom letzten gemessenen Y-Touchpanelwert
		if( ((y_coord_read < touchscreen.last_y - ADC_delta_for_change) || (y_coord_read > touchscreen.last_y + ADC_delta_for_change)) ) //&& y_coord_read<930 
		{
			touchscreen.touched_y  = y_coord_read;
			touchscreen.last_y = y_coord_read;
		
			// Wenn trotz Kalibrierung zu grosse Midi-werte berechnet wurden...
			if (midiy>127)     
				// Senden einer Midi-Message mit den neuen Koordinaten 
				controlChange(midi_channel,midi_touch_offset + 1,127); // midi_touch_offset + 1 = yCooord!
			else if (midiy<1)
				controlChange(midi_channel,midi_touch_offset + 0,0); // midi_touch_offset + 0 = yCooord!
			else
				// Senden einer Midi-Message mit den neuen Koordinaten
				controlChange(midi_channel,midi_touch_offset + 1,(uint8_t)midiy); // midi_touch_offset + 1 = yCooord!
			
			// Setze Displayflag dass Display neu gezeichnet werden muss
			touchscreen.FLAG_Display_change = 1;
		}
	}

}

// Source: 
// http://www.stm32circle.com/forum/viewtopic.php?id=1043

uint16_t MedianFilter(uint16_t* values)
{
    uint16_t Sorted[ADC_TOUCH_SAMPLES] ; //We only need so sort half of array
    uint16_t v;
    uint8_t i, j;

    Sorted[0] = values[0];
    
    for(i = 1; i < ADC_TOUCH_SAMPLES; i++)
    {
        v = values[i];
        j = i;
        for(; j > 0; j--){
            if(v > Sorted[j-1] ) break;
            Sorted[j] = Sorted[j-1];
        }
        Sorted[j] = v;
    }
	//Sorted[4] = values[4];
//	Sorted[5] = values[5];
 //   Sorted[6] = values[6];
    
    return Sorted[((ADC_TOUCH_SAMPLES+1) >> 1)-1];
}		






//--------------------Display Funktionen---------------------

//Fehlerkorregiertes Ausgabemuster
// Ledreihe-595-Pinindex ,  1-1 , 2-4 , 3-2, 4-5 , 5-3, 6-6
const uint8_t correct_matrix_touch_an_alt[6] = { 1, 4, 2, 5, 3, 6 };

void multiplex_display_row(uint8_t row)
{
	//set_output_74hc595(1, 0x00);
	
	// Deaktiviere Multiplex-Transistor des letzten durchlaufes
	if(row != 0)
		set_output_74hc595_pin(1, 1<<(correct_matrix_touch_an_alt[row-1]), 0); 
	else
		set_output_74hc595_pin(1, 1<<(correct_matrix_touch_an_alt[Reihen_Anz-1]), 0);
	
	// Setze neuen Multiplex-Transistor
	set_output_74hc595_pin(1, 1<<correct_matrix_touch_an_alt[row], 1);
//	set_output_74hc595(0,0xff);

}


/*

void multiplex_display_row(uint8_t row)
{
	//set_output_74hc595(1, 0x00);
	
	// Deaktiviere Multiplex-Transistor des letzten durchlaufes
	if(row != 0)
		set_output_74hc595_pin(1, 1<<(row-1), 0); 
	else
		set_output_74hc595_pin(1, 1<<(Reihen_Anz-1), 0);
	
	// Setze neuen Multiplex-Transistor
	set_output_74hc595_pin(1, 1<<row, 1);

}

*/

void Display_SetParabel(uint16_t x_ad_value,uint16_t y_ad_value)
{
	uint16_t helligkeit;
 

	for(uint16_t y=0;y<6;y++)
	{	
		// X-Leds berechnen
		for(uint16_t x=0;x<8;x++)
		{

			double bla = exp(pow(-y,2)/20024.0);
			
			
			helligkeit = 4095 * (      bla - ( (bla*pow(x,2))/20024.0  )+ (bla*(pow(x,4))/801921152.0)     );


			TLC5940_SetGS( y ,  x , helligkeit);
			//helligkeit = exp(1);
		}
	}

/*	for(uint16_t i=0;i<8;i++)
	{
		TLC5940_SetGS(yLED, i , 4095);
	}	
	for(uint16_t i=0;i<6;i++)
	{	
		TLC5940_SetGS(i, xLED , 4095);	
	}
	*/
}


void Display_SetCross(uint8_t xLED,uint8_t yLED)
{
	for(uint16_t i=0;i<8;i++)
	{
		TLC5940_SetGS(yLED, i , 4095);
	}
	for(uint16_t i=0;i<6;i++)
	{
		TLC5940_SetGS(i, xLED , 4095);	
	}
}

//--------------------Midi Funktionen---------------------



// Send a MIDI control changeDDRD 5 17
void controlChange(int channel, int controller, uint8_t value) {
   midiMsg(channel+0xB0, controller, value);
}

// Send a general MIDI message
void midiMsg(int cmd, int data1, int data2) {
  //digitalWrite(outPin,HIGH);  // indicate we're sending MIDI data
 // printf("%02x",cmd);
 // printf("%02x",data1);
 // printf("%02x",data2);
  uart_putchar(cmd,NULL);
  uart_putchar(data1,NULL);
  uart_putchar(data2,NULL);

  
  //bit_print(cmd,8);
  //bit_print(data1,8);
  //bit_print(data2,8);
  //digitalWrite(outPin,LOW);
}


//--------------------ADC Funktionen---------------------

void ADC_Init(void) {
 
  uint16_t result;
 
  ADMUX = (0<<REFS1) | (1<<REFS0);      // VCC als Referenzspannung nutzen
  // Bit ADFR ("free running") in ADCSRA steht beim Einschalten
  // schon auf 0, also single conversion
  ADCSRA = (1<<ADPS1) | (1<<ADPS0);     // Frequenzvorteiler
  ADCSRA |= (1<<ADEN);                  // ADC aktivieren
 
  /* nach Aktivieren des ADC wird ein "Dummy-Readout" empfohlen, man liest
     also einen Wert und verwirft diesen, um den ADC "warmlaufen zu lassen" */
  for( int i = 0; i<20; i++)
  {
  ADMUX = (ADMUX & ~(0x1F)) | (TOUCH_Y2 & 0x1F);
  ADCSRA |= (1<<ADSC);                  // eine ADC-Wandlung 
  while (ADCSRA & (1<<ADSC) ) {}        // auf Abschluss der Konvertierung warten
  /* ADCW muss einmal gelesen werden, sonst wird Ergebnis der nächsten
     Wandlung nicht übernommen. */
  result = ADCW;
  }
}  

uint16_t ADC_Read( uint8_t channel ) 
{
  // Kanal waehlen, ohne andere Bits zu beeinflußen
  ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F);
  ADCSRA |= (1<<ADSC);            // eine Wandlung "single conversion"
  while (ADCSRA & (1<<ADSC) ) {}  // auf Abschluss der Konvertierung warten
  return ADCW;                    // ADC auslesen und zurückgeben
}


uint8_t read_Touch_ADC_values( uint16_t* x_ADC_values, uint16_t* y_ADC_values)
{
//	PORTC &= ~0b01000000;

	uint16_t x_coord_read;
	uint16_t y_coord_read;

	uint8_t  x_validvalues = 0;
	uint8_t  y_validvalues = 0;

	uint16_t x_ADC[ADC_TOUCH_SAMPLES] ={};
	uint16_t y_ADC[ADC_TOUCH_SAMPLES] ={};

	///////////////////////////////////////////////////
	// X-Koord-READ
	// Lese Touchspannungswerte ein
 	// read_Touch_ADC_values( &x_coord_read, &y_coord_read );
	
 	setOutput(TOUCH_DDR, TOUCH_X1);
	setOutput(TOUCH_DDR, TOUCH_X2);
	setInput(TOUCH_DDR, TOUCH_Y1);	
	setInput(TOUCH_DDR, TOUCH_Y2);	

	setHigh(TOUCH_PORT, TOUCH_X1);
	setLow(TOUCH_PORT, TOUCH_X2);

	 _delay_ms(2);   // Kurze Pause dass sich Pins eingepegeln können

	x_coord_read = 1023-ADC_Read(TOUCH_Y1); 	 //  read Y2 

	for (int i =0;i<ADC_TOUCH_SAMPLES;i++)  // x Koord. aus [ADC_TOUCH_SAMPLES] Messungen bestimmen 
	{
		x_coord_read = 1023-ADC_Read(TOUCH_Y1); 	 //  read Y2 
		x_ADC[i] = x_coord_read; 
		// Prüfen auf gültige Werte
		// x/y_coord_read > validcoord : Wurde Touchpanel überhaupt gedrückt?
		// x/y_coord_read < 750/900 :    Wurde Touchpanel am Rand gedrückt ? => ungültige Werte da dort unlinear
		if(x_coord_read > valid_ADC_val_xmin && x_coord_read < valid_ADC_val_xmax  )  
		{
			// Speichere Werte im Wertearray
			x_ADC_values[i] = x_coord_read;
			
			x_validvalues++;
		}
		else
		{
			break;
		}
	}

	///////////////////////////////////////////////////
	// y-Koord-READ
	// Lese Touchspannungswerte ein
 	// read_Touch_ADC_values( &x_coord_read, &y_coord_read );

  	setOutput(TOUCH_DDR, TOUCH_Y1);
	setOutput(TOUCH_DDR, TOUCH_Y2);
	setInput(TOUCH_DDR, TOUCH_X1);	
	setInput(TOUCH_DDR, TOUCH_X2);	

	setHigh(TOUCH_PORT, TOUCH_Y1);
	setLow(TOUCH_PORT, TOUCH_Y2);
 
	_delay_ms(2);  // Kurze Pause dass sich Pins eingepegeln können

	for (int i =0;i<ADC_TOUCH_SAMPLES;i++)  // y Koord. aus [ADC_TOUCH_SAMPLES] Messungen bestimmen 
	{

		y_coord_read = 1023-ADC_Read(TOUCH_X2); // read X2
		y_ADC[i] = y_coord_read; 
		// Prüfen auf gültige Werte
		// x/y_coord_read > validcoord : Wurde Touchpanel überhaupt gedrückt?
		// x/y_coord_read < 750/900 :    Wurde Touchpanel am Rand gedrückt ? => ungültige Werte da dort unlinear
		if(y_coord_read > valid_ADC_val_ymin  && y_coord_read < valid_ADC_val_ymax)  
		{
			// Speichere Werte im Wertearray
			y_ADC_values[i] = y_coord_read;
			
			y_validvalues++;
		}
		else
		{
			break;
		}
	}


 	if( x_validvalues == ADC_TOUCH_SAMPLES && y_validvalues == ADC_TOUCH_SAMPLES )
		return 1;
	else
		return 0;
}


// --------------------------------UART--------------------------------
 
void uart_init(void)
{
    /* hier µC spezifischen Code zur Initialisierung */
    /* des UART einfügen... s.o. im AVR-GCC-Tutorial */
 
    // Beispiel: 
    //
    // myAVR Board 1.5 mit externem Quarz Q1 3,6864 MHz
    // 9600 Baud 8N1
 

 
    UCSRB |= (1<<TXEN) | (1<<RXEN) | (1<<RXCIE);    // UART TX und RX einschalten + RX interrupt einschalten
    UCSRC |= (1<<URSEL)|(3<<UCSZ0);    // Asynchron 8N1 
 
    UBRRH = (uint8_t)( UART_UBRR_CALC( UART_BAUD_RATE, F_CPU ) >> 8 );
    UBRRL = (uint8_t)UART_UBRR_CALC( UART_BAUD_RATE, F_CPU );
}
 

ISR(USART_RXC_vect) 
{
	// Zwischenspeichern des UDR-RX-Registers
    uint8_t RX_Byte_received = UDR;
	static uint8_t receivecounter = 0;

	static uint8_t input_unit_id;
	uint8_t Value;
	uint8_t xLED;
	uint8_t yLED;

	if ( ( ( RX_Byte_received & 0b11110000 ) == 0b10110000 ) && receivecounter == 0) 	 // Controllchange und erstes Byte empfangen
	{
		receivecounter++;
	}
	else if(receivecounter == 1)														// Controllchange und zweites Byte empfangen
	{
		input_unit_id = RX_Byte_received;	// unit_id empfangen!
		receivecounter++;
	}
	else if(receivecounter == 2) 														// Controllchange und drittes Byte empfangen
	{
		Value = RX_Byte_received;			// Wert von unit_id empfangen!

		// Gib nun anhand der input_unit_id den Wert an den entsprechenden Darstellungseinheiten aus.

		if ( input_unit_id < 20 ) // Taster 
		{
			set_output_74hc595_pin(button[input_unit_id].outputexpander_addr, button[input_unit_id].ledpin,Value);
		}
		else if ( input_unit_id >= 20 && input_unit_id <40) //  Encoder
		{
			//schreibe_Encoder_Leds(Value);
			encoder[input_unit_id - 20].value = Value;
		}
		else if ( input_unit_id == 41 ) //  TouchY
		{
			
			touchscreen.FLAG_Display_change = 1;	// Setze Flag dass LED-Matrix neu gezeichnet werden muss
			yLED = Value/22;						// Umrechnung von Midi-Wert in LED-Koordinate
			touchscreen.last_LED_y = yLED;			// Setze neue LED-Koordinate auf berechnete neue Koordinate
		}
		else if ( input_unit_id == 40 ) //  TouchX
		{
			touchscreen.FLAG_Display_change = 1;	// Setze Flag dass LED-Matrix neu gezeichnet werden muss
			xLED = Value/17;						// Umrechnung von Midi-Wert in LED-Koordinate
			touchscreen.last_LED_x = xLED;			// Setze neue LED-Koordinate auf berechnete neue Koordinate
		}


		receivecounter = 0;
	}



}
 
void sendString(char s[])
{
   int i = 0;
   
   //don't get stuck if it is a bad string
   while(i<64)
   {
      if( s[i] == '\0' ) break; //quit on string terminator
      uart_putchar(s[i++],stdout);
   }
}



// c. Definition der Ausgabefunktion
int uart_putchar( char c, FILE *stream )
{
    if( c == '\n' )
        uart_putchar( '\r', stream );
 
    loop_until_bit_is_set( UCSRA, UDRE );
    UDR = c;
    return 0;
}


/*
void calibrate_Touch()
{
	
	
	uint16_t x_coord_read;
	uint16_t y_coord_read;
	uint32_t x_coord = 0;
	uint32_t y_coord = 0;
	_delay_ms(2000);
	do
	{
		read_Touch_ADC_values( &x_coord_read, &y_coord_read );
		printf("%d %d\n",x_coord_read,y_coord_read);
		_delay_ms(2000);
		
	}while ( (x_coord_read < validcoord) || ( y_coord_read < validcoord));

	//////////////////////////////7

	printf("Unterer linker Punkt:\n");
	for(int i = 0; i<4;i++)
	{
		do
		{
		read_Touch_ADC_values( &x_coord_read, &y_coord_read );
		//printf("Gelesen %d %d\n",x_coord_read,y_coord_read);
		_delay_ms(1000);
		}while ( (x_coord_read < validcoord) || ( y_coord_read < validcoord));

		printf("%i.ter Pkt %d %d\n",i+1,x_coord_read,y_coord_read);
		x_coord += 	x_coord_read;		
		y_coord += 	y_coord_read;
	}
	x_min_y_min[0] = x_coord / 4;
	x_min_y_min[1] = y_coord / 4;
	printf("Calibriert %d %d\n",x_min_y_min[0],x_min_y_min[1]);

	////////////////////////////////////////////

	_delay_ms(4000);
	x_coord = 0;
	y_coord = 0;
	printf("\n\nUnterer rechter Punkt:\n");
	for(int i = 0; i<4;i++)
	{
		do
		{
		read_Touch_ADC_values( &x_coord_read, &y_coord_read );
		//printf("Gelesen %d %d\n",x_coord_read,y_coord_read);
		_delay_ms(1000);
		}while ( (x_coord_read < validcoord) || ( y_coord_read < validcoord));

		printf("%i.ter Pkt %d %d\n",i+1,x_coord_read,y_coord_read);
		x_coord += 	x_coord_read;		
		y_coord += 	y_coord_read;
	}
	x_max_y_min[0] = x_coord / 4;
	x_max_y_min[1] = y_coord / 4;
	printf("Calibriert %d %d\n",x_max_y_min[0],x_max_y_min[1]);

	////////////////////////////////////////////

	_delay_ms(4000);
	x_coord = 0;
	y_coord = 0;
	printf("\n\nOberer rechter Punkt:\n");
	for(int i = 0; i<4;i++)
	{
		do
		{
		read_Touch_ADC_values( &x_coord_read, &y_coord_read );
		//printf("Gelesen %d %d\n",x_coord_read,y_coord_read);
		_delay_ms(1000);
		}while ( (x_coord_read < validcoord) || ( y_coord_read < validcoord));

		printf("%i.ter Pkt %d %d\n",i+1,x_coord_read,y_coord_read);
		x_coord += 	x_coord_read;		
		y_coord += 	y_coord_read;
	}
	x_max_y_max[0] = x_coord / 4;
	x_max_y_max[1] = y_coord / 4;
	printf("Calibriert %d %d\n",x_max_y_max[0],x_max_y_max[1]);

	////////////////////////////////////////////

	_delay_ms(4000);
	x_coord = 0;
	y_coord = 0;
	printf("\n\nOberer linker Punkt:\n");
	for(int i = 0; i<4;i++)
	{
		do
		{
		read_Touch_ADC_values( &x_coord_read, &y_coord_read );
		//printf("Gelesen %d %d\n",x_coord_read,y_coord_read);
		_delay_ms(1000);
		}while ( (x_coord_read < validcoord) || ( y_coord_read < validcoord));

		printf("%i.ter Pkt %d %d\n",i+1,x_coord_read,y_coord_read);
		x_coord += 	x_coord_read;		
		y_coord += 	y_coord_read;
	}
	x_min_y_max[0] = x_coord / 4;
	x_min_y_max[1] = y_coord / 4;
	printf("Calibriert %d %d\n",x_min_y_max[0],x_min_y_max[1]);

}
*/

