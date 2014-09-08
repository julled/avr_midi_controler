// Buttons

struct button_t
{
	volatile uint8_t key_state;                         // entprellter key state:
	                                              	
	uint8_t key_pressed;                                // Wurde Taster als gedrückt markiert

	// 2-Bit-Counter = ct1 | ct0 
	uint8_t ct0;					
	uint8_t ct1;

	uint8_t inputexpander_addr;		// Index des Input-Portexpanders an dem der Taster angeschlossen ist
	uint8_t pin;					// Pin an dem der Taster angeschlossen ist. Markierung des einsprechenden Bits im Byte

	uint8_t outputexpander_addr;	// Index des Output-Portexpanders an dem die zum Taster dazugehörige LED angeschlossen ist
	uint8_t ledpin;					// Pin an dem die LED angeschlossen ist. Markierung des einsprechenden Bits im Byte
};



// Potis

struct potentiometer_t
{
	volatile uint16_t value;                   

	uint8_t adc_channel;
};  

// Touchscreen

struct touchscreen_t
{
	uint16_t touched_x;		// Koordinaten des aktuellen Druckpunktes
	uint16_t touched_y;		// Koordinaten des aktuellen Druckpunktes
	uint16_t last_x;		// Koordinaten des vorherigen Druckpunktes
	uint16_t last_y;		// Koordinaten des vorherigen Druckpunktes
	uint16_t last_LED_x;	// LED-Koordinaten des vorherigen Druckpunktes
	uint16_t last_LED_y;	// LED-Koordinaten des vorherigen Druckpunktes
	uint8_t FLAG_Display_change;	// Flag ob Display neu gezeichnet werden muss

} ; 

// Encoder

struct encoder_t
{
	volatile uint8_t value ;	// Speichert wert des Encoders
	int8_t last;				// Temp-Var zur Bestimmung der Drehrichtung
	volatile int8_t enc_delta; 	// Var. zum speichern der Drehrichtung	

	uint8_t inputexpander_addr ;// Index des Input-Expanders an dem der Drehgeber angeschlossen ist
	uint8_t pinA ;				// Pin an dem A angeschlossen ist. Markierung des einsprechenden Bits im Byte 
	uint8_t pinB ;				// Pin an dem A angeschlossen ist. Markierung des einsprechenden Bits im Byte 

	uint8_t led_ring_outputexpander_addr; 			// Index des Output-Expanders an dem der LED-Ring angeschlossen ist
	uint8_t led_ring_multiplex_outputexpander_addr;	// Index des Output-Expanders an dem der Multiplextransistor angeschlossen ist
	uint8_t led_ring_multiplex_pin;					// Pin an dem der Transistor angeschlossen ist. Markierung des einsprechenden Bits im Byte 
} ;
