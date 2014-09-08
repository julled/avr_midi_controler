
 
#pragma once

#include <stdint.h>
#include <avr/io.h>

// Festlegung der Chipanzahl
#define Input_expander_count 4 

#define PL_DDR DDRD
#define PL_PORT PORTD
#define PL_PIN PD6

#define CE_DDR DDRD
#define CE_PORT PORTD
#define CE_PIN PD7

#define MISO_DDR DDRB
#define MISO_PORT PORTB
#define MISO_PIN PB6

void read_74hc165();

uint8_t Input_Expander_Data[Input_expander_count];

void read_all_74hc165();

void init_74hc165();
