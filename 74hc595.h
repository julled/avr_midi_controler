
 
#pragma once

#include <stdint.h>
#include <avr/io.h>

// Festlegung der Chipanzahl
#define Output_expander_count  3 //5

#define RCK_DDR DDRC
#define RCK_PORT PORTC
#define RCK_PIN PC7

uint8_t Output_Expander_Data[Output_expander_count];





void write_74hc595(uint8_t data);

void set_output_74hc595(uint8_t port_addr, uint8_t data);

void set_output_74hc595_pin(uint8_t port_addr, uint8_t pin, uint8_t true_false);

void set_all_output_74hc595(uint8_t data);

void write_all_74hc595();

void init_74hc595();
