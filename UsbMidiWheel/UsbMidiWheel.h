#pragma once

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdio.h>

#include "Descriptors.h"

#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>

/* Timer preload values for 16 MHz /64 */
#define TIMER_PRELOAD_10MS		0xF63C
#define TIMER_PRELOAD_20MS		0xEC78
#define TIMER_PRELOAD_50MS		0xCF2C
#define TIMER_PRELOAD_100MS		0x9E58

#define LED_RX_ON		PORTB |=  (1 << PB0)
#define LED_RX_OFF		PORTB &= ~(1 << PB0)
#define LED_RX_TOGGLE	PORTB ^=  (1 << PB0)
#define LED_TX_ON		PORTD |=  (1 << PD5)
#define LED_TX_OFF		PORTD &= ~(1 << PD5)
#define LED_TX_TOGGLE	PORTD ^=  (1 << PD5)

/* User LED is high-side driven */
#define USER_LED_ON		PORTC |=  (1 << PC7)
#define USER_LED_OFF	PORTC &= ~(1 << PC7)
#define USER_LED_TOGGLE	PORTC ^=  (1 << PC7)

/* Inputs have internal pull-up resistor enabled */
#define ENCODER_A_PIN				(1 << PD0)	/* Encoder A @ D3/SCL */
#define ENCODER_B_PIN				(1 << PD1)	/* Encoder B @ D2/SDA */
#define ENCODER_A_VALUE				((PIND & ENCODER_A_PIN) ? 0 : 0x01)
#define ENCODER_B_VALUE				((PIND & ENCODER_B_PIN) ? 0 : 0x02)

#define ENCODER_SW_PIN				(1 << PD2)	/* Encoder Button @ D0/RX */
#define ENCODER_SW_PIN_PRESSED		(!(PIND & ENCODER_SW_PIN))

typedef struct Command_type 
{
	enum EncoderState { 
		ENCODER_NONE = 0,
		ENCODER_RIGHT = 1,
		ENCODER_LEFT = -1
	} Rotation;
	
	enum PushbuttonState {
		PUSHBUTTON_NONE = 0,
		PUSHBUTTON_PRESSED = 1,
		PUSHBUTTON_RELEASED = -1		
	} Enter;
		
} Command_type;

void EVENT_USB_Device_Connect (void);
void EVENT_USB_Device_Disconnect (void);
void EVENT_USB_Device_ConfigurationChanged (void);
